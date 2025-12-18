import math
from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from phoenix6.signals import (
    NeutralModeValue,
    InvertedValue,
    SensorDirectionValue,
)
from phoenix6.controls import VelocityVoltage, PositionVoltage
from phoenix6.orchestra import Orchestra
from wpilib import Timer, DriverStation
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants


class PhoenixSwerveModule(Subsystem):
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        turnMotorInverted: bool,
        driveMotorInverted: bool,
        canCoderCANId: int,
        canCoderInverted: bool,
        canCoderOffset: float,
        chassisAngularOffset: float,
        modulePlace: str,
    ) -> None:
        super().__init__()

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())
        self.modulePlace = modulePlace

        # driving motorRot -> wheelMeters
        self.driveMotorRotToMeters = (
            ModuleConstants.kWheelCircumferenceMeters
            / ModuleConstants.kDrivingMotorReduction
        )
        self.driveMotorRpsToMps = self.driveMotorRotToMeters

        # steering motorRot -> radians
        self.steerMotorRotToRad = (2 * math.pi) / ModuleConstants.kTurningMotorReduction
        self.radToSteerMotorRot = 1.0 / self.steerMotorRotToRad

        # Hardware
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)
        self.canCoder = CANcoder(canCoderCANId)
        self.canCoderOffset = canCoderOffset

        # CANcoder config
        canCoderConfig = CANcoderConfiguration()
        canCoderConfig.magnet_sensor.magnet_offset = self.canCoderOffset
        canCoderConfig.magnet_sensor.sensor_direction = (
            SensorDirectionValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.canCoder.configurator.apply(canCoderConfig)

        # Drive motor config
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = ModuleConstants.kDrivingMotorIdleMode
        drivingConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if driveMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        drivingConfig.slot0.k_p = ModuleConstants.kDrivingP
        drivingConfig.slot0.k_i = ModuleConstants.kDrivingI
        drivingConfig.slot0.k_d = ModuleConstants.kDrivingD
        self.drivingMotor.configurator.apply(drivingConfig)

        # Turn motor config
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = ModuleConstants.kTurningMotorIdleMode
        turningConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if turnMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        turningConfig.slot0.k_p = ModuleConstants.kTurningP
        turningConfig.slot0.k_i = ModuleConstants.kTurningI
        turningConfig.slot0.k_d = ModuleConstants.kTurningD
        self.turningMotor.configurator.apply(turningConfig)

        # Current Limit Configs
        # Driving motor current limit
        drivingMotorLimits = CurrentLimitsConfigs()
        drivingMotorLimits.supply_current_limit = ModuleConstants.kDrivingMotorCurrentLimit
        drivingMotorLimits.stator_current_limit = ModuleConstants.kDrivingMotorStatorCurrentLimit
        self.drivingMotor.configurator.apply(drivingMotorLimits)

        # Turning motor current limit
        turningMotorLimits = CurrentLimitsConfigs()
        turningMotorLimits.supply_current_limit = ModuleConstants.kTurningMotorCurrentLimit
        turningMotorLimits.stator_current_limit = ModuleConstants.kTurningMotorStatorCurrentLimit
        self.turningMotor.configurator.apply(turningMotorLimits)

        # Control requests
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        # Kalman timing
        self.nextSyncTime = 0.0

        # Initial alignment
        self.resetEncoders(force=True)

        # Orchestra stuff
        self.orchestra = Orchestra([self.drivingMotor, self.turningMotor])

    # Encoder Sync

    def resetEncoders(self, force: bool = False) -> None:
        self.drivingMotor.set_position(0)
        self.syncTurningEncoder(force)

    def syncTurningEncoder(self, force: bool = False) -> None:
        abs_signal = self.canCoder.get_absolute_position()
        abs_signal.refresh()
        absolute_rot = abs_signal.value  # 0–1 rotations

        current_motor_rot = self.turningMotor.get_position().value
        target_motor_rot = absolute_rot * ModuleConstants.kTurningMotorReduction

        diff = current_motor_rot - target_motor_rot
        full_rotations = round(diff / ModuleConstants.kTurningMotorReduction)
        adjusted_target = (
            target_motor_rot
            + full_rotations * ModuleConstants.kTurningMotorReduction
        )

        error = current_motor_rot - adjusted_target

        if force:
            self.turningMotor.set_position(adjusted_target)
            return

        if ModuleConstants.kTurningKalmanGain > 0:
            correction = -ModuleConstants.kTurningKalmanGain * error
            self.turningMotor.set_position(current_motor_rot + correction)

    def periodic(self) -> None:
        # Kalman disabled? Do nothing.
        if ModuleConstants.kTurningKalmanGain <= 0:
            return

        now = Timer.getFPGATimestamp()
        if now < self.nextSyncTime:
            return

        self.nextSyncTime = now + ModuleConstants.kTurningSyncIntervalSeconds
        self.syncTurningEncoder()

    # State / Odometry

    def getTurningPosition(self) -> float:
        motor_rot = self.turningMotor.get_position().value
        return motor_rot * self.steerMotorRotToRad

    def getState(self) -> SwerveModuleState:
        motor_rps = self.drivingMotor.get_velocity().value
        wheel_mps = motor_rps * self.driveMotorRpsToMps
        angle = self.getTurningPosition() - self.chassisAngularOffset
        return SwerveModuleState(wheel_mps, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        motor_rot = self.drivingMotor.get_position().value
        wheel_meters = motor_rot * self.driveMotorRotToMeters
        angle = self.getTurningPosition() - self.chassisAngularOffset
        return SwerveModulePosition(wheel_meters, Rotation2d(angle))

    # Optimize

    def _optimizeState(self, desired: SwerveModuleState) -> SwerveModuleState:
        """
        Optimize the desired SwerveModuleState to minimize steering rotation.

        :param desired: The desired SwerveModuleState (speed and angle).
        """
        angle_rad = self.getTurningPosition()
        angle_rad = math.atan2(math.sin(angle_rad), math.cos(angle_rad))
        current_angle = Rotation2d(angle_rad)
        target_angle = desired.angle

        delta = target_angle.radians() - current_angle.radians()
        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi

        optimized_angle = Rotation2d(current_angle.radians() + delta)
        optimized_speed = desired.speed

        if abs(delta) > math.pi / 2:
            optimized_speed = -optimized_speed
            optimized_angle = Rotation2d(
                optimized_angle.radians() + math.pi
            )

        return SwerveModuleState(optimized_speed, optimized_angle)

    # Control

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """
        Sets the desired state of the module.
        """
        if abs(desiredState.speed) < 0.01:
            desiredState = SwerveModuleState(0.0, self.desiredState.angle)

        desired_module = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset),
        )

        optimized = self._optimizeState(desired_module)

        # Drive
        motor_rps = optimized.speed / self.driveMotorRotToMeters
        self.drivingMotor.set_control(
            self.velocity_request.with_velocity(motor_rps)
        )

        # Turn
        current_angle_rad = self.getTurningPosition()
        current_angle_rad = math.atan2(
            math.sin(current_angle_rad),
            math.cos(current_angle_rad),
        )

        target_angle_rad = optimized.angle.radians()

        delta_rad = target_angle_rad - current_angle_rad
        while delta_rad > math.pi:
            delta_rad -= 2 * math.pi
        while delta_rad < -math.pi:
            delta_rad += 2 * math.pi

        delta_motor_rot = delta_rad * self.radToSteerMotorRot
        current_motor_rot = self.turningMotor.get_position().value

        self.turningMotor.set_control(
            self.position_request.with_position(
                current_motor_rot + delta_motor_rot
            )
        )

        self.desiredState = desiredState

    # Extras

    def getTemperature(self):
        """
        :return: Returns the temperatures of the module's motors.
        """
        drivingTemp = self.drivingMotor.get_device_temp().value
        turningTemp = self.turningMotor.get_device_temp().value
        return drivingTemp, turningTemp

    def getSupplyCurrent(self):
        """
        :return: Returns the current draw of the module's motors.
        """
        drivingCurrent = self.drivingMotor.get_supply_current()
        turningCurrent = self.turningMotor.get_supply_current()
        return drivingCurrent, turningCurrent

    # Orchestra

    def loadMusic(self, path: str):
        """
        Loads a music file into the orchestra.

        :path: (str): The file path to the music file to be loaded.
        """
        self.orchestra.load_music(path)

    def playMusic(self):
        """
        Plays music using the configured orchestra.
        """
        self.orchestra.play()

    def stopMusic(self):
        """
        Stops the music playback by halting the orchestra.
        """
        self.orchestra.stop()