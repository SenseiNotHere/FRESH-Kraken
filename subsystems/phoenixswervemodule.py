import math
from commands2 import Subsystem
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue, SensorDirectionValue
from phoenix6.controls import VelocityVoltage, PositionVoltage, MotionMagicVoltage
from phoenix6.orchestra import Orchestra
from wpilib import Timer, DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants


DEBUG_FUSED_ANGLE = True
DEBUG_TARGET_ANGLE = False

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
        self.steerFusedAngle = FusedTurningAngle(modulePlace)
        self.nextFuseTime = 0.0

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
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
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
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turningConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if turnMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        turningConfig.slot0.k_p = ModuleConstants.kTurningP
        turningConfig.slot0.k_i = ModuleConstants.kTurningI
        turningConfig.slot0.k_d = ModuleConstants.kTurningD
        self.turningMotor.configurator.apply(turningConfig)

        # Current limits
        # Driving Current Limits
        drivingCurrentLimits = CurrentLimitsConfigs()
        drivingCurrentLimits.supply_current_limit = ModuleConstants.kDrivingMotorCurrentLimit
        drivingCurrentLimits.stator_current_limit = ModuleConstants.kDrivingMotorStatorCurrentLimit
        drivingCurrentLimits.supply_current_limit_enable = True
        drivingCurrentLimits.stator_current_limit_enable = True
        self.drivingMotor.configurator.apply(drivingCurrentLimits)

        # Turning Current Limits
        turningCurrentLimits = CurrentLimitsConfigs()
        turningCurrentLimits.supply_current_limit = ModuleConstants.kTurningMotorCurrentLimit
        turningCurrentLimits.stator_current_limit = ModuleConstants.kTurningStatorCurrentLimit
        turningCurrentLimits.supply_current_limit_enable = True
        turningCurrentLimits.stator_current_limit_enable = True
        self.turningMotor.configurator.apply(turningCurrentLimits)

        # Control requests
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)
        self.turning_request = MotionMagicVoltage(0).with_slot(0)

        # Initial alignment
        self.resetEncoders()

        # Orchestra stuff
        self.orchestra = Orchestra([self.drivingMotor, self.turningMotor])

    # Encoder Sync

    def resetEncoders(self) -> None:
        """
        Resets the driving motor encoder to zero
        """
        self.drivingMotor.set_position(0)


    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        if now < self.nextFuseTime:
            return
        self.nextFuseTime = now + ModuleConstants.kFusedAngleRefreshSeconds

        relative, absolute = self.turningMotor.get_position(), self.canCoder.get_absolute_position()
        if absolute.status.is_ok() and relative.status.is_ok():
            self.steerFusedAngle.observe(
                absolute.value,
                relative.value / ModuleConstants.kTurningMotorReduction
            )

        elif not absolute.status.is_ok():
            self.steerFusedAngle.complain("absolute encoder unavailable")
        elif not relative.status.is_ok():
            self.steerFusedAngle.complain("relative encoder unavailable")

    # State / Odometry

    def getTurningPosition(self) -> float:
        """
        :return: The current turning position of the swerve module in radians.
        """
        motor_rot = self.steerFusedAngle.to_absolute_rotations(
            self.turningMotor.get_position().value / ModuleConstants.kTurningMotorReduction
        )
        return 2 * math.pi * motor_rot

    def getState(self) -> SwerveModuleState:
        """
        :return: The current state of the swerve module.
        """
        motor_rps = self.drivingMotor.get_velocity().value
        wheel_mps = motor_rps * self.driveMotorRpsToMps
        angle = self.getTurningPosition() - self.chassisAngularOffset
        return SwerveModuleState(wheel_mps, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """
        :return: The current position of the swerve module.
        """
        motor_rot = self.drivingMotor.get_position().value
        wheel_meters = motor_rot * self.driveMotorRotToMeters
        angle = self.getTurningPosition() - self.chassisAngularOffset
        return SwerveModulePosition(wheel_meters, Rotation2d(angle))

    # Optimize

    def _optimizeState(self, desired: SwerveModuleState) -> SwerveModuleState:
        """
        Optimize the desired state to minimize rotation.

        :param desired: The desired SwerveModuleState.
        :return: The optimized SwerveModuleState.
        """
        current_angle = Rotation2d(self.getTurningPosition())
        target_angle = desired.angle
        flip = False

        delta = target_angle.radians() - current_angle.radians()

        # this loop can be optimized, but let's first make sure it works
        while delta > math.pi / 2:
            delta -= math.pi
            flip = not flip
        while delta < -math.pi / 2:
            delta += math.pi
            flip = not flip

        optimized_angle = Rotation2d(current_angle.radians() + delta)
        optimized_speed = -desired.speed if flip else desired.speed
        return SwerveModuleState(optimized_speed, optimized_angle)

    # Control

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """
        Sets the desired state for the swerve module.

        :param desiredState: The desired state to set.
        :type desiredState: SwerveModuleState
        """
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
        steering_goal = self.steerFusedAngle.to_relative_rotations(
            optimized.angle.radians() / (2 * math.pi)
        )

        if DEBUG_TARGET_ANGLE:
            SmartDashboard.putNumber(f"swerveAngle_{self.modulePlace}/desired", desiredState.angle.degrees() / 360)
            SmartDashboard.putNumber(f"swerveAngle_{self.modulePlace}/optimized", optimized.angle.degrees() / 360)
            SmartDashboard.putNumber(f"swerveAngle_{self.modulePlace}/ozffset", self.steerFusedAngle.relative_minus_absolute)
            SmartDashboard.putNumber(f"swerveAngle_{self.modulePlace}/total_with_offset", steering_goal)
            denominator = desiredState.speed if desiredState.speed != 0 else 999999
            SmartDashboard.putNumber(f"swerveAngle_{self.modulePlace}/zflip", optimized.speed / denominator)

        self.turningMotor.set_control(
            self.position_request.with_position(
                steering_goal * ModuleConstants.kTurningMotorReduction
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


class FusedTurningAngle:
    """
    Converts the absolute rotations of the relative encoder into absolute rotations of absolute, and vice versa.
    (by observing the values these angles take)
    """
    def __init__(self, place: str):
        self.place = place
        self.relative_minus_absolute = 0.0
        self.not_ready = "no observations"

    def to_relative_rotations(self, absolute_rotations: float) -> float:
        return absolute_rotations + self.relative_minus_absolute

    def to_absolute_rotations(self, relative_rotations: float) -> float:
        return relative_rotations - self.relative_minus_absolute

    def observe(self, absolute_rotations: float, relative_rotations: float) -> None:
        """
        Makes an observation of the distance between absolute and relative rotation sensors.

        :absolute_rotations: (float): the reading of absolute encoder (in rotations)
        :relative_rotations: (float): the reading of relative encoder (in rotations)
        """
        if self.not_ready:
            self.relative_minus_absolute = relative_rotations - absolute_rotations
            self.complain("")
            return

        observation = relative_rotations - absolute_rotations
        while observation - self.relative_minus_absolute > 0.5:
            observation -= 1.0  # wrap around
        while observation - self.relative_minus_absolute < -0.5:
            observation += 1.0  # wrap around
        if ModuleConstants.kTurningKalmanGain > 0:
            correction = ModuleConstants.kTurningKalmanGain * (observation - self.relative_minus_absolute)
            self.relative_minus_absolute += correction

        if DEBUG_FUSED_ANGLE:
            SmartDashboard.putNumber(f"fusedAngle_{self.place}/absolute", absolute_rotations)
            SmartDashboard.putNumber(f"fusedAngle_{self.place}/relative", relative_rotations)
            SmartDashboard.putNumber(f"fusedAngle_{self.place}/offset", self.relative_minus_absolute)


    def complain(self, reason):
        if reason != self.not_ready:
            SmartDashboard.putString(f"fusedAngle_{self.place}/not_ready", reason)
            self.not_ready = reason
