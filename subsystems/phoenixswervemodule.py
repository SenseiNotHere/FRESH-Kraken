import math
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration
from phoenix6.signals import NeutralModeValue, InvertedValue
from phoenix6.controls import VelocityVoltage, PositionVoltage
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition

from constants import ModuleConstants


class PhoenixSwerveModule:
    def __init__(
        self,
        drivingCANId: int,
        turningCANId: int,
        turnMotorInverted: bool,
        canCoderCANId: int,
        canCoderInverted: bool,
        canCoderOffset: float,
        chassisAngularOffset: float,
        modulePlace: str,
    ) -> None:
        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())
        self.modulePlace = modulePlace

        # motorRot -> wheelMeters
        self.driveMotorRotToMeters = (
            ModuleConstants.kWheelCircumferenceMeters / ModuleConstants.kDrivingMotorReduction
        )
        self.driveMotorRpsToMps = self.driveMotorRotToMeters  # 1 rotation per second

        # steering motorRot -> radians
        self.steerMotorRotToRad = (2 * math.pi) / ModuleConstants.kTurningMotorReduction
        self.radToSteerMotorRot = 1.0 / self.steerMotorRotToRad

        # Hardware
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)
        self.canCoder = CANcoder(canCoderCANId)
        self.canCoderOffset = canCoderOffset

        # CANCoder config
        canCoderConfig = CANcoderConfiguration()
        # Offset so that abs position 0 = wheel forward for THIS module
        canCoderConfig.magnet_sensor.magnet_offset = self.canCoderOffset
        canCoderConfig.sensor_direction = (
            InvertedValue.CLOCKWISE_POSITIVE
            if canCoderInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.canCoder.configurator.apply(canCoderConfig, 0.1)

        # Drive motor config
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        drivingConfig.slot0.k_p = 0.3
        drivingConfig.slot0.k_i = 0.0
        drivingConfig.slot0.k_d = 0.0
        self.drivingMotor.configurator.apply(drivingConfig)

        # Turn motor config
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turningConfig.slot0.k_p = 4.0
        turningConfig.slot0.k_i = 0.0
        turningConfig.slot0.k_d = 0.05
        turningConfig.motor_output.inverted = (
            InvertedValue.CLOCKWISE_POSITIVE
            if turnMotorInverted
            else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.turningMotor.configurator.apply(turningConfig)

        # Control requests
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        # Start encoders aligned to CANCoder
        self.resetEncoders()

        #print(f"Device: {self.modulePlace} with position {self.canCoder.get_absolute_position().value}")

    def getTurningPosition(self) -> float:
        """
        Steering angle in radians, module frame (0 = wheel "forward" for this module).
        """
        motor_rot = self.turningMotor.get_position().value
        return motor_rot * self.steerMotorRotToRad  # rad

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module (used by kinematics/odometry)."""
        motor_rps = self.drivingMotor.get_velocity().value
        wheel_mps = motor_rps * self.driveMotorRpsToMps

        # Angle in robot frame = module angle - chassis offset
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModuleState(wheel_mps, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module (used by odometry)."""
        motor_rot = self.drivingMotor.get_position().value
        wheel_meters = motor_rot * self.driveMotorRotToMeters

        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModulePosition(wheel_meters, Rotation2d(angle))


    def syncTurningEncoder(self) -> None:
        """
        Sync the steering motor integrated encoder to the absolute CANCoder angle.
        Assumes CANCoder magnet_offset is set so that abs=0 means wheel forward.
        """
        absolute_rot = self.canCoder.get_absolute_position().value  # 0 → 1 rotations
        motor_rot = absolute_rot * ModuleConstants.kTurningMotorReduction
        self.turningMotor.set_position(motor_rot)

    def resetEncoders(self) -> None:
        """
        Reset drive position to 0 and align steering encoder to absolute.
        """
        self.drivingMotor.set_position(0)
        self.syncTurningEncoder()

    def _optimizeState(self, desired: SwerveModuleState) -> SwerveModuleState:
        """
        Re-implement WPILib's SwerveModuleState.optimize:
        choose the equivalent angle within ±90° of current to avoid spinning.
        """
        current_angle = Rotation2d(self.getTurningPosition())
        target_angle = desired.angle

        delta = target_angle.radians() - current_angle.radians()

        while delta > math.pi:
            delta -= 2 * math.pi
        while delta < -math.pi:
            delta += 2 * math.pi

        optimized_angle = Rotation2d(current_angle.radians() + delta)
        optimized_speed = desired.speed

        # If we're more than 90 degrees off, flip direction
        if abs(delta) > math.pi / 2:
            optimized_speed = -optimized_speed
            optimized_angle = Rotation2d(optimized_angle.radians() + math.pi)

        return SwerveModuleState(optimized_speed, optimized_angle)

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """
        Set the desired state for this module.
        """
        corrected = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset),
        )

        # Optimize relative to current angle
        optimized = self._optimizeState(corrected)

        motor_rps = optimized.speed / self.driveMotorRotToMeters

        # Convert desired angle to steering motor rotations
        angle_rad = optimized.angle.radians()
        motor_rot = angle_rad * self.radToSteerMotorRot

        # Send control requests
        self.drivingMotor.set_control(
            self.velocity_request.with_velocity(motor_rps)
        )
        self.turningMotor.set_control(
            self.position_request.with_position(motor_rot)
        )

        self.desiredState = desiredState

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(
            self.position_request.with_position(current_position)
        )
        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(
                speed=0, angle=self.desiredState.angle
            )

    def getTemperature(self):
        drivingTemp = self.drivingMotor.get_device_temp().value
        turningTemp = self.turningMotor.get_device_temp().value
        return drivingTemp, turningTemp
