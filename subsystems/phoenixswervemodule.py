import math
from phoenix6.hardware import TalonFX, CANcoder
from phoenix6.configs import TalonFXConfiguration, FeedbackConfigs, CANcoderConfiguration, MagnetSensorConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue, FeedbackSensorSourceValue, SensorDirectionValue
from phoenix6.controls import VelocityVoltage, PositionVoltage
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from constants import ModuleConstants

class PhoenixSwerveModule:
    def __init__(
            self,
            drivingCANId: int,
            turningCANId: int,
            feedbackDeviceId: int,
            feedbackInverted: bool,
            chassisAngularOffset: float,
            turnMotorInverted: bool,
    ) -> None:
        """
        :param drivingCANId: The CAN ID for the driving motor.
        :param turningCANId: The CAN ID for the turning motor.
        :param feedbackDeviceId: The feedback device ID.
        :param feedbackInverted: Whether the feedback should be inverted.
        :param chassisAngularOffset: Offset angle for this module.
        :param turnMotorInverted: Whether the turning motor is inverted or not.
        """

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        #Initialize the TalonFX Controllers
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)

        # Initialize feedback devices
        self.feedbackDevice = CANcoder(feedbackDeviceId)

        #Initialize Driving Motors
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # Set appropriate PID values for velocity control
        drivingConfig.slot0.k_p = 0.3
        drivingConfig.slot0.k_i = 0.0
        drivingConfig.slot0.k_d = 0.0
        self.drivingMotor.configurator.apply(drivingConfig)

        #Initialize Turning Motors
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        #Set appropriate PID values for position control
        turningConfig.slot0.k_p = 8.0
        turningConfig.slot0.k_i = 0.0
        turningConfig.slot0.k_d = 0.05
        #Use InvertedValue enum instead of bool
        turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        turningConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        turningConfig.feedback.feedback_device = feedbackDeviceId
        self.turningMotor.configurator.apply(turningConfig)

        # Configure Absolute Encoders
        feedbackConfig = CANcoderConfiguration()
        feedbackConfig.magnet_sensor.sensor_direction = SensorDirectionValue.CLOCKWISE_POSITIVE if ModuleConstants.kTurningEncoderInverted else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        self.feedbackDevice.configurator.apply(feedbackConfig)

        #Set up velocity and position requests for the motors
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Reset encoders to starting position
        self.resetEncoders()

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        velocity = self.drivingMotor.get_velocity().value
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModuleState(velocity, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current relatice position of the module."""
        distance = self.drivingMotor.get_position().value
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModulePosition(distance, Rotation2d(angle))

    def getTurningPosition(self) -> float:
        """Gets the turning motor position in radians."""
        # Convert rotations to radians (2π radians per rotation)
        return self.turningMotor.get_position().value * 2 * math.pi

    def getCancoderPosition(self):
        return 2 * math.pi * self.feedbackDevice.get_position().value

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state of the module."""

        # Apply chassis offset
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset)
        )

        # Get current wheel angle (absolute)
        cancoder_pos = self.feedbackDevice.get_position().value

        if cancoder_pos is None:
            current_angle = Rotation2d(0)
        else:
            current_angle = Rotation2d(self.getCancoderPosition())

        # Optimize using WPILib
        print("correctedDesiredState:", correctedDesiredState)
        print("current_angle:", current_angle)
        optimized = SwerveModuleState.optimize(correctedDesiredState, current_angle)
        print("optimized:", optimized)

        # Fallback if optimize fails
        if optimized is None:
            optimized = correctedDesiredState

        # Convert optimized wheel angle => motor rotations
        angle_in_rotations = (
                optimized.angle.radians() / (2 * math.pi)
                * ModuleConstants.kTurningMotorReduction
        )

        # Drive motor (m/s => motor RPS)
        self.drivingMotor.set_control(
            self.velocity_request.with_velocity(
                optimized.speed * ModuleConstants.kDrivingMotorReduction
                / ModuleConstants.kWheelCircumferenceMeters
            )
        )

        # Turn motor
        self.turningMotor.set_control(
            self.position_request.with_position(angle_in_rotations)
        )

        self.desiredState = desiredState

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(self.position_request.with_position(current_position))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Zeroes the Absolute and Relative Encoders."""
        self.drivingMotor.set_position(0)
        absolute_rotations = self.feedbackDevice.get_position().value

        if absolute_rotations is not None:
            # Set TalonFX rotor position to match absolute wheel angle
            self.turningMotor.set_position(
                absolute_rotations * ModuleConstants.kTurningMotorReduction
            )

    def absoluteZero(self):
        """Moves the wheels to Absolute Zero."""

        self.turningMotor.set_control(
            self.position_request.with_position(0)
        )