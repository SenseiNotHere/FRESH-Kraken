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
        drivingConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.ROTOR_SENSOR
        self.drivingMotor.configurator.apply(drivingConfig)

        #Initialize Turning Motors
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        turningConfig.slot0.k_p = 8.0
        turningConfig.slot0.k_i = 0.0
        turningConfig.slot0.k_d = 0.05

        # Fused CANcoder setup
        turningConfig.feedback.feedback_remote_sensor_id = feedbackDeviceId
        turningConfig.feedback.feedback_sensor_source = FeedbackSensorSourceValue.FUSED_CANCODER
        turningConfig.feedback.sensor_to_mechanism_ratio = 1.0
        turningConfig.feedback.rotor_to_sensor_ratio = ModuleConstants.kTurningMotorReduction

        self.turningMotor.configurator.apply(turningConfig)

        # Configure Absolute Encoders
        feedbackConfig = CANcoderConfiguration()
        feedbackConfig.magnet_sensor.sensor_direction = SensorDirectionValue.CLOCKWISE_POSITIVE if feedbackInverted else SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        self.feedbackDevice.configurator.apply(feedbackConfig)

        #Set up velocity and position requests for the motors
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Reset encoders to starting position
        self.resetEncoders()
        self.resetToAbsolute()

        current_angle_rad = self.getTurningPosition()
        self.desiredState = SwerveModuleState(0.0, Rotation2d(current_angle_rad))

    def getState(self) -> SwerveModuleState:
        motor_rps = self.drivingMotor.get_velocity().value  # motor rotations/sec
        wheel_rps = motor_rps / ModuleConstants.kDrivingMotorReduction
        velocity_mps = wheel_rps * ModuleConstants.kWheelCircumferenceMeters

        angle = self.getTurningPosition()
        return SwerveModuleState(velocity_mps, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        motor_rot = self.drivingMotor.get_position().value  # motor rotations
        wheel_rot = motor_rot / ModuleConstants.kDrivingMotorReduction
        distance_m = wheel_rot * ModuleConstants.kWheelCircumferenceMeters

        angle = self.getTurningPosition()
        return SwerveModulePosition(distance_m, Rotation2d(angle))

    def getTurningPosition(self):
        return self.turningMotor.get_position().value * 2 * math.pi

    def getCancoderPosition(self):
        position = self.feedbackDevice.get_position().value
        if position is None:
            return None
        else:
            return position

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        # If we're barely moving, keep the current desired angle to avoid jitter
        if abs(desiredState.speed) < 0.01:
            desiredState = SwerveModuleState(0.0, self.desiredState.angle)

        # Current wheel angle
        current_angle = Rotation2d(self.getTurningPosition())

        print("desiredState:", desiredState)
        print("current_angle:", current_angle)

        # Manual Optimize
        delta = desiredState.angle - current_angle
        if abs(delta.degrees()) > 90.0:
            optimized = SwerveModuleState(
                -desiredState.speed,
                desiredState.angle + Rotation2d.fromDegrees(180)
            )
        else:
            optimized = desiredState

        print("optimized:", optimized)

        # Wheel rotations target
        angle_in_rotations = optimized.angle.radians() / (2 * math.pi)

        # Drive motor
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

        self.desiredState = optimized

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(self.position_request.with_position(current_position))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Zeroes Relative Encoders."""
        self.drivingMotor.set_position(0)

    def resetToAbsolute(self):
        absolute = self.getCancoderPosition()
        if absolute is None:
            return

        self.turningMotor.set_position(absolute)
