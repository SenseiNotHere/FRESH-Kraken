import math
from phoenix6.hardware import TalonFX,CANcoder
from phoenix6.configs import TalonFXConfiguration, CANcoderConfiguration, CurrentLimitsConfigs
from phoenix6.signals import NeutralModeValue, InvertedValue, FeedbackSensorSourceValue
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
            modulePlace
    ) -> None:
        """
        :param drivingCANId: The CAN ID for the driving motor.
        :param turningCANId: The CAN ID for the turning motor.
        :param chassisAngularOffset: Offset angle for this module.
        :param turnMotorInverted: Whether the turning motor is inverted or not.
        """

        self.chassisAngularOffset = chassisAngularOffset
        self.desiredState = SwerveModuleState(0.0, Rotation2d())

        #Initialize the TalonFX Controllers
        self.drivingMotor = TalonFX(drivingCANId)
        self.turningMotor = TalonFX(turningCANId)
        self.canCoder = CANcoder(canCoderCANId)
        self.canCoderOffset = canCoderOffset
        self.modulePlace = modulePlace

        # Configure CANCoder
        canCoderConfig = CANcoderConfiguration()
        canCoderConfig.magnet_sensor.magnet_offset = self.canCoderOffset
        canCoderConfig.sensor_direction = InvertedValue.CLOCKWISE_POSITIVE if canCoderInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.canCoder.configurator.apply(canCoderConfig)

        #Initialize Driving Motors
        drivingConfig = TalonFXConfiguration()
        drivingConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        # Set appropriate PID values for velocity control
        drivingConfig.slot0.k_p = 0.3
        drivingConfig.slot0.k_i = 0.0
        drivingConfig.slot0.k_d = 0.0
        self.drivingMotor.configurator.apply(drivingConfig)

        # Driving Current Limits
#        drivingCurrentLimit = CurrentLimitsConfigs()
#        drivingCurrentLimit.stator_current_limit = 60
#        drivingCurrentLimit.supply_current_limit = 40
#        drivingCurrentLimit.stator_current_limit_enable = True
#        drivingCurrentLimit.supply_current_limit_enable = True
#       self.drivingMotor.configurator.apply(drivingCurrentLimit)

        #Initialize Turning Motors
        turningConfig = TalonFXConfiguration()
        turningConfig.motor_output.neutral_mode = NeutralModeValue.BRAKE
        #Set appropriate PID values for position control
        turningConfig.slot0.k_p = 8.0
        turningConfig.slot0.k_i = 0.0
        turningConfig.slot0.k_d = 0.05
        turningConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if turnMotorInverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.turningMotor.configurator.apply(turningConfig)

        # Turning Current Limits
#        turningCurrentLimit = CurrentLimitsConfigs()
#        turningCurrentLimit.stator_current_limit = 35
#        turningCurrentLimit.supply_current_limit = 20
#        turningCurrentLimit.stator_current_limit_enable = True
#        turningCurrentLimit.supply_current_limit_enable = True
#        self.turningMotor.configurator.apply(turningCurrentLimit)

        #Set up velocity and position requests for the motors
        self.velocity_request = VelocityVoltage(0).with_slot(0)
        self.position_request = PositionVoltage(0).with_slot(0)

        #Reset encoders to starting position
        self.resetEncoders()

        # Data puts
        # Temperatures
        SmartDashboard.putNumber(self.modulePlace + " Driving Temp", self.drivingMotor.get_device_temp().value)
        SmartDashboard.putNumber(self.modulePlace + " Turning Temp", self.turningMotor.get_device_temp().value)

        # Positions
        SmartDashboard.putNumber(self.modulePlace + " ABS Value", self.canCoder.get_absolute_position().value)
        SmartDashboard.putNumber(self.modulePlace + " Turning Position", self.turningMotor.get_position().value)
        SmartDashboard.putNumber(self.modulePlace + " Module Position", self.getTurningPosition())

        # Module States
        SmartDashboard.putNumber(self.modulePlace + " Actual State Speed", self.getState().speed)
        SmartDashboard.putNumber(self.modulePlace + " Actual State Angle", self.getState().angle.degrees())
        SmartDashboard.putNumber(self.modulePlace + " Desired State Speed", self.desiredState.speed)
        SmartDashboard.putNumber(self.modulePlace + " Desired State Angle", self.desiredState.angle.degrees())

        # Supply Current
        SmartDashboard.putNumber(self.modulePlace + " Driving Current", self.drivingMotor.get_supply_current().value)
        SmartDashboard.putNumber(self.modulePlace + " Turning Current", self.turningMotor.get_supply_current().value)

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module."""
        velocity = self.drivingMotor.get_velocity().value
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModuleState(velocity, Rotation2d(angle))

    def getPosition(self) -> SwerveModulePosition:
        """Returns the current position of the module."""
        distance = self.drivingMotor.get_position().value
        angle = self.getTurningPosition() - self.chassisAngularOffset

        return SwerveModulePosition(distance, Rotation2d(angle))

    def syncTurningEncoder(self) -> None:
        """Sync the turning motor integrated encoder to the absolute CANCoder angle."""
        absolute_rot = self.canCoder.get_absolute_position().value  # 0 => 1 rotations
        motor_rot = absolute_rot * ModuleConstants.kTurningMotorReduction
        self.turningMotor.set_position(motor_rot)

    def getTurningPosition(self) -> float:
        """Gets the turning motor position in radians using the relative encoder."""
        motor_rot = self.turningMotor.get_position().value
        wheel_rot = motor_rot / ModuleConstants.kTurningMotorReduction
        return wheel_rot * 2 * math.pi

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:
        """Sets the desired state for the module."""

        #print(f">>> Speed= {desiredState.speed}")
        # Debug current state
        current_angle_rad = self.getTurningPosition()
        current_angle_deg = math.degrees(current_angle_rad)
        cancoder_pos = self.canCoder.get_absolute_position().value
        motor_pos = self.turningMotor.get_position().value
        
        #print(f"[{self.drivingMotor.device_id}] DESIRED: Speed={desiredState.speed:.3f}, Angle={desiredState.angle.degrees():.1f}°")
        #print(f"[{self.drivingMotor.device_id}] CURRENT: CANCoder={cancoder_pos:.3f}rot, Motor={motor_pos:.3f}rot, Angle={current_angle_deg:.1f}°")
        
#        if abs(desiredState.speed) < ModuleConstants.kDrivingMinSpeedMetersPerSecond:
#            # If speed is too low, don't move, save power
#            inXBrake = abs(abs(desiredState.angle.degrees()) - 45) < 0.01
#            if not inXBrake:
#               print("Stopping module")
#                self.stop()
#                return

        # Apply chassis angular offset to the desired state
        correctedDesiredState = SwerveModuleState(
            desiredState.speed,
            desiredState.angle + Rotation2d(self.chassisAngularOffset)
        )

        # Get the current angle for optimization
        current_angle = Rotation2d(self.getTurningPosition())

        delta = correctedDesiredState.angle - current_angle

        if abs(delta.degrees()) > 90.0:
            optimized_speed = -correctedDesiredState.speed
            optimized_angle = correctedDesiredState.angle + Rotation2d(math.pi)
        else:
            optimized_speed = correctedDesiredState.speed
            optimized_angle = correctedDesiredState.angle

        # Convert optimized angle to rotations
        angle_in_rotations = optimized_angle.radians() / (2 * math.pi)

        #Send commands to motors
        self.drivingMotor.set_control(self.velocity_request.with_velocity(optimized_speed * ModuleConstants.kDrivingMotorReduction / ModuleConstants.kWheelCircumferenceMeters))
        self.turningMotor.set_control(self.position_request.with_position(angle_in_rotations * ModuleConstants.kTurningMotorReduction))

        self.desiredState = desiredState

#        abs_pos = self.canCoder.get_absolute_position().value
#        print(f"CanCoder ID: {self.canCoder.device_id} Position: {abs_pos}")

    def stop(self):
        """Stops the module."""
        self.drivingMotor.set_control(self.velocity_request.with_velocity(0))
        current_position = self.turningMotor.get_position().value
        self.turningMotor.set_control(self.position_request.with_position(current_position))

        if self.desiredState.speed != 0:
            self.desiredState = SwerveModuleState(speed=0, angle=self.desiredState.angle)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0 and the turning encoder to the absolute CANCoder angle."""
        self.drivingMotor.set_position(0)
        self.syncTurningEncoder()

    def getTemperature(self):
        """Gets the temperature of the module."""
        drivingTemp = self.drivingMotor.get_device_temp().value
        turningTemp = self.turningMotor.get_device_temp().value
        return drivingTemp, turningTemp