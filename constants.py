import math

from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians

from phoenix6.hardware import talon_fx
from phoenix6.signals import NeutralModeValue

from pathplannerlib.config import RobotConfig, ModuleConfig, DCMotor


# Motor Specs

class KrakenX60:
    kFreeSpeedRpm = 5800
    kMaxSpeedMetersPerSecond = 2.8


# Drivetrain / Chassis Constants

class DrivingConstants:
    # CAN IDs – Drive Motors
    kFrontLeftDriving = 7
    kFrontRightDriving = 5
    kBackLeftDriving = 1
    kBackRightDriving = 3

    # CAN IDs – Turning Motors
    kFrontLeftTurning = 8
    kFrontRightTurning = 6
    kBackLeftTurning = 2
    kBackRightTurning = 4

    # CAN IDs – Absolute Encoders
    kFrontLeftTurningEncoder = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder = 1
    kBackRightTurningEncoder = 2

    # Physical Limits
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    # Slew Rate Limiting
    kDirectionSlewRate = 2.0      # rad/s
    kMagnitudeSlewRate = 1.8      # percent/s
    kRotationalSlewRate = 2.0     # percent/s

    # Robot Geometry
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(+kWheelBase / 2, +kTrackWidth / 2),   # Front Left
        Translation2d(+kWheelBase / 2, -kTrackWidth / 2),   # Front Right
        Translation2d(-kWheelBase / 2, +kTrackWidth / 2),   # Back Left
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),   # Back Right
    ]

    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Absolute Encoder Usage
    kAssumeZeroOffsets = False

    kFrontLeftChassisAngularOffset = 0.0
    kFrontRightChassisAngularOffset = 0.0
    kBackLeftChassisAngularOffset = 0.0
    kBackRightChassisAngularOffset = 0.0

    # Gyro
    kGyroReversed = -1

    # Lock Deadbands
    kLockVxDeadband = 0.05      # m/s
    kLockVyDeadband = 0.05      # m/s
    kLockOmegaDeadband = 0.10  # rad/s

# Individual Swerve Module Constants

class ModuleConstants:
    # Motor / Encoder Inversion
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Absolute Encoder Offsets (rotations)
    kFrontLeftTurningEncoderOffset = 0.264892578125
    kFrontRightTurningEncoderOffset = 0.22265625
    kBackLeftTurningEncoderOffset = -0.080078125
    kBackRightTurningEncoderOffset = 0.09521484375

    # Gear Ratios / Mechanics
    kDrivingMotorPinionTeeth = 14
    kDrivingMotorReduction = 6.12
    kTurningMotorReduction = 12.6

    kWheelDiameterMeters = 0.0965
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # Derived Drive Values
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60.0
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    # Turning Encoder Conversion
    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0

    kTurningEncoderPositionPIDMinInput = 0.0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    # Sync / Drift Control
    kFusedAngleRefreshSeconds = 0.25
    kTurningKalmanGain = 0.05
    kTurningDriftDegrees = 10.0

    kTurningSyncMaxVelocity = 0.5
    kDrivingSyncMaxVelocity = 0.2

    # Drive PID + FF
    kDrivingP = 0.3
    kDrivingI = 0.0
    kDrivingD = 0.0
    kDrivingS = 0.0
    kDrivingV = 0.124
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps

    kDrivingMinOutput = -1.0
    kDrivingMaxOutput = 1.0

    # Turning PID + FF
    kTurningP = 3.0
    kTurningI = 0.0
    kTurningD = 0.0
    kTurningS = 0.1
    kTurningV = 2.49
    kTurningA = 0.0
    kTurningFF = 0.15

    kTurningDeadbandRot = 0.002
    kTurningMinOutput = -1.0
    kTurningMaxOutput = 1.0

    # Motion Magic
    kMotionMagicCruiseVelocity = 25.0
    kMotionMagicAcceleration = 100.0

    # Neutral Modes
    kDrivingMotorIdleMode = talon_fx.signals.NeutralModeValue(
        NeutralModeValue.BRAKE
    )
    kTurningMotorIdleMode = talon_fx.signals.NeutralModeValue(
        NeutralModeValue.COAST
    )

    # Current Limits
    kDrivingMotorCurrentLimit = 70
    kDrivingMotorStatorCurrentLimit = 120
    kTurningMotorCurrentLimit = 40
    kTurningStatorCurrentLimit = 60

    # Misc / Coupling
    kDrivingMinSpeedMetersPerSecond = 0.01
    kSteerDriveCouplingRatio = 3.857142857142857
    kSteerKs = 0.1

    kSteerHoldDeadband = math.radians(0.25) * (
        1.0 / ((2 * math.pi) / kTurningMotorReduction)
    )


# Operator Interface

class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05


# Autonomous / PathPlanner

class AutoConstants:
    # PathPlanner Robot Config
    moduleConfig = ModuleConfig(
        maxDriveVelocityMPS=DrivingConstants.kMaxMetersPerSecond,
        driveMotor=DCMotor.krakenX60(),
        driveCurrentLimit=ModuleConstants.kDrivingMotorCurrentLimit,
        numMotors=4,
        wheelRadiusMeters=ModuleConstants.kWheelDiameterMeters / 2,
        wheelCOF=1.0
    )

    config = RobotConfig(
        massKG=60.0,
        MOI=8.0,
        moduleConfig=moduleConfig,
        moduleOffsets=DrivingConstants.kModulePositions
    )

    config.maxModuleSpeed = DrivingConstants.kMaxMetersPerSecond
    config.driveBaseRadius = 0.45
    config.maxCentripetalAcceleration = 3.0

    # Driver Feel
    kUseSqrtControl = True

    # Trajectory Limits
    kMaxMetersPerSecond = 1.2
    kMaxAccelerationMetersPerSecondSquared = 3.5

    kMaxAngularSpeedRadiansPerSecond = 5.0
    kMaxAngularSpeedRadiansPerSecondSquared = 25.0

    # Holonomic PID
    kPXController = 5.0
    kPYController = 5.0
    kPThetaController = 6.0

    kIXController = 0.0
    kIYController = 0.0
    kIThetaController = 0.0

    kDXController = 0.0
    kDYController = 0.0
    kDThetaController = 0.0

    # Theta Constraints
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )

# Shooter Constants

class ShooterConstants:
    # CAN IDs
    kMotor1CANID = 1
    kMotor2CANID = 2

    # RPM Limits
    kMinRPM = 600        # minimum sensible non-zero RPM
    kMaxRPM = 6000

    # Control Gains (Velocity)
    # Scaled for RPM-based control
    kFF = 1.4 / 10000
    kP = 5.0 / 10000
    kD = 0.0 / 10000

