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
    # CAN IDs - Drive Motors
    kFrontLeftDriving = 7
    kFrontRightDriving = 5
    kBackLeftDriving = 2
    kBackRightDriving = 3

    # CAN IDs - Turning Motors
    kFrontLeftTurning = 8
    kFrontRightTurning = 6
    kBackLeftTurning = 1
    kBackRightTurning = 4

    # CAN IDs - CANCoders
    kFrontLeftTurningEncoder = 4
    kFrontRightTurningEncoder = 3
    kBackLeftTurningEncoder = 1
    kBackRightTurningEncoder = 2

    # Max Robot Speeds (physical limits)
    kMaxMetersPerSecond = 3.0
    kMaxAngularSpeed = math.tau

    # Slew Rate Limiting
    kDirectionSlewRate = 2.0      # rad/s
    kMagnitudeSlewRate = 1.8      # percent/s
    kRotationalSlewRate = 2.0     # percent/s

    # Chassis Geometry
    kTrackWidth = units.inchesToMeters(26.5)
    kWheelBase = units.inchesToMeters(26.5)

    kModulePositions = [
        Translation2d(kWheelBase / 2,  kTrackWidth / 2),
        Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        Translation2d(-kWheelBase / 2,  kTrackWidth / 2),
        Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    ]

    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # Absolute Encoder Offsets
    kAssumeZeroOffsets = False

    # Only used if assumeZeroOffsets == False
    kFrontLeftChassisAngularOffset = 0
    kFrontRightChassisAngularOffset = 0
    kBackLeftChassisAngularOffset = 0
    kBackRightChassisAngularOffset = 0

    # Gyro
    kGyroReversed = -1


# Individual Module Constants

class ModuleConstants:
    # Motor / Encoder Inversion
    kTurningEncoderInverted = False
    kTurningMotorInverted = False

    kFrontLeftDriveMotorInverted = True
    kFrontRightDriveMotorInverted = False
    kBackLeftDriveMotorInverted = True
    kBackRightDriveMotorInverted = False

    # Absolute Encoder Magnet Offsets
    kFrontLeftTurningEncoderOffset = 0.264404296875
    kFrontRightTurningEncoderOffset = 0.218505859375
    kBackLeftTurningEncoderOffset = -0.0810546875
    kBackRightTurningEncoderOffset = 0.0966796875

    # Mechanical Configuration
    kDrivingMotorPinionTeeth = 14
    kDrivingMotorReduction = 6.12
    kTurningMotorReduction = 12.6

    kWheelDiameterMeters = 0.0965
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    # Derived Drive Speeds
    kDrivingMotorFreeSpeedRps = KrakenX60.kFreeSpeedRpm / 60
    kDriveWheelFreeSpeedRps = (
        kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters
    ) / kDrivingMotorReduction

    # Turning Encoder Conversion
    kTurningEncoderPositionFactor = math.tau
    kTurningEncoderVelocityFactor = math.tau / 60.0

    kTurningEncoderPositionPIDMinInput = 0
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor

    # Turning Sync / Drift Control
    kTurningSyncIntervalSeconds = 0.25
    kTurningKalmanGain = 0.0 #0.05
    kTurningDriftDegrees = 10.0

    kTurningSyncMaxVelocity = 0.5
    kDrivingSyncMaxVelocity = 0.2

    # Drive PID
    kDrivingP = 0.3
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    # Turning PID
    kTurningP = 3.0
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0.05
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    # Motion Magic (Turning)
    kTurnCruiseVelocity = 80.0
    kTurnAcceleration = 160.0
    kTurnJerk = 1600.0

    # Motor Neutral Modes
    kDrivingMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.BRAKE)
    kTurningMotorIdleMode = talon_fx.signals.NeutralModeValue(NeutralModeValue.COAST)

    # Current Limits
    kDrivingMotorCurrentLimit = 70
    kDrivingMotorStatorCurrentLimit = 120
    kTurningMotorCurrentLimit = 40
    kTurningStatorCurrentLimit = 60

    # Misc
    kDrivingMinSpeedMetersPerSecond = 0.01


# Operator Interface

class OIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05


# Autonomous / PathPlanner

class AutoConstants:
    # PathPlanner Module Config
    moduleConfig = ModuleConfig(
        maxDriveVelocityMPS=DrivingConstants.kMaxMetersPerSecond,
        driveMotor=DCMotor.krakenX60(),
        driveCurrentLimit=ModuleConstants.kDrivingMotorCurrentLimit,
        numMotors=4,
        wheelRadiusMeters=ModuleConstants.kWheelDiameterMeters / 2,
        wheelCOF=1.0
    )

    config = RobotConfig(
        massKG=60.00,
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

    # Holonomic PID (Safe to Tune)
    kPXController = 5.0
    kPYController = 5.0
    kPThetaController = 6.0

    # Holonomic PID (Danger Zone)
    kIXController = 0.0
    kIYController = 0.0
    kIThetaController = 0.0

    kDXController = 0.0
    kDYController = 0.0
    kDThetaController = 0.0

    # Theta Motion Constraints
    kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )
