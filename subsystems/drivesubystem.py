from __future__ import annotations

import math
import typing

import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
)
from wpilib import SmartDashboard, Field2d, DriverStation

import swerveutils
from .phoenixswervemodule import PhoenixSwerveModule
import navx

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

from phoenix6.orchestra import Orchestra

from constants import DrivingConstants, ModuleConstants, AutoConstants
from commands.holonomicDrive import HolonomicDrive

from wpimath import applyDeadband
import commands2

class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor=None) -> None:
        super().__init__()

        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        enabledChassisAngularOffset = 0 if DrivingConstants.kAssumeZeroOffsets else 1

        self.frontLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontLeftDriving,
            turningCANId=DrivingConstants.kFrontLeftTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kFrontLeftDriveMotorInverted,
            canCoderCANId=DrivingConstants.kFrontLeftTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kFrontLeftTurningEncoderOffset,
            chassisAngularOffset=DrivingConstants.kFrontLeftChassisAngularOffset * enabledChassisAngularOffset,
            modulePlace="FL"
        )

        self.frontRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontRightDriving,
            turningCANId=DrivingConstants.kFrontRightTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kFrontRightDriveMotorInverted,
            canCoderCANId=DrivingConstants.kFrontRightTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kFrontRightTurningEncoderOffset,
            chassisAngularOffset=DrivingConstants.kFrontRightChassisAngularOffset * enabledChassisAngularOffset,
            modulePlace="FR"
        )

        self.backLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackLeftDriving,
            turningCANId=DrivingConstants.kBackLeftTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kBackLeftDriveMotorInverted,
            canCoderCANId=DrivingConstants.kBackLeftTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kBackLeftTurningEncoderOffset,
            chassisAngularOffset=DrivingConstants.kBackLeftChassisAngularOffset * enabledChassisAngularOffset,
            modulePlace="BL"
        )

        self.backRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackRightDriving,
            turningCANId=DrivingConstants.kBackRightTurning,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
            driveMotorInverted=ModuleConstants.kBackRightDriveMotorInverted,
            canCoderCANId=DrivingConstants.kBackRightTurningEncoder,
            canCoderInverted=ModuleConstants.kTurningEncoderInverted,
            canCoderOffset=ModuleConstants.kBackRightTurningEncoderOffset,
            chassisAngularOffset=DrivingConstants.kBackRightChassisAngularOffset * enabledChassisAngularOffset,
            modulePlace="BR"
        )

        #RoboRIO Gyro Sensor
        self.gyro = navx.AHRS.create_spi()
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0
        self._lastGyroState = "ok"

        #Slew rate filter variables for controlling lateral acceleration
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DrivingConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DrivingConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        #Odometry class for tracking robot pose
        self.odometry = SwerveDrive4Odometry(
            DrivingConstants.kDriveKinematics,
            Rotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        self.odometryHeadingOffset = Rotation2d(0)
        self.resetOdometry(Pose2d(0, 0, 0))

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        self.simPhysics = None

        AutoBuilder.configure(
            self.getPose,  # Robot pose supplier
            self.resetOdometry,  # Method to reset odometry (will be called if your auto has a starting pose)
            self.getRobotRelativeSpeeds,  # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            lambda speeds, feedforwards: self.drive(speeds.vx, speeds.vy, speeds.omega, True, True),
            # Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also outputs individual module feedforwards
            PPHolonomicDriveController(
                PIDConstants(
                    AutoConstants.kPXController,
                    AutoConstants.kIXController,
                    AutoConstants.kDXController
                ), # Translation PID Values
                PIDConstants(
                    AutoConstants.kPThetaController,
                    AutoConstants.kIThetaController,
                    AutoConstants.kDThetaController
                ) # Rotation PID Values
            ),
            AutoConstants.config,  # The robot configuration
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self  # Reference to this subsystem to set requirements
        )

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        """Returns the current robot-relative ChassisSpeeds"""
        return DrivingConstants.kDriveKinematics.toChassisSpeeds(
            (
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState(),
            )
        )

    def shouldFlipPath(self):
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self) -> None:
        if self.simPhysics is not None:
            self.simPhysics.periodic()

        # Periodically sync wheel angles to absolute encoders to prevent drift
        # Only syncs if drift exceeds ~5 degrees to avoid violent corrections
        current_time = wpilib.Timer.getFPGATimestamp()
        if not hasattr(self, '_lastSyncTime'):
            self._lastSyncTime = current_time
        
        if current_time - self._lastSyncTime > 1.0:  # Check every 1 second
            self.frontLeft.syncTurningEncoder()
            self.frontRight.syncTurningEncoder()
            self.backLeft.syncTurningEncoder()
            self.backRight.syncTurningEncoder()
            self._lastSyncTime = current_time

        # Update the odometry in the periodic block
        pose = self.odometry.update(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.field.setRobotPose(pose)

        # Data Puts
        # Pose
        SmartDashboard.putNumber("X Coordinate", pose.x)
        SmartDashboard.putNumber("Y Coordinate", pose.y)
        SmartDashboard.putNumber("Heading (deg)", pose.rotation().degrees())

        # Temperatures
        SmartDashboard.putNumberArray("Front Left Temp", self.frontLeft.getTemperature())
        SmartDashboard.putNumberArray("Front Right Temp", self.frontRight.getTemperature())
        SmartDashboard.putNumberArray("Back Left Temp", self.backLeft.getTemperature())
        SmartDashboard.putNumberArray("Back Right Temp", self.backRight.getTemperature())

        # Positions
        SmartDashboard.putNumber("Front Left Position", self.frontLeft.getPosition().angle.degrees())
        SmartDashboard.putNumber("Front Right Position", self.frontRight.getPosition().angle.degrees())
        SmartDashboard.putNumber("Back Left Position", self.backLeft.getPosition().angle.degrees())
        SmartDashboard.putNumber("Back Right Position", self.backRight.getPosition().angle.degrees())

    def getHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        self.gyro.reset()
        self.gyro.setAngleAdjustment(0)
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0

        self.odometry.resetPosition(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )
        self.odometryHeadingOffset = self.odometry.getPose().rotation() - self.getGyroHeading()

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation() - self.odometryHeadingOffset,
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            newPose,
        )
        self.odometryHeadingOffset += dRot

    def stop(self):
        self.arcadeDrive(0, 0)

    def arcadeDrive(
            self,
            xSpeed: float,
            rot: float,
            assumeManualInput: bool = False,
    ) -> None:
        self.drive(xSpeed, 0, rot, False, False, square=assumeManualInput)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param rotSpeed: rotation speed
        """
        self.arcadeDrive(0, rotSpeed)

    def drive(
            self,
            xSpeed: float,
            ySpeed: float,
            rot: float,
            fieldRelative: bool,
            rateLimit: bool,
            square: bool = False
    ) -> None:
        """Method to drive the robot using joystick info.

        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the
                              field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """

        if square:
            rot = rot * abs(rot)
            norm = math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed)
            xSpeed = xSpeed * norm
            ySpeed = ySpeed * norm

        if (xSpeed != 0 or ySpeed != 0) and self.maxSpeedScaleFactor is not None:
            norm = math.sqrt(xSpeed * xSpeed + ySpeed * ySpeed)
            scale = abs(self.maxSpeedScaleFactor() / norm)
            if scale < 1:
                xSpeed = xSpeed * scale
                ySpeed = ySpeed * scale

        xSpeedCommanded = xSpeed
        ySpeedCommanded = ySpeed

        if rateLimit:
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(
                    DrivingConstants.kDirectionSlewRate / self.currentTranslationMag
                )
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = wpilib.Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(
                inputTranslationDir, self.currentTranslationDir
            )
            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(
                    inputTranslationMag
                )

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(
                        self.currentTranslationDir + math.pi
                    )
                    self.currentTranslationMag = self.magLimiter.calculate(
                        inputTranslationMag
                    )

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(
                self.currentTranslationDir
            )
            ySpeedCommanded = self.currentTranslationMag * math.sin(
                self.currentTranslationDir
            )
            self.currentRotation = self.rotLimiter.calculate(rot)

        else:
            self.currentRotation = rot

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * DrivingConstants.kMaxMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * DrivingConstants.kMaxMetersPerSecond
        rotDelivered = self.currentRotation * DrivingConstants.kMaxAngularSpeed

        swerveModuleStates = DrivingConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                self.getGyroHeading(),
            )
            if fieldRelative
            else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered)
        )
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(
            SwerveModuleState(0, Rotation2d.fromDegrees(-45))
        )
        self.backLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.backRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def setModuleStates(
            self,
            desiredStates: typing.Tuple[
                SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
            ],
    ) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        self.frontLeft.resetEncoders()
        self.backLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backRight.resetEncoders()

    def getGyroHeading(self) -> Rotation2d:
        """Returns the heading of the robot, tries to be smart when gyro is disconnected

        :returns: the robot's heading as Rotation2d
        """
        now = wpilib.Timer.getFPGATimestamp()
        past = self._lastGyroAngleTime
        state = "ok"

        if not self.gyro.isConnected():
            state = "disconnected"
        else:
            if self.gyro.isCalibrating():
                state = "calibrating"
            self._lastGyroAngle = self.gyro.getAngle()
            self._lastGyroAngleTime = now

        if state != self._lastGyroState:
            SmartDashboard.putString("gyro", f"{state} after {int(now - past)}s")
            self._lastGyroState = state

        return Rotation2d.fromDegrees(self._lastGyroAngle * DrivingConstants.kGyroReversed)

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * DrivingConstants.kGyroReversed

    def getTurnRateDegreesPerSec(self) -> float:
        """Returns the turn rate of the robot (in degrees per second)

        :returns: The turn rate of the robot, in degrees per second
        """
        return self.getTurnRate() * 180 / math.pi

class BadSimPhysics(object):
    """
    this is the wrong way to do it, it does not scale!!!
    the right way is shown here: https://github.com/robotpy/examples/blob/main/Physics/src/physics.py
    and documented here: https://robotpy.readthedocs.io/projects/pyfrc/en/stable/physics.html
    (but for a swerve drive it will take some work to add correctly)
    """
    def __init__(self, drivetrain: DriveSubsystem, robot: wpilib.RobotBase):
        self.drivetrain = drivetrain
        self.robot = robot
        self.t = 0

    def periodic(self):
        past = self.t
        self.t = wpilib.Timer.getFPGATimestamp()
        if past == 0:
            return  # it was first time

        dt = self.t - past
        if self.robot.isEnabled():
            drivetrain = self.drivetrain

            states = (
                drivetrain.frontLeft.desiredState,
                drivetrain.frontRight.desiredState,
                drivetrain.backLeft.desiredState,
                drivetrain.backRight.desiredState,
            )
            speeds = DrivingConstants.kDriveKinematics.toChassisSpeeds(states)

            dx = speeds.vx * dt
            dy = speeds.vy * dt

            heading = drivetrain.getHeading()
            trans = Translation2d(dx, dy).rotateBy(heading)
            rot = (speeds.omega * 180 / math.pi) * dt

            g = drivetrain.gyro
            g.setAngleAdjustment(g.getAngleAdjustment() + rot * DrivingConstants.kGyroReversed)
            drivetrain.adjustOdometry(trans, Rotation2d())