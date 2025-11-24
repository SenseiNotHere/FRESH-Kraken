from __future__ import annotations

import math
import typing

import wpilib

from commands2 import Subsystem
from magicbot import feedback
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

from constants import DrivingConstants, ModuleConstants, AutoConstants
from commands.holonomicdrive import HolonomicDrive

from wpimath import applyDeadband
import commands2


class DriveSubsystem(Subsystem):
    def __init__(self, maxSpeedScaleFactor=None) -> None:
        super().__init__()

        self.isAutoDriving = False  # <----- PathPlanner flag

        if maxSpeedScaleFactor is not None:
            assert callable(maxSpeedScaleFactor)

        self.maxSpeedScaleFactor = maxSpeedScaleFactor

        enabledChassisAngularOffset = 0 if DrivingConstants.kAssumeZeroOffsets else 1

        self.frontLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontLeftDriving,
            turningCANId=DrivingConstants.kFrontLeftTurning,
            feedbackDeviceId=DrivingConstants.kFrontLeftCANCoder,
            feedbackInverted=ModuleConstants.kTurningEncoderInverted,
            chassisAngularOffset=DrivingConstants.kFrontLeftChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
        )

        self.frontRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kFrontRightDriving,
            turningCANId=DrivingConstants.kFrontRightTurning,
            feedbackDeviceId=DrivingConstants.kFrontRightCANCoder,
            feedbackInverted=ModuleConstants.kTurningEncoderInverted,
            chassisAngularOffset=DrivingConstants.kFrontRightChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
        )

        self.backLeft = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackLeftDriving,
            turningCANId=DrivingConstants.kBackLeftTurning,
            feedbackDeviceId=DrivingConstants.kBackLeftCANCoder,
            feedbackInverted=ModuleConstants.kTurningEncoderInverted,
            chassisAngularOffset=DrivingConstants.kBackLeftChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
        )

        self.backRight = PhoenixSwerveModule(
            drivingCANId=DrivingConstants.kBackRightDriving,
            turningCANId=DrivingConstants.kBackRightTurning,
            feedbackDeviceId=DrivingConstants.kBackRightCANCoder,
            feedbackInverted=ModuleConstants.kTurningEncoderInverted,
            chassisAngularOffset=DrivingConstants.kBackRightChassisAngularOffset * enabledChassisAngularOffset,
            turnMotorInverted=ModuleConstants.kTurningMotorInverted,
        )

        # Gyro
        self.gyro = navx.AHRS.create_spi()
        self._lastGyroAngleTime = 0
        self._lastGyroAngle = 0
        self._lastGyroState = "ok"

        # Slew filters
        self.currentTranslationDir = 0.0
        self.currentTranslationMag = 0.0

        self.magLimiter = SlewRateLimiter(DrivingConstants.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(DrivingConstants.kRotationalSlewRate)
        self.prevTime = wpilib.Timer.getFPGATimestamp()

        # Odometry
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
            self.getPose,                # Pose supplier
            self.resetOdometry,         # PP will call this at start
            self.getRobotRelativeSpeeds,   # Robot-relative speeds
            lambda speeds, ff: self._ppDrive(speeds),   # Drive for auto
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
            ),
            AutoConstants.config,
            self.shouldFlipPath,
            self
        )

    def _ppDrive(self, speeds: ChassisSpeeds):
        """Internal: PathPlanner robot-relative driving."""
        self.isAutoDriving = True
        self.drive(
            speeds.vx,
            speeds.vy,
            speeds.omega,
            fieldRelative=False,   # MUST be robot-relative
            rateLimit=False,       # MUST be no slew limiting
            square=False           # MUST be linear input
        )
        self.isAutoDriving = False

    def getRobotRelativeSpeeds(self) -> ChassisSpeeds:
        return DrivingConstants.kDriveKinematics.toChassisSpeeds(
            (
                self.frontLeft.getState(),
                self.frontRight.getState(),
                self.backLeft.getState(),
                self.backRight.getState(),
            )
        )

    def shouldFlipPath(self):
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    def periodic(self) -> None:
        if self.simPhysics is not None:
            self.simPhysics.periodic()

        pose = self.odometry.update(
            self.getGyroHeading(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
        SmartDashboard.putNumber("x", pose.x)
        SmartDashboard.putNumber("y", pose.y)
        SmartDashboard.putNumber("heading", pose.rotation().degrees())
        self.field.setRobotPose(pose)

    def getHeading(self) -> Rotation2d:
        return self.getPose().rotation()

    def getPose(self) -> Pose2d:
        return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
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
        self.odometryHeadingOffset = Rotation2d(0)

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.getPose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)
        self.odometry.resetPosition(
            pose.rotation(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            newPose,
        )

    def stop(self):
        self.arcadeDrive(0, 0)

    def arcadeDrive(self, xSpeed: float, rot: float, assumeManualInput: bool = False):
        self.drive(xSpeed, 0, rot, False, False, square=assumeManualInput)

    def rotate(self, rotSpeed) -> None:
        self.arcadeDrive(0, rotSpeed)

    def drive(self, xSpeed, ySpeed, rot, fieldRelative, rateLimit, square=False):
        if self.isAutoDriving:
            rateLimit = False
            square = False

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
            inputTranslationDir = math.atan2(ySpeed, xSpeed)
            inputTranslationMag = math.hypot(xSpeed, ySpeed)

            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(
                    DrivingConstants.kDirectionSlewRate / self.currentTranslationMag
                )
            else:
                directionSlewRate = 500.0

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
        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            desiredStates, DrivingConstants.kMaxMetersPerSecond
        )
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.backLeft.setDesiredState(rl)
        self.backRight.setDesiredState(rr)

    def resetEncoders(self) -> None:
        self.frontLeft.resetEncoders()
        self.backLeft.resetEncoders()
        self.frontRight.resetEncoders()
        self.backRight.resetEncoders()

    def getGyroHeading(self) -> Rotation2d:
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

        return Rotation2d.fromDegrees(
            self._lastGyroAngle * DrivingConstants.kGyroReversed
        )

    def getTurnRate(self) -> float:
        return self.gyro.getRate() * DrivingConstants.kGyroReversed

    def getTurnRateDegreesPerSec(self) -> float:
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