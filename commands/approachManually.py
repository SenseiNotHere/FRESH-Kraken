from __future__ import annotations

from commands.approach import Tunable
from commands.aimtodirection import AimToDirectionConstants
from commands.gotopoint import GoToPointConstants

import commands2
from wpimath.geometry import Rotation2d, Translation2d
from wpilib import Timer, SmartDashboard, SendableChooser

import math
import typing


class ApproachManually(commands2.Command):

    def __init__(
        self,
        camera,
        drivetrain,
        speed: typing.Callable[[], float],
        specificHeadingDegrees=None,
        reverse=False,
        settings: dict | None=None,
        cameraMinimumFps=4.0,
        dashboardName="apmn"
    ):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specificHeadingDegrees: do you want the robot to face in a very specific direction? then specify it
        :param speed: function to get positive speed, even if camera is on the back of your robot (for the latter case set reverse=True)
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        :param cameraMinimumFps: what is the minimal number of **detected** frames per second expected from this camera
        """
        super().__init__()
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"

        self.drivetrain = drivetrain
        self.camera = camera
        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.reverse = reverse
        self.speed = speed  # ensure that the speed is between 0.0 and 1.0
        if not callable(speed):
            self.speed = lambda: speed

        assert cameraMinimumFps > 0, f"non-positive cameraMinimumFps={cameraMinimumFps}"
        self.frameTimeoutSeconds = 1.0 / cameraMinimumFps

        # setting the target heading in a way that works for all cases
        self.targetDegrees = specificHeadingDegrees
        if specificHeadingDegrees is None:
            self.targetDegrees = lambda: self.drivetrain.getHeading().degrees()
        elif not callable(specificHeadingDegrees):
            self.targetDegrees = lambda: specificHeadingDegrees

        # state
        self.targetDirection = None
        self.lastSeenObjectTime = None
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenDistanceToTag = None
        self.everSawObject = False
        self.tReachedGlidePath = 0.0  # time when aligned to the tag and desired direction for the first time
        self.tReachedFinalApproach = 0.0  # time when reached the final approach
        self.xyReachedFinalApproach = Translation2d(0, 0)
        self.lostTag = ""
        self.finished = ""

        self.initTunables(settings, dashboardName)


    def isReady(self, minRequiredObjectSize=0.3):
        return self.camera.hasDetection() and self.camera.getA() > minRequiredObjectSize


    def initTunables(self, settings, prefix):
        self.KPMULT_TRANSLATION = Tunable(settings, prefix, "GainTran", 0.6, (0.1, 8.0))  # gain for how quickly to move
        self.KPMULT_ROTATION = Tunable(settings, prefix, "GainRot", 0.8, (0.1, 8.0))  # gail for how quickly to rotate

        self.tunables = [
            self.KPMULT_TRANSLATION,
            self.KPMULT_ROTATION,
        ]

    def initialize(self):
        for t in self.tunables:
            t.fetch()

        print(f"ApproachTag: translation gain value {self.KPMULT_TRANSLATION.value}")

        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())

        self.everSawObject = False
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenDistanceToTag = 999
        self.lastSeenObjectTime = Timer.getFPGATimestamp()

        self.tReachedGlidePath = 0.0  # time when aligned to the tag and desired direction for the first time
        self.tReachedFinalApproach = 0.0  # time when reached the final approach
        self.xyReachedFinalApproach = Translation2d(0, 0)
        self.lostTag = ""
        self.finished = ""

        SmartDashboard.putString("command/c" + self.__class__.__name__, "running")


    def isFinished(self) -> bool:
        return False   # never


    def end(self, interrupted: bool):
        self.drivetrain.arcadeDrive(0, 0)


    def execute(self):
        now = Timer.getFPGATimestamp()

        # 0. look at the camera
        self.updateVision(now)

        # 1. how many degrees are left to turn? (and recommended rotation speed)
        rotationSpeed, degreesLeftToRotate = self.getGyroBasedRotationSpeed()

        # 2. how far from the glide path? (and recommended left translation speed)
        leftSpeed = self.getVisionBasedSwerveLeftSpeed(now)

        # 3. the forward speed
        fwdSpeed = self.speed()
        fwdSpeed = fwdSpeed * abs(fwdSpeed)

        # 4. if we have not seen that object in a while (or about to lose it from sight), go slower
        if self.everSawObject:
            visionOld = (now - self.lastSeenObjectTime) / (0.5 * self.frameTimeoutSeconds)
            closeToEdge = abs(self.lastSeenObjectX) / 5.0 if self.lastSeenObjectX * rotationSpeed > 0 else 0.0
            leftSpeed *= max(0.0, 1.0 - visionOld)
            rotationSpeed *= max(0.25, 1.0 - closeToEdge)

        # 5. drive!
        if self.reverse:
            self.drivetrain.drive(-fwdSpeed, -leftSpeed, rotationSpeed, fieldRelative=False, rateLimit=True)
        else:
            self.drivetrain.drive(fwdSpeed, leftSpeed, rotationSpeed, fieldRelative=False, rateLimit=True)


    def getGyroBasedRotationSpeed(self):
        # 1. how many degrees are left to turn?
        currentDirection = self.drivetrain.getHeading()
        rotationRemaining = self.targetDirection - currentDirection
        degreesRemaining = rotationRemaining.degrees()
        # (optimize: do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degreesRemaining > 180:
            degreesRemaining -= 360
        while degreesRemaining < -180:
            degreesRemaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportionalSpeed = self.KPMULT_ROTATION.value * AimToDirectionConstants.kP * abs(degreesRemaining)
        if AimToDirectionConstants.kUseSqrtControl:
            proportionalSpeed = math.sqrt(0.5 * proportionalSpeed)  # will match the non-sqrt value when 50% max speed

        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        turnSpeed = min([proportionalSpeed, 1.0])
        if degreesRemaining < 0:
            turnSpeed = -turnSpeed

        return turnSpeed, degreesRemaining


    def getVisionBasedSwerveLeftSpeed(self, now):
        # can we trust the last seen object?
        if not (self.lastSeenObjectSize > 0):
            return 0.0  # the object is not yet there, hoping that this is temporary

        # where are we?
        robotX, robotY = self.localize()

        # which speed to use to reduce robotY
        leftSpeed = self.computeProportionalSpeed(abs(robotY))
        if robotY > 0:
            leftSpeed = -leftSpeed

        return leftSpeed


    def localize(self):
        """
        localize the robot camera in the frame of the tag
        """
        distanceToTag = self.lastSeenDistanceToTag

        # trigonometry: how many meters on the left is our tag? (if negative, then it's on the right)
        angle = Rotation2d.fromDegrees(self.lastSeenObjectX)
        y = -distanceToTag * angle.sin()

        distanceToFinalApproach = distanceToTag
        return -distanceToFinalApproach, -y


    def computeProportionalSpeed(self, distance) -> float:
        kpMultTran = self.KPMULT_TRANSLATION.value
        velocity = distance * GoToPointConstants.kPTranslate * kpMultTran
        if GoToPointConstants.kUseSqrtControl:
            velocity = math.sqrt(0.5 * velocity * kpMultTran)
        if velocity > 1.0:
            velocity = 1.0
        if velocity < GoToPointConstants.kMinTranslateSpeed:
            velocity = GoToPointConstants.kMinTranslateSpeed
        return velocity


    def computeTagDistanceFromTagSizeOnFrame(self, objectSizePercent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        return math.sqrt(0.2 * 0.2 / (1.70 * 0.01 * objectSizePercent))
        # note: Arducam w OV9281 (and Limelight 3 / 4) is 1.70 sq radians (not 1.33)


    def updateVision(self, now):
        if self.camera.hasDetection():
            x = self.camera.getX()
            a = self.camera.getA()
            if x != 0 and a > 0 and a < 3:
                self.lastSeenDistanceToTag = self.computeTagDistanceFromTagSizeOnFrame(a)
                self.lastSeenObjectTime = now
                self.lastSeenObjectSize = a
                self.lastSeenObjectX = x
                if not self.everSawObject:
                    self.everSawObject = True
