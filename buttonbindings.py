from wpilib import XboxController, PS4Controller
from commands2 import cmd, InstantCommand, RunCommand
from commands2.button import CommandGenericHID
from subsystems.drivesubsystem import DriveSubsystem
from subsystems.limelightcamera import LimelightCamera
from constants import OIConstants

from commands.reset_XY import ResetXY, ResetSwerveFront
from commands.followObject import FollowObject
from commands.setCameraPipeline import SetCameraPipeline

class ButtonBindings:
    def __init__(self, robot_container):
        """Initialize ButtonBindings with access to the robot container
        """
        self.robotContainer = robot_container
        self.robotDrive = robot_container.robotDrive
        self.driverController = robot_container.driverController
        self.limelight = robot_container.limelight

    def configureButtonBindings(self):
        """Configure button bindings for the robot."""

        # Driver Controls
        # Reset XY Position
        povUpDriverButton = self.driverController.pov(0)
        povUpDriverButton.onTrue(ResetXY(x=0.0, y=0.0, headingDegrees=0.0, drivetrain=self.robotDrive))

        # Reset Swerve Front
        povDownDriverButton = self.driverController.pov(180)
        povDownDriverButton.onTrue(ResetSwerveFront(self.robotDrive))

        # X-Break
        povLeftDriverButton = self.driverController.pov(270)
        povLeftDriverButton.whileTrue(InstantCommand(self.robotDrive.setX, self.robotDrive))

        # Follow Object
        yDriverButton = self.driverController.button(XboxController.Button.kY)
        yDriverButton.onTrue(SetCameraPipeline(self.limelight, 0))
        yDriverButton.whileTrue(FollowObject(camera=self.limelight, drivetrain=self.robotDrive))

        # Play selected song
        bButton = self.driverController.button(XboxController.Button.kB)
        bButton.onTrue(InstantCommand(lambda: self.robotDrive.playSound(self.robotContainer.songChooser.getSelected())))

        # Stop song
        aButton = self.driverController.button(XboxController.Button.kA)
        aButton.onTrue(InstantCommand(lambda: self.robotDrive.stopSound()))
