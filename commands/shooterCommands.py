from commands2 import Command
from subsystems.shootersubsystem import Shooter

class RunShooter(Command):
    def __init__(self, shooter: Shooter):
        super().__init__()
        self.shooter = shooter
        self.addRequirements(shooter)

    def initialize(self):
        self.shooter.enable()

    def end(self, interrupted):
        self.shooter.disable()

    def isFinished(self):
        return False
