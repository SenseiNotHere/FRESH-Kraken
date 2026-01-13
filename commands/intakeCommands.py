from commands2 import Command, InstantCommand
from subsystems.intakesubsystem import Intake


class RunIntake(Command):
    def __init__(self, intake: Intake, rpm: float):
        super().__init__()
        self.intake = intake
        self.rpm = rpm
        self.addRequirements(intake)

    def initialize(self):
        self.intake.enable()
        self.intake.setRPM(self.rpm)

    def end(self, interrupted: bool):
        self.intake.disable()

    def isFinished(self) -> bool:
        return False


class ReverseIntake(Command):
    def __init__(self, intake: Intake, rpm: float):
        super().__init__()
        self.intake = intake
        self.rpm = abs(rpm)
        self.addRequirements(intake)

    def initialize(self):
        self.intake.enable()
        self.intake.setRPM(-self.rpm)

    def end(self, interrupted: bool):
        self.intake.disable()

    def isFinished(self) -> bool:
        return False


class IntakeForTime(Command):
    def __init__(self, intake: Intake, rpm: float, seconds: float):
        super().__init__()
        self.intake = intake
        self.rpm = rpm
        self.seconds = seconds
        self.addRequirements(intake)

    def initialize(self):
        self.intake.enable()
        self.intake.runForTime(self.rpm, self.seconds)

    def end(self, interrupted: bool):
        self.intake.disable()

    def isFinished(self) -> bool:
        return True


class StopIntake(InstantCommand):
    def __init__(self, intake: Intake):
        super().__init__(intake.stop, intake)
