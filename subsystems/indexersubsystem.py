from commands2 import Subsystem
from phoenix6.configs import TalonFXConfiguration, Slot0Configs
from phoenix6.hardware import TalonFX
from phoenix6.controls import VelocityVoltage, DutyCycleOut
from phoenix6.signals import NeutralModeValue, InvertedValue

from constants import IndexerConstants

class Indexer(Subsystem):
    def __init__(self, motorCANID: int, motorInverted: bool):
        super().__init__()

        # Motor Init
        self.motor = TalonFX(motorCANID)

        # Motor config
        indexerMotorConfig = TalonFXConfiguration()
        indexerMotorConfig.motor_output.neutral_mode = NeutralModeValue.COAST
        indexerMotorConfig.motor_output.inverted = (
            InvertedValue.COUNTER_CLOCKWISE_POSITIVE
            if motorInverted
            else InvertedValue.CLOCKWISE_POSITIVE
        )

        # Slot config
        indexerSlotConfig = Slot0Configs()
        indexerSlotConfig.k_p = IndexerConstants.kP
        indexerSlotConfig.k_d = IndexerConstants.kD
        indexerSlotConfig.k_v = IndexerConstants.kFF
        self.motor.configurator.apply(indexerMotorConfig)

        # Control requests
        self.velocityRequest = VelocityVoltage(0.0).with_slot(0)
        self.percentRequest = DutyCycleOut(0.0)

        # State
        self.targetRPS = 0.0
        self.enabled = False

    def periodic(self):
        if not self.enabled:
            self.motor.set_control(self.percentRequest.with_output(0.0))
            return

        self.motor.set_control(
            self.velocityRequest.with_velocity(self.targetRPS)
        )

    # API

    def enable(self):
        """
        Starts the indexer motor at the configured feed speed.
        """
        self.enabled = True
        self.targetRPS = IndexerConstants.kFeedRPS

    def stop(self):
        """
        Stops the indexer motor.
        """
        self.enabled = False
        self.targetRPS = 0.0

    def feedPercent(self, percent: float):
        """
        Sets the indexer motor to a constant percent output.
        """
        self.enabled = False
        self.motor.set_control(
            self.percentRequest.with_output(percent)
        )
