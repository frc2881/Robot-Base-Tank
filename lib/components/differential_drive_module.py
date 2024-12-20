from typing import TYPE_CHECKING
from wpimath import units
from wpilib import SmartDashboard
from rev import SparkBase, SparkBaseConfig, SparkLowLevel, SparkMax, SparkRelativeEncoder
from lib.classes import DifferentialDriveModuleConfig, MotorIdleMode
from lib import utils, logger
if TYPE_CHECKING: import constants

class DifferentialDriveModule:
  def __init__(
    self,
    config: DifferentialDriveModuleConfig,
    constants: "constants.Subsystems.Drive.DifferentialDriveModule"
  ) -> None:
    self._config = config
    self._constants = constants

    self._baseKey = f'Robot/Drive/DifferentialDriveModules/{self._config.location.name}'
    self._drivingTargetSpeed: units.meters_per_second = 0
 
    self._drivingMotor = SparkMax(self._config.drivingMotorCANId, SparkLowLevel.MotorType.kBrushless)
    self._drivingMotorConfig = SparkBaseConfig()
    (self._drivingMotorConfig
      .setIdleMode(SparkBaseConfig.IdleMode.kBrake)
      .smartCurrentLimit(self._constants.kDrivingMotorCurrentLimit)
      .secondaryCurrentLimit(self._constants.kDrivingMotorCurrentLimit)
      .inverted(self._config.isInverted))
    (self._drivingMotorConfig.encoder
      .positionConversionFactor(self._constants.kDrivingEncoderPositionConversionFactor)
      .velocityConversionFactor(self._constants.kDrivingEncoderVelocityConversionFactor))
    if self._config.leaderMotorCANId is not None:
      self._drivingMotorConfig.follow(self._config.leaderMotorCANId)
    utils.setSparkConfig(
      self._drivingMotor.configure(
        self._drivingMotorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters
      )
    )

    self._drivingEncoder = self._drivingMotor.getEncoder()
    self._drivingEncoder.setPosition(0)

    utils.addRobotPeriodic(self._updateTelemetry)

  def getMotor(self) -> SparkMax:
    return self._drivingMotor
  
  def getEncoder(self) -> SparkRelativeEncoder:
    return self._drivingEncoder

  def getPosition(self) -> float:
    return self._drivingEncoder.getPosition()

  def setIdleMode(self, motorIdleMode: MotorIdleMode) -> None:
    idleMode = SparkBaseConfig.IdleMode.kCoast if motorIdleMode == MotorIdleMode.Coast else SparkBaseConfig.IdleMode.kBrake
    utils.setSparkConfig(self._drivingMotor.configure(SparkBaseConfig().setIdleMode(idleMode), SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters))
    
  def reset(self) -> None:
    self._drivingEncoder.setPosition(0)

  def _updateTelemetry(self) -> None:
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Target', self._drivingTargetSpeed)
    SmartDashboard.putNumber(f'{self._baseKey}/Driving/Speed/Actual', self._drivingEncoder.getVelocity())
