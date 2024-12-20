from typing import Callable
import math
from wpilib import SmartDashboard, SendableChooser
from wpilib.drive import DifferentialDrive
from wpimath import units
from wpimath.controller import PIDController
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d
from wpimath.kinematics import ChassisSpeeds, DifferentialDriveWheelSpeeds
from rev import SparkBase, SparkBaseConfig, SparkMax, SparkLowLevel
from commands2 import Subsystem, Command
from lib import utils, logger
from lib.classes import ChassisLocation, MotorIdleMode, SpeedMode, DriveOrientation, OptionState, LockState
from lib.components.differential_drive_module import DifferentialDriveModule
import constants

class DriveSubsystem(Subsystem):
  def __init__(
      self, 
      getGyroHeading: Callable[[], units.degrees]
    ) -> None:
    super().__init__()
    self._getGyroHeading = getGyroHeading
    
    self._constants = constants.Subsystems.Drive

    self._differentialDriveModules = tuple(DifferentialDriveModule(c, self._constants.DifferentialDriveModule) for c in self._constants.kDifferentialDriveModuleConfigs)

    self._drivetrain = DifferentialDrive(
      self._differentialDriveModules[ChassisLocation.FrontLeft.value].getMotor(),
      self._differentialDriveModules[ChassisLocation.FrontRight.value].getMotor()
    )

    self._leftEncoder = self._differentialDriveModules[ChassisLocation.RearLeft.value].getEncoder()
    self._rightEncoder = self._differentialDriveModules[ChassisLocation.RearRight.value].getEncoder()

    self._isDriftCorrectionActive: bool = False
    self._driftCorrectionThetaController = PIDController(
      self._constants.kDriftCorrectionThetaControllerPIDConstants.P, 
      self._constants.kDriftCorrectionThetaControllerPIDConstants.I, 
      self._constants.kDriftCorrectionThetaControllerPIDConstants.D
    )
    self._driftCorrectionThetaController.enableContinuousInput(-180.0, 180.0)
    self._driftCorrectionThetaController.setTolerance(
      self._constants.kDriftCorrectionThetaControllerPositionTolerance, 
      self._constants.kDriftCorrectionThetaControllerVelocityTolerance
    )

    self._isAlignedToTarget: bool = False
    self._targetAlignmentThetaController = PIDController(
      self._constants.kTargetAlignmentThetaControllerPIDConstants.P, 
      self._constants.kTargetAlignmentThetaControllerPIDConstants.I, 
      self._constants.kTargetAlignmentThetaControllerPIDConstants.D
    )
    self._targetAlignmentThetaController.enableContinuousInput(-180.0, 180.0)
    self._targetAlignmentThetaController.setTolerance(
      self._constants.kTargetAlignmentThetaControllerPositionTolerance, 
      self._constants.kTargetAlignmentThetaControllerVelocityTolerance
    )

    self._inputXFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputYFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)
    self._inputRotationFilter = SlewRateLimiter(self._constants.kInputRateLimitDemo)

    self._speedMode: SpeedMode = SpeedMode.Competition
    speedModeChooser = SendableChooser()
    speedModeChooser.setDefaultOption(SpeedMode.Competition.name, SpeedMode.Competition)
    speedModeChooser.addOption(SpeedMode.Demo.name, SpeedMode.Demo)
    speedModeChooser.onChange(lambda speedMode: setattr(self, "_speedMode", speedMode))
    SmartDashboard.putData("Robot/Drive/SpeedMode", speedModeChooser)

    self._orientation: DriveOrientation = DriveOrientation.Field
    orientationChooser = SendableChooser()
    orientationChooser.setDefaultOption(DriveOrientation.Field.name, DriveOrientation.Field)
    orientationChooser.addOption(DriveOrientation.Robot.name, DriveOrientation.Robot)
    orientationChooser.onChange(lambda orientation: setattr(self, "_orientation", orientation))
    SmartDashboard.putData("Robot/Drive/Orientation", orientationChooser)

    self._driftCorrection: OptionState = OptionState.Enabled
    driftCorrectionChooser = SendableChooser()
    driftCorrectionChooser.setDefaultOption(OptionState.Enabled.name, OptionState.Enabled)
    driftCorrectionChooser.addOption(OptionState.Disabled.name, OptionState.Disabled)
    driftCorrectionChooser.onChange(lambda driftCorrection: setattr(self, "_driftCorrection", driftCorrection))
    SmartDashboard.putData("Robot/Drive/DriftCorrection", driftCorrectionChooser)

    idleModeChooser = SendableChooser()
    idleModeChooser.setDefaultOption(MotorIdleMode.Brake.name, MotorIdleMode.Brake)
    idleModeChooser.addOption(MotorIdleMode.Coast.name, MotorIdleMode.Coast)
    # idleModeChooser.onChange(lambda idleMode: self._setIdleMode(idleMode))
    SmartDashboard.putData("Robot/Drive/IdleMode", idleModeChooser)

    SmartDashboard.putNumber("Robot/Drive/Chassis/Length", self._constants.kWheelBase)
    SmartDashboard.putNumber("Robot/Drive/Chassis/Width", self._constants.kTrackWidth)
    SmartDashboard.putNumber("Robot/Drive/Speed/Max", self._constants.kTranslationSpeedMax)

    self.resetEncoder()

  def periodic(self) -> None:
    self._updateTelemetry()

  def resetEncoder(self) -> None:
    self._leftEncoder.setPosition(0) 
    self._rightEncoder.setPosition(0)

  def getDistance(self) -> float:
    return self._leftEncoder.getPosition()

  def getSpeeds(self) -> ChassisSpeeds:
    wheelSpeeds = DifferentialDriveWheelSpeeds(
      self._leftEncoder.getVelocity(),
      self._rightEncoder.getVelocity()
    )
    return self._constants.kDifferentialDriveKinematics.toChassisSpeeds(wheelSpeeds)

  def driveWithSpeeds(self, speeds: ChassisSpeeds) -> None:
    wheelSpeeds = self._constants.kDifferentialDriveKinematics.toWheelSpeeds(speeds)
    self._drivetrain.tankDrive(wheelSpeeds.left, wheelSpeeds.right)

  def driveWithControllerCommand(
      self, 
      getLeftY: Callable[[], float], 
      getRightX: Callable[[], float]
    ) -> Command:
    return self.run(
      lambda: self._arcadeDrive(getLeftY(), getRightX())
    ).withName("DriveWithController")

  def getLeftEncoderPosition(self) -> float:
    return self._leftEncoder.getPosition()
  
  def getRightEncoderPosition(self) -> float:
    return self._rightEncoder.getPosition()
  
  def _arcadeDrive(self, speed: float, rotation: float) -> None:
    self._drivetrain.arcadeDrive(speed, rotation, True)

  def _setIdleMode(self, idleMode: MotorIdleMode) -> None:
    # TODO: implement idleMode change on motor controllers
    SmartDashboard.putString("Robot/Drive/IdleMode/selected", idleMode.name)

  def alignToTargetCommand(self, getRobotPose: Callable[[], Pose2d], getTargetHeading: Callable[[], units.degrees]) -> Command:
    return self.run(
      lambda: self._alignToTarget(getRobotPose().rotation().degrees())
    ).beforeStarting(
      lambda: [
        self.clearTargetAlignment(),
        self._targetAlignmentThetaController.reset(),
        self._targetAlignmentThetaController.setSetpoint(utils.wrapAngle(getTargetHeading() + self._constants.kTargetAlignmentHeadingInversion))  
      ]
    ).until(
      lambda: self._isAlignedToTarget
    ).withName("DriveSubsystem:AlignToTarget")

  def _alignToTarget(self, robotHeading: units.degrees) -> None:
    speedRotation = self._targetAlignmentThetaController.calculate(robotHeading)
    speedRotation += math.copysign(self._constants.kTargetAlignmentCarpetFrictionCoeff, speedRotation)
    if self._targetAlignmentThetaController.atSetpoint():
      speedRotation = 0
      self._isAlignedToTarget = True
    self._arcadeDrive(0, speedRotation)

  def isAlignedToTarget(self) -> bool:
    return self._isAlignedToTarget
  
  def clearTargetAlignment(self) -> None:
    self._isAlignedToTarget = False

  def reset(self) -> None:
    # self._setIdleMode(MotorIdleMode.Brake)
    self._arcadeDrive(0, 0)
    self.clearTargetAlignment()
    self.resetEncoder()
  
  def _updateTelemetry(self) -> None:
    SmartDashboard.putBoolean("Robot/Drive/IsAlignedToTarget", self._isAlignedToTarget)
    SmartDashboard.putNumber("Robot/Drive/LeftEncoder", self._leftEncoder.getPosition())
    SmartDashboard.putNumber("Robot/Drive/RightEncoder", self._rightEncoder.getPosition())
  