import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import DifferentialDriveKinematics
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PPLTVController
from pathplannerlib.pathfinding import PathConstraints
from photonlibpy.photonPoseEstimator import PoseStrategy
from lib import logger, utils
from lib.classes import PID, MotorControllerType, DifferentialModuleConstants, DifferentialModuleConfig, DifferentialModuleLocation, PoseSensorConfig, PoseSensorLocation

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)
PATHPLANNER_ROBOT_CONFIG = RobotConfig.fromGUISettings()

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(23)
    kWheelBase: units.meters = units.inchesToMeters(27.5)

    kTranslationSpeedMax: units.meters_per_second = 4.8
    kRotationSpeedMax: units.radians_per_second = 2 * math.pi # type: ignore

    _differentialModuleConstants = DifferentialModuleConstants(
      wheelDiameter = units.inchesToMeters(3.0),
      drivingMotorControllerType = MotorControllerType.SparkMax,
      drivingMotorCurrentLimit = 50,
      drivingMotorReduction = 8.46
    )

    kDifferentialModuleConfigs: tuple[DifferentialModuleConfig, ...] = (
      DifferentialModuleConfig(DifferentialModuleLocation.LeftFront, 2, None, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.LeftCenter, 3, 2, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.LeftRear, 4, 2, True, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.RightFront, 5, None, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.RightCenter, 6, 5, False, _differentialModuleConstants),
      DifferentialModuleConfig(DifferentialModuleLocation.RightRear, 7, 5, False, _differentialModuleConstants)
    )

    kDriveKinematics = DifferentialDriveKinematics(kTrackWidth)

    kPathPlannerRobotConfig = PATHPLANNER_ROBOT_CONFIG
    kPathPlannerController = PPLTVController(0.02)
    kPathFindingConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kDriftCorrectionControllerPID = PID(0.01, 0, 0)
    kDriftCorrectionPositionTolerance: float = 0.5
    kDriftCorrectionVelocityTolerance: float = 0.5

    kTargetAlignmentControllerPID = PID(0.075, 0, 0)
    kTargetAlignmentPositionTolerance: float = 1.0
    kTargetAlignmentVelocityTolerance: float = 1.0
    kTargetAlignmentCarpetFrictionCoeff: float = 0.2
    kTargetAlignmentHeadingAdjustment: units.degrees = 180.0

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

  class Localization:
    kSingleTagStandardDeviations: tuple[float, ...] = (1.0, 1.0, 2.0)
    kMultiTagStandardDeviations: tuple[float, ...] = (0.5, 0.5, 1.0)
    kMaxPoseAmbiguity: units.percent = 0.2

class Sensors:
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    _poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    _fallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      PoseSensorConfig(
        PoseSensorLocation.Front,
        Transform3d(
          Translation3d(units.inchesToMeters(0), units.inchesToMeters(0), units.inchesToMeters(0)),
          Rotation3d(units.degreesToRadians(0), units.degreesToRadians(0), units.degreesToRadians(0))
        ), _poseStrategy, _fallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      ),
    )

  class Camera:
    kStreams: dict[str, str] = {
      "Front": "http://10.28.81.6:1184/?action=stream",
      "Driver": "http://10.28.81.6:1188/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

class Game:
  class Commands:
    kAutoMoveTimeout: units.seconds = 4.0

  class Field:
    kAprilTagFieldLayout = APRIL_TAG_FIELD_LAYOUT
    kLength = APRIL_TAG_FIELD_LAYOUT.getFieldLength()
    kWidth = APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
    kBounds = (Translation2d(0, 0), Translation2d(kLength, kWidth))

    class Targets:
      kBlueTarget = APRIL_TAG_FIELD_LAYOUT.getTagPose(7) or Pose3d()
      kRedTarget = APRIL_TAG_FIELD_LAYOUT.getTagPose(4) or Pose3d()

      kTargetTransform = Transform3d(
        units.inchesToMeters(6.0),
        units.inchesToMeters(12.0),
        units.inchesToMeters(24),
        Rotation3d()
      )
