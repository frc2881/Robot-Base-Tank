import math
from wpimath import units
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose3d, Translation2d
from wpimath.kinematics import DifferentialDriveKinematics
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from navx import AHRS
from photonlibpy.photonPoseEstimator import PoseStrategy
from pathplannerlib.controller import PIDConstants as PathPlannerPIDConstants
from pathplannerlib.pathfinding import PathConstraints
from lib.classes import PIDConstants, ChassisLocation, DifferentialDriveModuleConfig, PoseSensorConfig

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

class Subsystems:
  class Drive:
    kTrackWidth: units.meters = units.inchesToMeters(23)
    kWheelBase: units.meters = units.inchesToMeters(27.5)

    kTranslationSpeedMax: units.meters_per_second = 4.8
    kRotationSpeedMax: units.radians_per_second = 2 * math.pi # type: ignore

    kInputLimitDemo: units.percent = 0.5
    kInputRateLimitDemo: units.percent = 0.33

    kDriftCorrectionThetaControllerPIDConstants = PIDConstants(0.01, 0, 0, 0)
    kDriftCorrectionThetaControllerPositionTolerance: float = 0.5
    kDriftCorrectionThetaControllerVelocityTolerance: float = 0.5

    kTargetAlignmentThetaControllerPIDConstants = PIDConstants(0.075, 0, 0, 0)
    kTargetAlignmentThetaControllerPositionTolerance: float = 1.0
    kTargetAlignmentThetaControllerVelocityTolerance: float = 1.0
    kTargetAlignmentCarpetFrictionCoeff: float = 0.2
    kTargetAlignmentHeadingInversion: units.degrees = 180.0

    kPathFollowerTranslationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFollowerRotationPIDConstants = PathPlannerPIDConstants(5.0, 0, 0)
    kPathFindingConstraints = PathConstraints(2.4, 1.6, units.degreesToRadians(540), units.degreesToRadians(720))

    kDifferentialDriveModuleConfigs: tuple[DifferentialDriveModuleConfig, ...] = (
      DifferentialDriveModuleConfig(ChassisLocation.FrontLeft, 2, None, True),
      DifferentialDriveModuleConfig(ChassisLocation.Left, 3, 2, True),
      DifferentialDriveModuleConfig(ChassisLocation.RearLeft, 4, 2, True),
      DifferentialDriveModuleConfig(ChassisLocation.FrontRight, 5, None, False),
      DifferentialDriveModuleConfig(ChassisLocation.Right, 6, 5, False),
      DifferentialDriveModuleConfig(ChassisLocation.RearRight, 7, 5, False)
    )

    kDifferentialDriveKinematics = DifferentialDriveKinematics(kTrackWidth)

    class DifferentialDriveModule:
      kWheelDiameter: units.meters = units.inchesToMeters(3.0)
      kDrivingMotorReduction: float = 8.46
      kDrivingEncoderPositionConversionFactor: float = (kWheelDiameter * math.pi) / kDrivingMotorReduction
      kDrivingEncoderVelocityConversionFactor: float = ((kWheelDiameter * math.pi) / kDrivingMotorReduction) / 60.0
      kDrivingMotorCurrentLimit: int = 50

class Sensors:
  class Gyro:
    class NAVX2:
      kComType = AHRS.NavXComType.kUSB1

  class Pose:
    kPoseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
    kFallbackPoseStrategy = PoseStrategy.LOWEST_AMBIGUITY
    kSingleTagStandardDeviations: tuple[float, ...] = (1.0, 1.0, 2.0)
    kMultiTagStandardDeviations: tuple[float, ...] = (0.5, 0.5, 1.0)
    kMaxPoseAmbiguity: units.percent = 0.2

    kPoseSensorConfigs: tuple[PoseSensorConfig, ...] = (
      # PoseSensorConfig(
      #   ChassisLocation.Front.name,
      #   Transform3d(
      #     Translation3d(units.inchesToMeters(9.62), units.inchesToMeters(4.12), units.inchesToMeters(21.25)),
      #     Rotation3d(units.degreesToRadians(0), units.degreesToRadians(-22.3), units.degreesToRadians(0.0))
      #   ), kPoseStrategy, kFallbackPoseStrategy, APRIL_TAG_FIELD_LAYOUT
      # ),
    )

  class Camera:
    kStreams: dict[str, str] = {
     # "Front": "http://10.28.81.6:1184/?action=stream",
     # "Driver": "http://10.28.81.6:1188/?action=stream"
    }

class Controllers:
  kDriverControllerPort: int = 0
  kOperatorControllerPort: int = 1
  kInputDeadband: units.percent = 0.1

APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout().loadField(AprilTagField.k2024Crescendo)

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