from typing import TYPE_CHECKING
from commands2 import Command, cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathPlannerPath
if TYPE_CHECKING: from robot_container import RobotContainer
from classes import AutoPath
import constants

class AutoCommands:
  def __init__(
      self,
      robot: "RobotContainer"
    ) -> None:
    self._robot = robot

    self._paths: dict[AutoPath, PathPlannerPath] = {}
    for path in AutoPath:
      self._paths[path] = PathPlannerPath.fromPathFile(path.name)

    self._addAutoOptions()

  def _move(self, path: AutoPath) -> Command:
    # return AutoBuilder.pathfindThenFollowPath(self._paths.get(path), constants.Subsystems.Drive.kPathFindingConstraints)\
    return AutoBuilder.followPath(self._paths.get(path))
  
  def _alignToTarget(self) -> Command:
    return cmd.sequence(self._robot.gameCommands.alignRobotToTargetCommand())

  def _addAutoOptions(self) -> None:
    self._robot.addAutoOption("[0] 0_", self.auto_0_)
    self._robot.addAutoOption("[2] 2_", self.auto_2_)

  def auto_0_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move0)
    ).withName("AutoCommands:[0]_")

  def auto_2_(self) -> Command:
    return cmd.sequence(
      self._move(AutoPath.Move2),
      self._alignToTarget()
    ).withName("AutoCommands:[2]_")
  
