import typing
import commands2
import wpimath

from constants.swerve_constants import SwerveOIConstants
from subsystems.drive_subsystem import DriveSubsystem


class DriveCommand(commands2.CommandBase):
    def __init__(
        self,
        drive: DriveSubsystem,
        x_speed: float,
        y_speed: float,
        rot: float,
        field_relative: bool,
        rate_limit: bool
    ) -> None:
        super().__init__()

        self.drive = drive
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.rot = rot
        self.field_relative = field_relative
        self.rate_limit = rate_limit

        self.addRequirements([self.drive])

    def execute(self) -> None:
        self.drive.drive(wpimath.applyDeadband(self.x_speed, SwerveOIConstants.kDriveDeadband),
                         wpimath.applyDeadband(self.y_speed, SwerveOIConstants.kDriveDeadband),
                         wpimath.applyDeadband(self.rot, SwerveOIConstants.kDriveDeadband),
                         self.field_relative,
                         self.rate_limit)
