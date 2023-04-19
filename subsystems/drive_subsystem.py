# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
import ntcore
import commands2


from navx import AHRS
from wpilib import SerialPort
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState

from constants.swerve_constants import *
from utils.swerve_utils import *
from utils.max_swerve_module import MAXSwerveModule


class DriveSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()

        self.front_left = MAXSwerveModule(SwerveDriveConstants.kFrontLeftDrivingCanId,
                                          SwerveDriveConstants.kFrontLeftTurningCanId,
                                          SwerveDriveConstants.kFrontLeftChassisAngularOffset)

        self.front_right = MAXSwerveModule(SwerveDriveConstants.kFrontRightDrivingCanId,
                                           SwerveDriveConstants.kFrontRightTurningCanId,
                                           SwerveDriveConstants.kFrontRightChassisAngularOffset)

        self.rear_left = MAXSwerveModule(SwerveDriveConstants.kRearLeftDrivingCanId,
                                         SwerveDriveConstants.kRearLeftTurningCanId,
                                         SwerveDriveConstants.kRearLeftChassisAngularOffset)

        self.rear_right = MAXSwerveModule(SwerveDriveConstants.kRearRightDrivingCanId,
                                          SwerveDriveConstants.kRearRightTurningCanId,
                                          SwerveDriveConstants.kRearRightChassisAngularOffset)

        """The gyro sensor"""
        self.gyro = AHRS(SerialPort.Port.kUSB)

        """Slew rate filter variables for controlling lateral acceleration"""
        self.current_rotation = 0.0
        self.current_translation_dir = 0.0
        self.current_translation_mag = 0.0

        self.mag_limiter = SlewRateLimiter(SwerveDriveConstants.kMagnitudeSlewRate)
        self.rot_limiter = SlewRateLimiter(SwerveDriveConstants.kRotationalSlewRate)
        self.prev_time = ntcore._now() * math.e-6  # This is a guess for how time works here

        """Odometry class for tracking robot pose"""
        self.odometry = kinematics.SwerveDrive4Odometry(SwerveDriveConstants.kDriveKinematics,
                                                        geometry.Rotation2d.fromDegrees(self.gyro.getAngle()),
                                                        (self.front_left.get_position(),
                                                         self.front_right.get_position(),
                                                         self.rear_left.get_position(),
                                                         self.rear_right.get_position()
                                                         ))

    def periodic(self) -> None:
        """Update the odometry in the periodic block"""
        self.odometry.update(geometry.Rotation2d.fromDegrees(self.gyro.getAngle()),
                             self.front_left.get_position(),
                             self.front_right.get_position(),
                             self.rear_left.get_position(),
                             self.rear_right.get_position())

    def get_pose(self) -> geometry.Pose2d:
        """Returns the currently-estimated pose of the robot."""
        return self.odometry.getPose()

    def reset_odometry(self, pose: geometry.Pose2d) -> None:
        """
        Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.
        """
        self.odometry.resetPosition(geometry.Rotation2d.fromDegrees(self.gyro.getAngle()),
                                    pose,
                                    self.front_left.get_position(),
                                    self.front_right.get_position(),
                                    self.rear_left.get_position(),
                                    self.rear_right.get_position())

    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool, rate_limit: bool) -> None:
        """
        Method to drive the robot using joystick info.

        :param x_speed: Speed of the robot in the x direction (forward).
        :param y_speed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param field_relative: Whether the provided x and y speeds are relative to the field.
        :param rate_limit: Whether to enable rate limiting for smoother control.
        """
        x_speed_commanded: float
        y_speed_commanded: float

        if rate_limit:
            """Convert XY to polar for rate limiting"""
            input_translation_dir = math.atan2(y_speed, x_speed)
            input_translation_mag = math.sqrt(math.pow(x_speed, 2) + math.pow(y_speed, 2))

            """Calculate the direction slew rate based on an estimate of the lateral acceleration"""
            direction_slew_rate: float

            if self.current_translation_mag is not 0.0:
                direction_slew_rate = abs(SwerveDriveConstants.kDirectionSlewRate / self.current_translation_mag)
            else:
                direction_slew_rate = 500.0  # some high number that means the slew rate is effectively instantaneous

            current_time = ntcore._now() * math.e-6
            elapsed_time = current_time - self.prev_time
            angle_dif = SwerveUtils.angle_difference(self, input_translation_dir, self.current_translation_dir)

            if angle_dif < 0.45*math.pi:
                self.current_translation_dir = SwerveUtils.step_towards_circular(self,
                                                                                 self.current_translation_dir,
                                                                                 input_translation_dir,
                                                                                 direction_slew_rate*elapsed_time)
                self.current_translation_mag = self.mag_limiter.calculate(input_translation_mag)
            elif angle_dif > 0.85*math.pi:
                if self.current_translation_mag > math.e-4:  # smol num avoids floating-point errors w/ equality checkin
                    """keep currentTranslationDir unchanged"""
                    self.current_translation_mag = self.mag_limiter.calculate(0.0)
                else:
                    self.current_translation_dir = SwerveUtils.wrap_angle(self, self.current_translation_dir + math.pi)
                    self.current_translation_mag = self.mag_limiter.calculate(input_translation_mag)
            else:
                self.current_translation_dir = SwerveUtils.step_towards_circular(self,
                                                                                 self.current_translation_dir,
                                                                                 input_translation_dir,
                                                                                 direction_slew_rate*elapsed_time)
                self.current_translation_mag = self.mag_limiter.calculate(0.0)

            self.prev_time = current_time

            x_speed_commanded = self.current_translation_mag * math.cos(self.current_translation_dir)
            y_speed_commanded = self.current_translation_mag * math.sin(self.current_translation_dir)
            self.current_rotation = self.rot_limiter.calculate(rot)

        else:
            x_speed_commanded = x_speed
            y_speed_commanded = y_speed
            self.current_rotation = rot

        x_speed_delivered = x_speed_commanded * SwerveDriveConstants.kMaxSpeedMetersPerSecond
        y_speed_delivered = y_speed_commanded * SwerveDriveConstants.kMaxSpeedMetersPerSecond
        rot_delivered = self.current_rotation * SwerveDriveConstants.kMaxAngularSpeed

        swerve_module_states = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered,
                                                  Rotation2d.fromDegrees(self.gyro.getAngle()))
            if field_relative else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered))

        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states,
                                                                SwerveDriveConstants.kMaxSpeedMetersPerSecond)

        self.front_left.set_desired_state(swerve_module_states[0])
        self.front_right.set_desired_state(swerve_module_states[1])
        self.rear_left.set_desired_state(swerve_module_states[2])
        self.rear_right.set_desired_state(swerve_module_states[3])

    def set_x(self) -> None:
        """
        Sets the wheels into an X formation to prevent movement.
        """
        self.front_left.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.front_right.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rear_left.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rear_right.set_desired_state(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def set_module_states(self, desired_states: tuple[SwerveModuleState, SwerveModuleState,
                                                      SwerveModuleState, SwerveModuleState]) -> None:
        """
        Sets the swerve ModuleStates.

        :param desired_states: The desired SwerveModule states.
        """
        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states,
                                                                SwerveDriveConstants.kMaxSpeedMetersPerSecond)
        self.front_left.set_desired_state(desired_states[0])
        self.front_right.set_desired_state(desired_states[1])
        self.rear_left.set_desired_state(desired_states[2])
        self.rear_right.set_desired_state(desired_states[3])

    def reset_encoders(self) -> None:
        """
        Resets the drive encoders to currently read a position of 0.
        """
        self.front_left.reset_encoders()
        self.front_right.reset_encoders()
        self.rear_left.reset_encoders()
        self.rear_right.reset_encoders()

    def zero_heading(self) -> None:
        """
        Zeroes the heading of the robot.
        """
        self.gyro.reset()

    def get_heading(self) -> float:
        """
        Returns the heading of the robot.

        Returns the robot's heading in degrees, from -180 to 180
        """
        return Rotation2d.fromDegrees(self.gyro.getAngle()).degrees()

    def get_turn_rate(self) -> float:
        """
        Returns the turn rate of the robot.

        The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if SwerveDriveConstants.kGyroReversed else 1.0)
