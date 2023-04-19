import math
from wpimath import kinematics, geometry, trajectory
import rev

INCH_TO_METER_CONSTANT = 0.0254


class NeoMotorConstants:
    """Constants from specifications of the REV NEO motor"""
    kFreeSpeedRpm = 5676


class SwerveDriveConstants:
    """Constants used for swerve drive setup and kinematics. Translated from Java."""
    """
    Driving Parameters - Note that these are not the maximum
    capable speeds of the robot, rather the allowed maximum speeds
    """
    kMaxSpeedMetersPerSecond = 4.8
    kMaxAngularSpeed = 2 * math.pi  # Radians per second

    kDirectionSlewRate = 1.2  # Radians per second
    kMagnitudeSlewRate = 1.8  # Percent per second (1 = 100%)
    kRotationalSlewRate = 2.0  # Percent per second (1 = 100%)

    """Chassis configuration"""
    kTrackWidth = INCH_TO_METER_CONSTANT * 25.6  # Distance between centers of right and left wheels on robot
    kWheelBase = INCH_TO_METER_CONSTANT * 25.6  # Distance between front and back wheels on robot

    kDriveKinematics: kinematics.SwerveDrive4Kinematics(
        geometry.Translation2d(kWheelBase / 2, kTrackWidth / 2),
        geometry.Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        geometry.Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        geometry.Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
    )

    """Angular offsets of the modules relative to the chassis in radians"""
    kFrontLeftChassisAngularOffset = -math.pi / 2
    kFrontRightChassisAngularOffset = 0
    kRearLeftChassisAngularOffset = math.pi
    kRearRightChassisAngularOffset = math.pi / 2

    """SPARK MAX CAN IDs"""
    kFrontLeftDrivingCanId = 11
    kRearLeftDrivingCanId = 13
    kFrontRightDrivingCanId = 15
    kRearRightDrivingCanId = 17

    kFrontLeftTurningCanId = 10
    kRearLeftTurningCanId = 12
    kFrontRightTurningCanId = 14
    kRearRightTurningCanId = 16

    kGyroReversed = False


class SwerveModuleConstants:
    """Constants used as part of the MAXSwerveModule class and related functions. Translated from Java."""
    """
    The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    This changes the drive speed of the module.
    (a pinion gear with more teeth will result in a robot that drives faster).
    """
    kDrivingMotorPinionTeeth = 14

    """
    Invert the turning encoder, since the output shaft rotates in the opposite direction of
    the steering motor in the MAXSwerve Module.
    """
    kTurningEncoderInverted = True

    """
    Calculations required for driving motor conversion factors and feed forward
    """
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 0.0762
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi

    """45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion"""
    kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) / 60.0  # meters/sec

    kTurningEncoderPositionFactor = (2 * math.pi)  # radians
    kTurningEncoderVelocityFactor = (2 * math.pi) / 60.0  # radians per second

    kTurningEncoderPositionPIDMinInput = 0  # radians
    kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor  # radians

    kDrivingP = 0.04
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps
    kDrivingMinOutput = -1
    kDrivingMaxOutput = 1

    kTurningP = 1
    kTurningI = 0
    kTurningD = 0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    kDrivingMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake
    kTurningMotorIdleMode = rev.CANSparkMax.IdleMode.kBrake

    kDrivingMotorCurrentLimit = 50  # amps
    kTurningMotorCurrentLimit = 20  # amps


class SwerveOIConstants:
    kDriverControllerPort = 0
    kDriveDeadband = 0.05


class SwerveAutoConstants:
    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3
    kMaxAngularSpeedRadiansPerSecond = math.pi
    kMaxAngularSpeedRadiansPerSecondSquared = math.pi

    kPXController = 1
    kPYController = 1
    kPThetaController = 1

    kThetaControllerConstraints: trajectory.TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularSpeedRadiansPerSecondSquared
    )
