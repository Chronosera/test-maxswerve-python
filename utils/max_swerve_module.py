import rev
from wpimath import kinematics, geometry

from constants.swerve_constants import SwerveModuleConstants


class MAXSwerveModule:
    """
    Constructs a MAXSwerveModule and configures the driving and turning motor,
    encoder, and PID controller. This configuration is specific to the REV
    MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore Encoder.
    """
    def __init__(self,
                 driving_can_id: int,
                 turning_can_id: int,
                 chassis_angular_offset: float):
        self.desired_state = kinematics.SwerveModuleState(0.0, geometry.Rotation2d)

        self.driving_spark_max = rev.CANSparkMax(driving_can_id, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.turning_spark_max = rev.CANSparkMax(turning_can_id, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        # Factory reset, so we get the SPARKS MAX to a known state before configuring them.
        # This is useful in case a SPARK MAX is swapped out.
        self.driving_spark_max.restoreFactoryDefaults()
        self.turning_spark_max.restoreFactoryDefaults()

        # Setup encoders and PID controllers for the driving and turning SPARKS MAX.
        self.driving_encoder = self.driving_spark_max.getEncoder()
        self.turning_encoder = self.turning_spark_max.getAbsoluteEncoder(rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle)
        self.driving_pid_controller = self.driving_spark_max.getPIDController()
        self.turning_pid_controller = self.turning_spark_max.getPIDController()
        self.driving_pid_controller.setFeedbackDevice(self.driving_encoder)
        self.turning_pid_controller.setFeedbackDevice(self.turning_encoder)

        """
        Apply position and velocity conversion factors for the driving encoder.
        The native units for position and velocity are rotations and RPM, respectively,
        but we want meters and meters per second to use with WPILib's swerve APIs.
        """
        self.driving_encoder.setPositionConversionFactor(SwerveModuleConstants.kDrivingEncoderPositionFactor)
        self.driving_encoder.setVelocityConversionFactor(SwerveModuleConstants.kDrivingEncoderVelocityFactor)

        """
        Apply position and velocity conversion factors for the turning encoder. 
        We want these in radians and radians per second to use with WPILib's swerve APIs.
        """
        self.turning_encoder.setPositionConversionFactor(SwerveModuleConstants.kTurningEncoderPositionFactor)
        self.turning_encoder.setVelocityConversionFactor(SwerveModuleConstants.kTurningEncoderVelocityFactor)

        """
        Invert the turning encoder, since the output shaft rotates in the opposite direction of
        the steering motor in the MAXSwerve Module.
        """
        self.turning_encoder.setInverted(SwerveModuleConstants.kTurningEncoderInverted)

        """
        Enable PID wrap around for the turning motor. This will allow the PID
        controller to go through 0 to get to the set point i.e. going from 350 degrees
        to 10 degrees will go through 0 rather than the other direction which is a longer route.
        """
        self.turning_pid_controller.setPositionPIDWrappingEnabled(True)
        self.turning_pid_controller.setPositionPIDWrappingMinInput(SwerveModuleConstants.
                                                                   kTurningEncoderPositionPIDMinInput)
        self.turning_pid_controller.setPositionPIDWrappingMaxInput(SwerveModuleConstants.
                                                                   kTurningEncoderPositionPIDMaxInput)

        """
        Set the PID gains for the driving motor. Note these are example gains, and you
        may need to tune them for your own robot!
        """
        self.driving_pid_controller.setP(SwerveModuleConstants.kDrivingP)
        self.driving_pid_controller.setI(SwerveModuleConstants.kDrivingI)
        self.driving_pid_controller.setD(SwerveModuleConstants.kDrivingD)
        self.driving_pid_controller.setFF(SwerveModuleConstants.kDrivingFF)
        self.driving_pid_controller.setOutputRange(SwerveModuleConstants.kDrivingMinOutput,
                                                   SwerveModuleConstants.kDrivingMaxOutput)

        """
        Set the PID gains for the turning motor. Note these are example gains, and you
        may need to tune them for your own robot!
        """
        self.turning_pid_controller.setP(SwerveModuleConstants.kTurningP)
        self.turning_pid_controller.setI(SwerveModuleConstants.kTurningI)
        self.turning_pid_controller.setD(SwerveModuleConstants.kTurningD)
        self.turning_pid_controller.setFF(SwerveModuleConstants.kTurningFF)
        self.turning_pid_controller.setOutputRange(SwerveModuleConstants.kTurningMinOutput,
                                                   SwerveModuleConstants.kTurningMaxOutput)

        self.driving_spark_max.setIdleMode(SwerveModuleConstants.kDrivingMotorIdleMode)
        self.turning_spark_max.setIdleMode(SwerveModuleConstants.kTurningMotorIdleMode)
        self.driving_spark_max.setSmartCurrentLimit(SwerveModuleConstants.kDrivingMotorCurrentLimit)
        self.turning_spark_max.setSmartCurrentLimit(SwerveModuleConstants.kTurningMotorCurrentLimit)

        """
        Save the SPARK MAX configurations. If a SPARK MAX browns out during
        operation, it will maintain the above configurations.
        """
        self.driving_spark_max.burnFlash()
        self.turning_spark_max.burnFlash()

        self.chassis_angular_offset = chassis_angular_offset
        self.desired_state.angle = geometry.Rotation2d(self.turning_encoder.getPosition())
        self.driving_encoder.setPosition(0)

    def get_state(self) -> kinematics.SwerveModuleState:
        """
        Returns the current state of the module.
        """
        """Apply chassis angular offset to the encoder position to get the position relative to the chassis."""
        return kinematics.SwerveModuleState(self.driving_encoder.getVelocity(),
                                            geometry.Rotation2d(self.turning_encoder.getPosition()
                                                                - self.chassis_angular_offset))

    def get_position(self) -> kinematics.SwerveModulePosition:
        """
        Returns the current position of the module.
        """
        """Apply chassis angular offset to the encoder position to get the position relative to the chassis."""
        return kinematics.SwerveModulePosition(self.driving_encoder.getPosition(),
                                               geometry.Rotation2d(self.turning_encoder.getPosition()
                                                                   - self.chassis_angular_offset))

    def set_desired_state(self, desired_state: kinematics.SwerveModuleState):
        """
        Sets the desired state for the module.

        :param desired_state: Desired state with speed and angle.
        """
        """Apply chassis angular offset to the desired state."""  # Maybe doesn't work? IDK the language was weird
        corrected_desired_state = kinematics.SwerveModuleState()
        corrected_desired_state.speed = desired_state.speed
        corrected_desired_state.angle = desired_state.angle.__add__(geometry.Rotation2d
                                                                    (self.chassis_angular_offset))

        """Optimize the reference state to avoid spinning further than 90 degrees."""
        optimized_desired_state = kinematics.SwerveModuleState.optimize(corrected_desired_state,
                                                                        geometry.Rotation2d(self.turning_encoder
                                                                                            .getPosition()))

        """Command driving and turning SPARKS MAX towards their respective set points."""
        self.driving_pid_controller.setReference(optimized_desired_state.speed,
                                                 rev.CANSparkMax.ControlType.kVelocity)
        self.turning_pid_controller.setReference(optimized_desired_state.angle.radians(),
                                                 rev.CANSparkMax.ControlType.kPosition)

        self.desired_state = desired_state

    def reset_encoders(self):
        self.driving_encoder.setPosition(0)
