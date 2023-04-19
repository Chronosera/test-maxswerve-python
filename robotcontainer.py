# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import wpilib

import commands2
import commands2.cmd
import commands2.button

import commands.drive_command
import subsystems.drive_subsystem


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    def __init__(self):
        """The container for the robot. Contains subsystems, OI devices, and commands."""
        # The robot's subsystems
        self.robotDrive = subsystems.drive_subsystem.DriveSubsystem()

        # The driver's controller
        self.driverController = wpilib.Joystick(0)

        # Configure the button bindings
        self.configure_button_bindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(
            # A single-stick swerve drive command - field relative, and rate limited
            commands.drive_command.DriveCommand(self.robotDrive,
                                                self.driverController.getX(),
                                                self.driverController.getY(),
                                                self.driverController.getZ(),
                                                True,
                                                True)
        )

    def configure_button_bindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """

    def get_autonomous_command(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return commands2.InstantCommand()
