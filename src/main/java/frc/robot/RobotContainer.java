// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(
      swerveSubsystem.driveCommand(()->-driverController.getLeftY(), ()->-driverController.getLeftX(), ()->-driverController.getRightX())
    );

    // new Trigger(driverController.button(8))
    // .onTrue()
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
