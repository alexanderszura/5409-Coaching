// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // Subsystems

  // Controllers
  private final CommandXboxController primaryController   = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {

      }
      case SIM -> {

      }
      default -> {

      }
    }

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
