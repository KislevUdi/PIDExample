// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.VelocityPIDCommand;
import frc.robot.subsystems.Chassis;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Chassis chassis;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    chassis = new Chassis(this);
    configureBindings();
  }

  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return new VelocityPIDCommand(chassis).withTimeout(3);
  }
}
