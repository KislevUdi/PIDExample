// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Chassis;

public class Drive extends CommandBase {
  Chassis chassis;
  CommandXboxController xboxController;
  /** Creates a new Drive. */
  public Drive(Chassis chassis , CommandXboxController xboxController) {
    addRequirements(chassis);
    this.chassis = chassis;
    this. xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = Math.pow(xboxController.getLeftY(), 2) + Math.pow(xboxController.getLeftX(), 2);
    double rightY = Math.pow(xboxController.getLeftY(), 2) + Math.pow(xboxController.getLeftX(), 2) * -1;
    if(0.1 < rightY && 0.1 < leftY){
      chassis.setPower(xboxController.getLeftY(), xboxController.getRightY());

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
