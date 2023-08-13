// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.ChassisConstants;
import frc.robot.subsystems.*;
import java.lang.Math;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControlStuff extends CommandBase {
  /** Creates a new ControlStuff. */
  Joystick left;
  Joystick right;
  Chassis chassis;
  public ControlStuff(Joystick left, Joystick right,Chassis chassis) {
    this.left = left;
    this.right = right;
    this.chassis = chassis;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    chassis.setPower(Math.pow(this.left.getY(),2), Math.pow(this.right.getY(),2));
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
