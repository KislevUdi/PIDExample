// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.Math;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class ChassisConstants {
    public static final int LeftFrontMotor = 1;
    public static final int LeftBackMotor = 1;
    public static final int RightFrontMotor = 1;
    public static final int RightBackMotor = 1;
    public static final boolean LeftInverted = true;
    public static final boolean RightInverted = false;

    public static final double WheelCircumference = 4 * 0.0254 * Math.PI; // 4 Inch wheels
    public static final double GearRatio = 8.14;
    public static final double PulsePerRotation = 2048;
    public static final double PulsePerMeter = (1/WheelCircumference)*GearRatio*PulsePerRotation;

    public static final double VelocityKP = 0.1;
    public static final double VelocityKI = 0.01;
    public static final double VelocityKD = 0;

    public static final String AutoVelocityID = "Auto Velocity";

  }
  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
  }
}
