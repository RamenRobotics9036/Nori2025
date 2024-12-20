// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  /**
   * Constants for the joystick.
   */
  public static class OperatorConstants
  {
    public static final int kDriverPort = 0;
    public static final int kArmPort = 1;
    public static final double kExpo = 4; //do not change this value
    public static final double kExpoRatio = 0.8; // change this 0..1 to add more exponential, 0 = no expo (linear)
    public static final double kDeadband = 0.07;
  }
  /**
   * Constants for the swerve system.
   */
  public static class SwerveConstants
  {
    public static final String  kJsonDirectory = "swerve";
    public static final double kMaxSpeedMetersPerSecond = 5.06;
    public static final double kRobotMass = Units.lbsToKilograms(120); // TODO: update
    public static final Matter kChassisMatter = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass); // TODO: update
    public static final double kLoopTime = 0.13; //s, 20ms + 110ms sprk max velocity lag
  }
  /**
   * Constants for telemetry.
   */
  public static class TelemetryConstants
  {
    public static final TelemetryVerbosity kSwerveVerbosity = TelemetryVerbosity.HIGH;
  }
  /**
   * Constants for autonomous mode.
   */
  public static final class AutoConstants
  {
    public static final PIDConstants kTranslationpid = new PIDConstants(0.7, 0, 0); // try 0.7,0,0 from YAGSL-Example
    public static final PIDConstants kAnglePID = new PIDConstants(0.4, 0, 0.01); // try 0.4,0,0.01 from YAGSL-Example
    public static final double kMaxSpeedMetersPerSecond = 5.06; // try 4.5 from YAGSL-Example
  }
}
