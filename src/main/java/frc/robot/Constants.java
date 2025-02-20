// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    public static final Alliance kAlliance = (DriverStation.getAlliance().isPresent()) ? DriverStation.getAlliance().get() : Alliance.Red;
  }
  /**
   * Constants for the swerve system.
   */
  public static class SwerveConstants
  {
    // USe the directory matching the robot
    public static final String  kJsonDirectory = "pancake";
    // public static final String  kJsonDirectory = "nori";
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

  public static final class TestSwerveConstants {
    public static final boolean kIsTestMode = true;
    public static final double maxTimeSeconds = 60;
  }

  public static final class VisionConstants {
    public static final String limelightName = "limelight";
    public static final double allowedAngleUncertaintyDegrees = 0.5;
    public static final double allowedAngleUncertaintyMetersDrive = 0.05;
    public static final double allowedAngleUncertaintyMetersStrafe = 0.025;

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }

  public static class VisionSimConstants {
      public static final String kCameraName = "RAMEN SIM CAMERA";
      // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
      // pitched upward.
      // NOTE: For reefscape, set camPitch to 15 degrees, since 30 degrees was too high to detect
      // the april tags near the coral reefs.
      private static final double camPitch = Units.degreesToRadians(15.0); // Units.degreesToRadians(30.0);
      public static final Transform3d kRobotToCam =
              new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));

      // The standard deviations of our vision estimated poses, which affect correction rate
      // (Fake values. Experiment and determine estimation noise on an actual robot.)
      public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
      public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static final class CommandConstants {
    public static final class AimAtLimeLightConstants {
      public static final double maxTimeSeconds = 10;
      public static final double kP = 0.01;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double maxSpeed = 0.02;
    }

    public static final class AlignRobotConstants {
      public static final double maxTimeSeconds = 10;
      public static final double maxSpeed = 0.4;
      public static final double maxSpeedRot = 0.2;

      /**
       * How much the robot should be offset from the April tag pose rotation.
      */
      public static final double transformRot = 0.0;

      /**
       * How much the robot should be offset from the April tag pose x direction.
      */
      public static final double transformDrive = (OperatorConstants.kAlliance == Alliance.Red) ? 0.5 : 0.5;

      /**
       * How much the robot should be offset from the April tag pose y direction.
      */
      public static final double transformStrafe = 0;
    }

    public static final class AimAtLimeLightV2Constants {
      // 5 seconds should be plenty of time to just turn
      public static final double maxTimeSeconds = 5;
      public static final double maxSpeed = 0.4;
      public static final double kMaxRotateRadsPerSecond = 2 * Math.PI;

      public static final double kP = 0.01;
      public static final double kI = 0.0;
      public static final double kD = 0.0;
      public static final double allowedAngleUncertaintyDegrees = 0.5;
      public static final int dontRotateIfSmallDegrees = 5;

      /**
       * How much the robot should be offset from the April tag pose rotation.
      */
      public static final double transformRot = 0.0;
    }
  }

  public static final class IntakeConstants{
    public static final int kPullMotorID = 21;
    public static final int kLoadMotorID = 20;
    public static final int kStallLimit = 20;
    public static final double kMaxOutputPercentage = 1.0;
    public static final int pullMotorGearBoxFactor= 4;
    public static final int loadMotorGearBoxFactor = 4;
  }

  public static final class IntakeDefaultCommandConstants {
    public static final double speed = 0.3;
    public static final double pullSpeed = 0.5;
  }

  public static final class IntakeSpitCommandConstants {
    public static final int maxTime = 3;
    public static final double speed = 0.8;
    public static final double numRotations = 15;
    public static final double bucketSpeed = 0.5;
  }

  public static final class ArmConstants {
    public static final int kArmMotorID = 22;
    public static final double maxOutput = 1.0;
    public static final int kArmEncoderID = 0;
    public static final double kArmGearBoxRatio = 125 * (44/30);
    public static final double kMaxArmRotation = 6.15;
    public static final double kMinArmRotation = 3.4;
    public static final double kAbsoluteEncoderOffset = 0;
    public static final int kcurrentLimit = 20;
    public static final double tolerance = 0.1;
    public static final double setArmMaxTime = 4;
  }

  public static final class ArmDefaultCommandConstants {
    public static final double armAngleChangeRate = 5;
  }

  public static final class OuttakeConstants {
    public static final int sparkflexID = 32;
    public static final int sparkmaxID = 33;
    public static final int currentLimit = 20;
    public static final double motorGearRatio = 1;
  }

  public static final class OuttakeSpitCommandConstants {
    public static final int maxTime = 3;
    public static final double speed = 1.0;
    public static final double numRotations = 40;
  }
}
