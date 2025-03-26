// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DriveForwardNow extends Command
{
  private final SwerveSubsystem m_swerve;
  private Timer m_timer = new Timer();
  private double startX;
  private double startY;
  private double distanceMeters;

  public DriveForwardNow(SwerveSubsystem swerve, double distanceMeters)
  {
    this.m_swerve = swerve;
    this.distanceMeters = distanceMeters;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    // final double resetToAngle = (OperatorConstants.kAlliance.get() == Alliance.Red) ? 90 : 270;
    final double resetToAngle = 90;
    m_swerve.resetOdometry(new Pose2d(m_swerve.getPose().getTranslation(), Rotation2d.fromDegrees(resetToAngle)));
    m_timer.restart();
    startX = m_swerve.getPose().getX();
    startY = m_swerve.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    this.m_swerve.drive(
        ChassisSpeeds.fromRobotRelativeSpeeds(
            1.0, // Forward speed in meters per second
            0.0, // Sideways speed in meters per second
            0.0, // Rotational speed in radians per second
            m_swerve.getPose().getRotation() // Robot's current rotation
        )
    );

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if (m_timer.get() > 15) {
      System.out.println("auto timed out");
        return true;
    }
    if (Math.abs(m_swerve.getPose().getX() - startX) + Math.abs(m_swerve.getPose().getY() - startY) >= distanceMeters) {
      System.out.println("auto hit distance limit");
      return true;
    }

    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    m_swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
