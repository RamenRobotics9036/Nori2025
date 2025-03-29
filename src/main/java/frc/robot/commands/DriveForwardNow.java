// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.InitialPose;

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
  private Optional<Pose2d> nonmirroredInitialPose;

  public DriveForwardNow(SwerveSubsystem swerve, double distanceMeters, Optional<Pose2d> nonmirroredInitialPose)
  {
    this.m_swerve = swerve;
    this.distanceMeters = distanceMeters;
    this.nonmirroredInitialPose = nonmirroredInitialPose;

    addRequirements(swerve);
  }

  public DriveForwardNow(SwerveSubsystem swerve, double distanceMeters)
  {
    this(swerve, distanceMeters, Optional.empty());
  }

  @Override
  public void initialize()
  {
    if (nonmirroredInitialPose.isPresent()) {
      Pose2d mirroredPose = InitialPose.getCalculatedInitialPose(nonmirroredInitialPose.get());
      m_swerve.resetOdometry(mirroredPose);
    }

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
      System.out.println("Error: Auto timed out");
        return true;
    }
    if (Math.abs(m_swerve.getPose().getX() - startX) + Math.abs(m_swerve.getPose().getY() - startY) >= distanceMeters) {
      System.out.println("Error: Auto hit distance limit");
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
