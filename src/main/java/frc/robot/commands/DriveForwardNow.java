// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants.AimAtLimeLightV2Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

/**
 * An example command that uses an example subsystem.
 */
public class DriveForwardNow extends Command
{
  private final SwerveSubsystem m_swerve;
  private Timer m_timer = new Timer();

  public DriveForwardNow(SwerveSubsystem swerve)
  {
    this.m_swerve = swerve;

    addRequirements(swerve);
  }

  @Override
  public void initialize()
  {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    this.m_swerve.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            1.0, // Forward speed in meters per second
            0.0, // Sideways speed in meters per second
            0.0, // Rotational speed in radians per second
            new Rotation2d(0) // Robot's current rotation
        )
    );

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    if (m_timer.get() > 1.0) {
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
