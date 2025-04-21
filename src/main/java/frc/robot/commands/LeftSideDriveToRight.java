package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class LeftSideDriveToRight extends Command {
    private final SwerveSubsystem m_swerve;
    private Timer m_timer = new Timer();
    private double startX;
    private double startY;
    private double distanceMeters;
    private boolean hasTurned = false; // Flag to check if robot has finished turning

    // Constructor takes the swerve subsystem and distance to drive
    public LeftSideDriveToRight(SwerveSubsystem swerve, double distanceMeters) {
        this.m_swerve = swerve;
        this.distanceMeters = distanceMeters;

        // Declare subsystem dependencies
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Reset the robot's odometry, start with a 90 degree rotation (facing forward)
        final double resetToAngle = 90;
        m_swerve.resetOdometry(new Pose2d(m_swerve.getPose().getTranslation(), Rotation2d.fromDegrees(resetToAngle)));

        // Start the timer to track elapsed time
        m_timer.restart();
        
        // Record starting positions for distance tracking
        startX = m_swerve.getPose().getX();
        startY = m_swerve.getPose().getY();

        // Reset the turning flag
        hasTurned = false;
    }

    @Override
    public void execute() {
        if (!hasTurned) {
            // Move forward at a speed of 1 meter per second
            m_swerve.drive(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    1.0, // Forward speed (meters per second)
                    0.0, // No sideways movement
                    0.0, // No rotational movement
                    m_swerve.getPose().getRotation() // Maintain current rotation
                )
            );
        } else {
            // Once the robot has moved the desired distance, turn 45 degrees to the right
            m_swerve.drive(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    0.0, // No forward movement
                    0.0, // No sideways movement
                    Math.toRadians(45), // Turn 45 degrees to the right
                    m_swerve.getPose().getRotation() // Maintain current rotation
                )
            );
        }
    }

    @Override
    public boolean isFinished() {
        // End the command after a timeout or once the target distance is reached
        if (m_timer.get() > 15) {
            System.out.println("Auto timed out");
            return true;
        }

        // If the robot has driven the specified distance, set the flag to turn
        if (Math.abs(m_swerve.getPose().getX() - startX) + Math.abs(m_swerve.getPose().getY() - startY) >= distanceMeters) {
            System.out.println("Robot hit distance limit, starting turn");
            hasTurned = true; // Set the flag to true to start turning
        }

        // Check if the robot has turned 45 degrees
        if (hasTurned && Math.abs(m_swerve.getPose().getRotation().getDegrees() - 135) < 5) {
            System.out.println("Robot finished turn to 45 degrees");
            return true; // Command is finished when the robot turns 45 degrees
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot's movement when the command ends
        m_swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}
