package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class DriveForwardCommand extends Command {
    private SwerveSubsystem m_swerveDrive;
    private double startX;
    private double startY;
    private Timer m_timer = new Timer();
    private PIDController m_xPID = new PIDController(1, 0, 0);
    private PIDController m_yPID = new PIDController(2, 0, 0);
    
    public DriveForwardCommand(SwerveSubsystem swerve, double dx, double dy) {
        m_swerveDrive = swerve;

        m_xPID.setSetpoint(dx);
        m_yPID.setSetpoint(dy);

        m_xPID.setTolerance(0.01);
        m_yPID.setTolerance(0.01);

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        Pose2d pose = m_swerveDrive.getPose();
        startX = pose.getX();
        startY = pose.getY();
    }

    @Override
    public void execute() {
        Pose2d pose = m_swerveDrive.getPose();
        double driveX = m_xPID.calculate(startX - pose.getX());
        double driveY = m_yPID.calculate(startY - pose.getY());

        m_swerveDrive.drive(
                ChassisSpeeds.fromRobotRelativeSpeeds(
                    driveX, // Forward speed in meters per second
                    driveY, // Sideways speed in meters per second
                    0.0, // Rotational speed in radians per second
                    m_swerveDrive.getPose().getRotation()) // Robot's current rotation
        );
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > 5) {
            return true;
        }

        return m_xPID.atSetpoint() && m_yPID.atSetpoint();
    }
}
