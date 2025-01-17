package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSystem;
import frc.robot.Constants.VisionConstants;

public class AlignRobot extends Command {
    private Timer m_timer = new Timer();
    private SwerveSubsystem m_swerveDrive;
    private PIDController m_rotPIDController = new PIDController(0.0, 0, 0);
    private PIDController m_xPIDController = new PIDController(0.01, 0, 0);
    private PIDController m_yPIDController = new PIDController(0.01, 0, 0);


    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        m_timer.restart();
        Pose2d targetPose = new Pose2d();
        if (VisionSystem.isDetecting()) {
            targetPose = VisionSystem.getTargetPose();
            Transform2d transform = new Transform2d(
                AlignRobotConstants.transformX,
                AlignRobotConstants.transformY,
                Rotation2d.fromDegrees(AlignRobotConstants.transformRot)
            );
            targetPose = targetPose.plus(transform);
            m_swerveDrive.driveToPose(targetPose).schedule();
        }
        m_rotPIDController.setTolerance(VisionConstants.allowedAngleUncertainty);
        m_xPIDController.setTolerance(VisionConstants.allowedAngleUncertainty);
        m_yPIDController.setTolerance(VisionConstants.allowedAngleUncertainty);

        m_rotPIDController.setSetpoint(targetPose.getRotation().getDegrees());
        m_xPIDController.setSetpoint(targetPose.getX());
        m_yPIDController.setSetpoint(targetPose.getY());
    }

    @Override
    public void execute() {
        // Translation2d translation = new Translation2d(0, 0);
        // m_swerveDrive.drive(translation, rotation, true);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            return true;
        }
        if (!VisionSystem.isDetecting()) {
            return true;
        }
        return m_rotPIDController.atSetpoint() && m_xPIDController.atSetpoint() && m_yPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
