package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionSystem;
import swervelib.SwerveController;

public class AlignRobot extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Timer m_timer = new Timer();
    private Pose2d m_targetPose;

    private PIDController m_drivePIDController = new PIDController(0.01, 0, 0);
    private PIDController m_strafePIDController = new PIDController(0.01, 0, 0);


    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        m_drivePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyMeters);
        m_strafePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyMeters);

        m_drivePIDController.setSetpoint(AlignRobotConstants.transformDrive);
        m_strafePIDController.setSetpoint(AlignRobotConstants.transformStrafe);

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        if (!VisionSystem.isDetecting()) {
            cancel();
        }
        m_timer.restart();
        m_targetPose = VisionSystem.getTargetPose();
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_swerveDrive.getPose();

        double drive = m_drivePIDController.calculate(robotPose.getX() - m_targetPose.getX());
        double strafe = m_strafePIDController.calculate(robotPose.getY() - m_targetPose.getY());

        drive = MathUtil.clamp(drive, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        strafe = MathUtil.clamp(strafe, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);

        m_swerveDrive.drive(new Translation2d(drive, 0), 0, false);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            return true;
        }
        // if (!VisionSystem.isDetecting()) {
        //     return true;
        // }
        return false;
        // return m_drivePIDController.atSetpoint() && m_strafePIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
