package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionSystem;

public class AlignRobot extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Timer m_timer = new Timer();

    private PIDController m_drivePIDController = new PIDController(2, 0, 0);
    private PIDController m_strafePIDController = new PIDController(2, 0, 0);


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
        m_timer.restart();
    }

    @Override
    public void execute() {
        Pose2d targetPose = VisionSystem.getTargetPose();

        double drive = m_drivePIDController.calculate(targetPose.getY());
        double strafe = m_strafePIDController.calculate(targetPose.getX());

        drive = MathUtil.clamp(drive, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        strafe = MathUtil.clamp(strafe, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);

        m_swerveDrive.drive(new Translation2d(drive, strafe), 0, false);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            return true;
        }
        if (!VisionSystem.isDetecting()) {
            return true;
        }
        return m_drivePIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
