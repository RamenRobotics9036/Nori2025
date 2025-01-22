package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
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
    private PIDController m_strafePIDController = new PIDController(4, 0, 0);
    private PIDController m_rotatePIDController = new PIDController(0.2, 0, 0);

    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        m_drivePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyMetersDrive);
        m_strafePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyMetersStrafe);
        m_rotatePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyDegrees);

        m_drivePIDController.setSetpoint(AlignRobotConstants.transformDrive);
        m_strafePIDController.setSetpoint(AlignRobotConstants.transformStrafe);
        m_rotatePIDController.setSetpoint(AlignRobotConstants.transformRot);

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
        Pose3d targetPose = VisionSystem.getTargetPose();

        double drive = m_drivePIDController.calculate(targetPose.getZ());
        double strafe = m_strafePIDController.calculate(targetPose.getX());
        double rotate = m_rotatePIDController.calculate(VisionSystem.getTX());

        drive = MathUtil.clamp(drive, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        strafe = MathUtil.clamp(strafe, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        rotate = MathUtil.clamp(rotate, -AlignRobotConstants.maxSpeedRot, AlignRobotConstants.maxSpeedRot);

        // Drive is negative
        m_swerveDrive.drive(new Translation2d(-drive, strafe), -rotate, false);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            return true;
        }
        if (!VisionSystem.isDetecting()) {
            return true;
        }
        return (m_drivePIDController.atSetpoint() && m_strafePIDController.atSetpoint()) && m_rotatePIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
