package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.CommandConstants.AlignRobotConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.VisionSystem;

public class AlignRobot extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Timer m_timer = new Timer();
    private Pose2d m_targetPose;

    private PIDController m_drivePIDController = new PIDController(0.01, 0, 0);
    private PIDController m_strafePIDController = new PIDController(0.01, 0, 0);
    private PIDController m_rotationPIDController = new PIDController(0.1, 0, 0.05);


    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        m_drivePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyMeters);
        m_strafePIDController.setTolerance(VisionConstants.allowedAngleUncertaintyMeters);
        m_rotationPIDController.setTolerance(VisionConstants.allowedAngleUncertaintyDegrees);

        m_drivePIDController.setSetpoint(AlignRobotConstants.transformDrive);
        m_strafePIDController.setSetpoint(AlignRobotConstants.transformStrafe);
        m_rotationPIDController.setSetpoint(AlignRobotConstants.transformRot);
    }

    @Override
    public void initialize() {
        if (!VisionSystem.isDetecting()) {
            cancel();
        }
        m_timer.restart();
        m_targetPose = VisionSystem.getTargetPose();

        m_drivePIDController.reset();
        m_strafePIDController.reset();
        m_rotationPIDController.reset();
    }

    @Override
    public void execute() {
        Pose2d robotPose = m_swerveDrive.getPose();

        double drive = m_drivePIDController.calculate(robotPose.getX() - m_targetPose.getX());
        double strafe = m_strafePIDController.calculate(robotPose.getY() - m_targetPose.getY());
        double rotation = m_rotationPIDController.calculate(VisionSystem.getTX());

        drive = MathUtil.clamp(drive, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        strafe = MathUtil.clamp(strafe, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        rotation = MathUtil.clamp(rotation, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);

        m_swerveDrive.drive(new ChassisSpeeds(drive, strafe, 0));
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            return true;
        }
        // if (!VisionSystem.isDetecting()) {
        //     return true;
        // }
        return m_drivePIDController.atSetpoint(); // && (m_drivePIDController.atSetpoint() && m_strafePIDController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
