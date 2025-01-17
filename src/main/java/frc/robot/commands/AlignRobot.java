package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

    private PIDController m_drivePIDController = new PIDController(0.01, 0, 0);
    private PIDController m_strafePIDController = new PIDController(0.01, 0, 0);
    private PIDController m_rotationPIDController = new PIDController(0.01, 0, 0);


    public AlignRobot(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;
    }

    @Override
    public void initialize() {
        double drive = m_drivePIDController.calculate(VisionSystem.getTX());
        double strafe = m_strafePIDController.calculate(VisionSystem.getTX());
        double rotation = m_rotationPIDController.calculate(VisionSystem.getTX());

        drive = MathUtil.clamp(drive, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        strafe = MathUtil.clamp(strafe, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);
        rotation = MathUtil.clamp(rotation, -AlignRobotConstants.maxSpeed, AlignRobotConstants.maxSpeed);

        Translation2d translation = new Translation2d(drive, strafe);
        m_swerveDrive.drive(translation, rotation, true);
    }

    @Override
    public void execute() {
        m_swerveDrive.drive(null, 0, isScheduled());
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignRobotConstants.maxTimeSeconds) {
            return true;
        }
        if (!VisionSystem.isDetecting()) {
            return true;
        }
        return (m_drivePIDController.atSetpoint() && m_strafePIDController.atSetpoint()) && m_rotationPIDController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
