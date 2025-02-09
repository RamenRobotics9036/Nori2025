package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants.AlignV2RobotConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AimAtLimeLightV2 extends Command {
    private Timer m_timer = new Timer();
    private PIDController m_pidController = new PIDController(AlignV2RobotConstants.kP, AlignV2RobotConstants.kP, AlignV2RobotConstants.kD);
    private SwerveSubsystem m_swerveDrive;

    public AimAtLimeLightV2(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_pidController.setSetpoint(0);
        m_pidController.setTolerance(AlignV2RobotConstants.allowedAngleUncertaintyDegrees);
    }
    
    @Override
    public void execute() {
        Translation2d translation = new Translation2d(0, 0);
        double rotation = 0; // $TODO m_pidController.calculate(m_swerveDrive.getVisionSystem().getTX());
        rotation = MathUtil.clamp(rotation, -AlignV2RobotConstants.maxSpeed, AlignV2RobotConstants.maxSpeed);
        m_swerveDrive.drive(translation, rotation, true);
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > AlignV2RobotConstants.maxTimeSeconds) {
            return true;
        }
        if (true) { // $TODO !m_swerveDrive.getVisionSystem().isDetecting()) {
            return true;
        }
        return m_pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
