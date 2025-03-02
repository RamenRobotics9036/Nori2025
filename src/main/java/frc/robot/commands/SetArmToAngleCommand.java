package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.IntakeArmSystem;

public class SetArmToAngleCommand extends Command {
    private IntakeArmSystem m_armSystem;
    private Timer m_timer = new Timer();
    private double m_desiredAngle = ArmConstants.kMinArmRotation;

    public SetArmToAngleCommand(IntakeArmSystem armSystem, double desiredAngle) {
        m_armSystem = armSystem;
        m_desiredAngle = desiredAngle;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_armSystem.setReference(m_desiredAngle);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > ArmConstants.setArmMaxTime) {
            System.out.println("WARNING: SetArmToAngleCommand timed out!");
            return true;
        }
        if (MathUtil.applyDeadband(m_desiredAngle - m_armSystem.getArmAngleRelative(), ArmConstants.tolerance) == 0) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
    }
}
