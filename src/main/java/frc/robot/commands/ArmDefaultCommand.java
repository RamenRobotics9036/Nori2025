package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmDefaultCommandConstants;
import frc.robot.subsystems.IntakeArmSystem;

public class ArmDefaultCommand extends Command {
    private IntakeArmSystem m_armSystem;
    private Supplier<Double> m_deltaArmAngleFromController;
    private double desiredAngle = ArmConstants.kMinArmRotation;

    public ArmDefaultCommand(IntakeArmSystem armSystem, Supplier<Double> deltaArmAngleFromController) {
        m_armSystem = armSystem;
        m_deltaArmAngleFromController = deltaArmAngleFromController;
        addRequirements(m_armSystem);
    }

    @Override
    public void initialize() {
        desiredAngle = m_armSystem.getArmAngleRelative();
    }

    @Override
    public void execute() {
        setArmAngle(m_armSystem.getArmAngleRelative() + (m_deltaArmAngleFromController.get() / ArmDefaultCommandConstants.kArmAngleChangeRate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_armSystem.stopSystem();
    }

    private void setArmAngle(double value) {
        value = MathUtil.clamp(value, ArmConstants.kMinArmRotation, ArmConstants.kMaxArmRotation);
        desiredAngle = value;
        m_armSystem.setReference(desiredAngle);
    }
}
