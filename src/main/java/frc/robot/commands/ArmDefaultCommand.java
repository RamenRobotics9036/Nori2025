package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmDefaultCommandConstants;
import frc.robot.subsystems.IntakeArmSystem;
import java.util.function.Supplier;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
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

        // Call setPoint to set the arm-motor PID loop to the desired angle.
        setArmAngle(desiredAngle);
    }

    @Override
    public void execute() {
        if (RobotBase.isSimulation()) {
            // NOTE: Fixed a bug where the desiredAngle was being updated based on the CURRENT position
            // of the arm.  That means every time execute() was called, it was setting a new setpoint
            // in the arm-motor.  The problem is that the arm-motor was already in a closed-loop PID control mode.
            // So we should ONLY set the SetPoint if the joystick is moved.
            // Also, we don't want things like gravity on the arm to cause it to slowly change the setpoint
            // downward.  Instead, the setpoint should remain constant until the joystick is moved.
            double deltaControllerValue = m_deltaArmAngleFromController.get() / ArmDefaultCommandConstants.armAngleChangeRate;

            // Only tell arm (which already uses a closed loop PID controller) to move if
            // joystick is moved
            if (deltaControllerValue != 0) {
                desiredAngle += deltaControllerValue;
                setArmAngle(desiredAngle);
            }

            return;
        }

        // $TODO - See bugfix made for simulation above.  We should fix this for the actual physical robot too.
        // Needs investigation.
        setArmAngle(m_armSystem.getArmAngleRelative() + (m_deltaArmAngleFromController.get() / ArmDefaultCommandConstants.armAngleChangeRate)); 
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
