package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmDefaultCommandConstants;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.IntakeArmSystem;

public class ElevatorManualCommand extends Command {
    private ElevatorSystem m_elevatorSystem;
    private Supplier<Double> m_deltaArmAngleFromController;
    private double desiredAngle = ArmConstants.kMinArmRotation;

    public ElevatorManualCommand(ElevatorSystem elevatorSystem, Supplier<Double> deltaArmAngleFromController) {
        m_elevatorSystem = elevatorSystem;
        m_deltaArmAngleFromController = deltaArmAngleFromController;
        addRequirements(m_elevatorSystem);
    }

    @Override
    public void initialize() {
        desiredAngle = m_elevatorSystem.getPosition();

        // Call setPoint to set the arm-motor PID loop to the desired angle.
        setPosition(desiredAngle);
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
                setPosition(desiredAngle);
            }

            return;
        }

        // $TODO - See bugfix made for simulation above.  We should fix this for the actual physical robot too.
        // Needs investigation. Shlok commented the next line out at 3/27/25 because it was causing the Elevator to slowly descend. This was because the desired position of the elevator was slowly decreasing without any human input.
        // setPosition(m_elevatorSystem.getPosition() + (m_deltaArmAngleFromController.get() / 20)); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSystem.stopSystem();
    }

    private void setPosition(double value) {
        // value = MathUtil.clamp(value, 0, 1.5);
        desiredAngle = value;
        m_elevatorSystem.setPosition(desiredAngle);
    }
}
