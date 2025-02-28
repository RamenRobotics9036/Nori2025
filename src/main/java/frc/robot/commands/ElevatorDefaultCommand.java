package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorDefaultCommand extends Command {
    private ElevatorSystem m_elevatorSystem;
    private Supplier<Double> m_deltaElevatorDistanceFromController;
    private double desiredAngle = ElevatorConstants.kDownElevatorPosition;

    public ElevatorDefaultCommand(ElevatorSystem elevatorSystem, Supplier<Double> deltaElevatorDistanceFromController) {
        m_elevatorSystem = elevatorSystem;
        m_deltaElevatorDistanceFromController = deltaElevatorDistanceFromController;
        addRequirements(m_elevatorSystem);
    }

    @Override
    public void initialize() {
        desiredAngle = m_elevatorSystem.getPosition();
    }

    @Override
    public void execute() {
        setArmAngle(m_elevatorSystem.getPosition() + (m_deltaElevatorDistanceFromController.get() / ElevatorConstants.elevatorAngleChange));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevatorSystem.stopSystem();
    }

    public void setArmAngle(double value) {
        value = MathUtil.clamp(value, ElevatorConstants.kDownElevatorPosition, ElevatorConstants.kMaxElevatorPosition);
        desiredAngle = value;
        m_elevatorSystem.setPosition(desiredAngle);
    }
}
