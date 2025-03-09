package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorDefaultCommandConstants;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorDefaultCommand extends Command{
    private ElevatorSystem m_elevator;
    private DoubleSupplier m_joystick;

    public ElevatorDefaultCommand(ElevatorSystem elevator, DoubleSupplier joystick){
        m_elevator = elevator;
        m_joystick = joystick;

        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_elevator.setPosition(
            MathUtil.clamp(m_elevator.getPosition() + m_joystick.getAsDouble() * ElevatorDefaultCommandConstants.kElevatorSpeed, //TODO: adjust value
            ElevatorConstants.kDownElevatorPosition,
            ElevatorConstants.kMaxElevatorPosition));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_elevator.stopSystem();
    }
}
