package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorToPositionCommand extends Command{
    private ElevatorSystem m_elevator;
    private double m_position;
    public ElevatorToPositionCommand(ElevatorSystem elevator, double position){
        m_elevator = elevator;
        m_position = position;
        addRequirements(m_elevator);
    }
}
