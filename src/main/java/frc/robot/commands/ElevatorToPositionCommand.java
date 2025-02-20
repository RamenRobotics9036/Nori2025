package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorToPositionCommand extends Command{
    private ElevatorSystem m_elevator;
    private double m_desiredPosition;
    // private Timer m_timer = new Timer();

    public ElevatorToPositionCommand(ElevatorSystem elevator, double desiredPosition){
        m_elevator = elevator;
        m_desiredPosition = desiredPosition;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // m_timer.restart();
        m_elevator.setPosition(m_desiredPosition);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}