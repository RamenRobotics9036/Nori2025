package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorToPositionCommand extends Command{
    private ElevatorSystem m_elevator;
    private double m_desiredPosition;
    private Timer m_timer = new Timer();

    public ElevatorToPositionCommand(ElevatorSystem elevator, double desiredPosition){
        m_elevator = elevator;
        m_desiredPosition = desiredPosition;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_elevator.setPosition(m_desiredPosition);
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        if (m_timer.get() > ElevatorConstants.maxTime) {
            return true;
        }
        return MathUtil.applyDeadband(m_desiredPosition - m_elevator.getPosition(), ElevatorConstants.tolerance) == 0;
    }
}