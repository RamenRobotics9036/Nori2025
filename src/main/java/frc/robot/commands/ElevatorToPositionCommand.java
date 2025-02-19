package frc.robot.commands;

import java.security.KeyStore.TrustedCertificateEntry;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSystem;

public class ElevatorToPositionCommand extends Command{
    private ElevatorSystem m_elevator;
    private double m_desiredPosition;
    public ElevatorToPositionCommand(ElevatorSystem elevator, double desiredPosition){
        m_elevator = elevator;
        m_desiredPosition = desiredPosition;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize(){
        m_elevator.setPosition(m_desiredPosition);
    }

    @Override
    public boolean isFinished(){
        return
            m_elevator.getPosition() >= m_desiredPosition - ElevatorConstants.kMarginOfError &&
            m_elevator.getPosition() <= m_desiredPosition + ElevatorConstants.kMarginOfError;
    }
}