package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeDefaultCommandConstants;
import frc.robot.subsystems.IntakeSystem;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class IntakeDefaultCommand extends Command {
    private IntakeSystem m_intake;

    public IntakeDefaultCommand(IntakeSystem intake){
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        if (!m_intake.isHoldingCoral()) {
            m_intake.setPullMotorSpeed(-IntakeDefaultCommandConstants.speed);
            m_intake.setLoadMotorSpeed(-IntakeDefaultCommandConstants.speed);
        } else{
            m_intake.setPullMotorSpeed(0);
            m_intake.setLoadMotorSpeed(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopSystem();
    }   
}
