package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;

//IMPORTANT: Just for testing. Not real command.
public class IntakeTestCommand extends Command{
    private IntakeSystem m_intake;

    public IntakeTestCommand(IntakeSystem intake){
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        //0.3 is a filler value. Probably fine for a random test command.
        m_intake.setPullMotorSpeed(0.3);
        m_intake.setLoadMotorSpeed(0.3);
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