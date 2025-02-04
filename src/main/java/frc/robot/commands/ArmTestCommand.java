package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSystem;

//IMPORTANT: Just for testing. Not real command.
public class ArmTestCommand extends Command{
    private IntakeArmSystem m_arm;
    private boolean m_isFinished = false;

    public ArmTestCommand(IntakeArmSystem system){
        m_arm = system;
        addRequirements(m_arm);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        //0.3 is a filler value. Probably fine for a random test command.
        System.out.println(m_arm.getArmAngle());
        m_isFinished = true;
    }

    @Override
    public boolean isFinished(){
        return m_isFinished;
    }

    @Override
    public void end(boolean interrupted){
        m_arm.stopSystem();
    }
}