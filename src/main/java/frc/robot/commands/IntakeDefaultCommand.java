package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeDefaultCommandConstants;
import frc.robot.subsystems.IntakeSystem;

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
            m_intake.setPullMotorSpeed(-IntakeDefaultCommandConstants.kSpeed);
            m_intake.setLoadMotorSpeed(-IntakeDefaultCommandConstants.kSpeed);
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
