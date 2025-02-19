package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeSpitCommandConstants;
import frc.robot.subsystems.IntakeSystem;

public class IntakeSpitCommand extends Command {
    private IntakeSystem m_intake;
    private Timer m_timer = new Timer();
    private double m_startingRotations;
    private double m_speed;

    public IntakeSpitCommand(IntakeSystem intake, double speed){
        m_speed = speed;
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        m_timer.restart();
        m_startingRotations = m_intake.getPullMotorPosition();
    }

    @Override
    public void execute(){
        // Reversed because spitting out
        m_intake.setPullMotorSpeed(m_speed);
        m_intake.setLoadMotorSpeed(-m_speed);
    }

    @Override
    public boolean isFinished(){
        //checks if command has been running for too long...
        if (m_timer.get() > IntakeSpitCommandConstants.maxTime) {
            return true;
        }
        //...or for too many rotations.
        if (Math.abs((m_intake.getPullMotorPosition() - m_startingRotations)) / IntakeConstants.pullMotorGearBoxFactor > IntakeSpitCommandConstants.numRotations) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopSystem();
    }   
}
