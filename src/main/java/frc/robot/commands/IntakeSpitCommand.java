package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeSpitCommandConstants;
import frc.robot.subsystems.IntakeSystem;

@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class IntakeSpitCommand extends Command {
    private IntakeSystem m_intake;
    private Timer m_timer = new Timer();
    private double m_startingRotations;
    private double m_speed;
    private boolean canFinish = false;

    // NECESSARILY requires two parameters: IntakeSystem and a double containing a speed value
    // Currently has multiple presets defined in Constants.java: speed and bucketSpeed
    // m_speed refers to how fast the rollers spin
    // This function is meant to be used to spinn the rollers quickly for a moment to eject Coral
    // To deposit Coral into the bucket, this function is fed a negative value, and to eject Coral it's fed a positive one
    public IntakeSpitCommand(IntakeSystem intake, double speed, boolean canFinish){
        System.out.print("##### Intake1");
        m_speed = speed;
        m_intake = intake;
        this.canFinish = canFinish;
        addRequirements(m_intake);
    }

    public IntakeSpitCommand(IntakeSystem intake, double speed){
        System.out.print("##### Intake2");
        m_speed = speed;
        m_intake = intake;
        addRequirements(m_intake);
    }

    @Override
    public void initialize(){
        System.out.print("##### Intake3");
        m_timer.restart();
        m_startingRotations = m_intake.getPullMotorPosition();
    }

    @Override
    public void execute(){
        // Reversed because spitting out
        m_intake.setPullMotorSpeed(-m_speed);
        m_intake.setLoadMotorSpeed(m_speed);
    }

    @Override
    public boolean isFinished(){
        if (canFinish) {
            if (m_timer.get() > IntakeSpitCommandConstants.maxTime) {
                System.out.println("WARNING: IntakeSpitCommand timed out!");
                return true;
            }
            if (Math.abs((m_intake.getPullMotorPosition() - m_startingRotations)) / IntakeConstants.pullMotorGearBoxFactor > IntakeSpitCommandConstants.numRotations) {
                System.out.println("WARNING: IntakeSpitCommand ROTATED motor too many times!");
                return true;
            }
        }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_intake.stopSystem();
    }   
}
