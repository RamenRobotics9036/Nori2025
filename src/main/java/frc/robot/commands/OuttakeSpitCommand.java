package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OuttakeConstants;
import frc.robot.Constants.OuttakeSpitCommandConstants;
import frc.robot.subsystems.OuttakeSystem;

public class OuttakeSpitCommand extends Command {
    private OuttakeSystem m_outtake;
    private Timer m_timer = new Timer();
    private double m_startingRotations;
    private double speed;

    public OuttakeSpitCommand(OuttakeSystem outtake, double speed){
        m_outtake = outtake;
        this.speed = speed;
        addRequirements(m_outtake);
    }

    @Override
    public void initialize(){
        m_timer.restart();
        m_startingRotations = m_outtake.getLeaderPosition();
    }

    @Override
    public void execute(){
        // Reversed because spitting out
        m_outtake.setMotorSpeeds(speed);
    }

    @Override
    public boolean isFinished(){
        //checks if command has been running for too long...
        // if (m_timer.get() > OuttakeSpitCommandConstants.maxTime) {
        //     System.out.println("WARNING: OuttakeSpitCommand timed out!");
        //     return true;
        // }
        // //...or for too many rotations.
        // if (Math.abs((m_outtake.getLeaderPosition() - m_startingRotations)) / OuttakeConstants.motorGearRatio > OuttakeSpitCommandConstants.numRotations) {
        //     System.out.println("WARNING: OuttakeSpitCommand ROTATED motor too many times!");
        //     return true;
        // }
        return false;
    }

    @Override
    public void end(boolean interrupted){
        m_outtake.setMotorSpeeds(0);
    }   
}
