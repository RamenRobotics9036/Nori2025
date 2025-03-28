package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.CommandAppliedController;

public class ControllerRumbleCommand extends Command {
    private double m_time;
    private double m_intensity;
    private CommandAppliedController m_controller;
    private Timer m_timer = new Timer();

    public ControllerRumbleCommand(CommandAppliedController controller, double time, double intensity){
        m_controller = controller;
        m_time = time;
        m_intensity = intensity;
    }

    public ControllerRumbleCommand(CommandAppliedController controller, double time){
        this(controller, time, 0.3);
    }

    @Override
    public void initialize(){
        m_timer.restart();
    }
    
    @Override
    public void execute(){
        m_controller.setRumble(GenericHID.RumbleType.kBothRumble, m_intensity);
    }

    @Override
    public boolean isFinished(){
        return m_timer.get() > m_time;
    }

    public void end(boolean interrupted){
        m_controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }
}
