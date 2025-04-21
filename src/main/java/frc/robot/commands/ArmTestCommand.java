package frc.robot.commands; 

import edu.wpi.first.wpilibj2.command.Command; 
import frc.robot.subsystems.IntakeArmSystem; 
import java.util.function.Supplier; 
 
//Command for testing arm woithout relying on PID 
@SuppressWarnings({"all"}) // suppress CheckStyle warnings in this file
public class ArmTestCommand extends Command { 
    private IntakeArmSystem m_armSystem; 
    private Supplier<Double> m_deltaArmAngleFromController; 
 
    public ArmTestCommand(IntakeArmSystem armSystem, Supplier<Double> deltaArmAngleFromController){ 
        m_armSystem = armSystem; 
        m_deltaArmAngleFromController = deltaArmAngleFromController; 
 
        addRequirements(m_armSystem); 
    } 
 
    @Override 
    public void initialize(){ 
 
    } 
 
    @Override 
    public void execute(){ 
        m_armSystem.setArmMotorSpeed(m_deltaArmAngleFromController.get()); 
    } 
 
    @Override 
    public boolean isFinished(){ 
        return false; 
    } 
 
    @Override 
    public void end(boolean interrupted){ 
         
    } 
} 
