package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.IntakeArmSystem;

// Resets the encoder position. Encoder may be inncacurate if tooth is skipped.
public class ArmHomeComamand extends Command{
    private IntakeArmSystem m_arm;
    private boolean m_atTop;

    public ArmHomeComamand(IntakeArmSystem arm, boolean atTop){
        m_arm = arm;
        m_atTop = atTop;

        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
        if(m_atTop){
            m_arm.setArmAngle(ArmConstants.);
        } else{

        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }
}