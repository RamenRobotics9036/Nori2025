package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateCommand extends Command{
    private Timer m_Timer =  new Timer();
    private SwerveSubsystem m_swerve;
    private double m_desiredAngle;

    public RotateCommand(SwerveSubsystem swerve, double desiredAngle){
        m_swerve = swerve;
        m_desiredAngle = desiredAngle;

        addRequirements(m_swerve);
    }

    public RotateCommand(SwerveSubsystem swerve){
        m_swerve = swerve;
        m_desiredAngle = 90;

        addRequirements(m_swerve);
    }

    @Override
    public void initialize() {
        m_desiredAngle += m_swerve.getPose().getRotation().getDegrees();

    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    @Override
    public boolean isFinished() {
        //timeout
        if (m_Timer.get() >= AutoConstants.kRotTimeout) { //TODO: filler value
            return true;
        }
        return MathUtil.applyDeadband(m_swerve.getPose().getRotation().getDegrees() - m_desiredAngle, AutoConstants.kRotMarginOfError) == 0; //TODO: filler value
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stopSystem();
    }
}
