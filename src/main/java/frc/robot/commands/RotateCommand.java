package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
}
