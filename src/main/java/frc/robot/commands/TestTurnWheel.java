package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestSwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TestTurnWheel extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Timer m_timer = new Timer();

    // Constructor
    public TestTurnWheel(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (m_timer.get() > TestSwerveConstants.maxTimeSeconds) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
