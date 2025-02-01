package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestSwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class TestTurnWheel extends Command {
    private SwerveSubsystem m_swerveDrive;
    private Timer m_timer = new Timer();
    private static final double TURN_TIME_SECONDS = 2.0;
    private SwerveModule m_module = null;

    // Constructor
    public TestTurnWheel(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        SwerveModule[] moduleList = m_swerveDrive.getSwerveDrive().getModules();
        m_module = getSingleModuleByName(moduleList, "frontleft");
    }

    @Override
    public void execute() {
        if (m_module == null) {
            return;
        }

        double fraction = Math.min(m_timer.get() / TURN_TIME_SECONDS, 1.0);
        double angleDegrees = fraction * 90.0;
    }

    private SwerveModule getSingleModuleByName(SwerveModule[] moduleList, String name) {
        for (SwerveModule module : moduleList) {
            if (module.configuration.name.equals(name)) {
                return module;
            }
        }
        return null;
    }

    @Override
    public boolean isFinished() {
        if (m_module == null) {
            // End command if there isnt a module named appropriately
            return true;
        }

        if (m_timer.get() > TestSwerveConstants.maxTimeSeconds) {
            return true;
        }

        return m_timer.get() >= TURN_TIME_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
