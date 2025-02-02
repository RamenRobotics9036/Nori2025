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
    private static final double TURN_TIME_SECONDS = 4.0;
    private static final int CYCLES = 4; // Number of full back and forth cycles
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

        double elapsed = m_timer.get();
        double fullCycleTime = 2 * TURN_TIME_SECONDS;
        double phaseTime = elapsed % fullCycleTime;
        double angleDegrees = 0.0;
        
        if (phaseTime < TURN_TIME_SECONDS) {
            // Forward phase: 0 -> 90 using sinusoidal interpolation
            double fraction = phaseTime / TURN_TIME_SECONDS;
            angleDegrees = 90.0 * Math.sin((Math.PI / 2.0) * fraction);
        } else {
            // Backward phase: 90 -> 0 using sinusoidal interpolation
            double fraction = (phaseTime - TURN_TIME_SECONDS) / TURN_TIME_SECONDS;
            angleDegrees = 90.0 * Math.cos((Math.PI / 2.0) * fraction);
        }
        
        System.out.println("Elapsed time: " + elapsed + "s, Angle: " + angleDegrees + " degrees");
        m_module.setAngle(angleDegrees);
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

        double totalTime = 2 * TURN_TIME_SECONDS * CYCLES;

        // Safety check of N seconds
        if (totalTime > TestSwerveConstants.maxTimeSeconds) {
            throw new IllegalStateException("Total time exceeds maximum allowed time. Total time: " + totalTime + ", Max allowed time: " + TestSwerveConstants.maxTimeSeconds);
        }
        if (m_timer.get() > TestSwerveConstants.maxTimeSeconds) {
            return true;
        }

        return m_timer.get() >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.stopSystem();
    }
}
