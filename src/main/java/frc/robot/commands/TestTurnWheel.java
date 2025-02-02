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
    private static final double TURN_TIME_SECONDS = 1.0;
    private static final int CYCLES = 2; // Number of full back and forth cycles
    private SwerveModule m_module = null;
    private SwerveDrive m_swerve = null;
    private int m_moduleIndex = -1;

    // Constructor
    public TestTurnWheel(SwerveSubsystem swerveDrive) {
        m_swerveDrive = swerveDrive;

        addRequirements(m_swerveDrive);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        System.out.println("TestTurnWheel: initialize");

        m_swerve = m_swerveDrive.getSwerveDrive();
        m_moduleIndex = getModuleIndexByName(m_swerve.getModules(), "frontleft");
        m_module = getModuleByName(m_swerve.getModules(), "frontleft");
    }

    @Override
    public void execute() {
        if (m_moduleIndex == -1) {
            return;
        }

        double elapsed = m_timer.get();
        double fullCycleTime = 2 * TURN_TIME_SECONDS;
        double phaseTime = elapsed % fullCycleTime;
        double angleDegrees = 0.0;
        
        if (phaseTime < TURN_TIME_SECONDS) {
            // Forward phase: 0 -> 90 with ease-in/ease-out via cosine easing:
            double fraction = phaseTime / TURN_TIME_SECONDS;
            angleDegrees = 90.0 * (1 - Math.cos(Math.PI * fraction)) / 2.0;
        } else {
            // Backward phase: 90 -> 0 with ease-in/ease-out:
            double fraction = (phaseTime - TURN_TIME_SECONDS) / TURN_TIME_SECONDS;
            angleDegrees = 90.0 * (1 + Math.cos(Math.PI * fraction)) / 2.0;
        }
        
        System.out.println("Elapsed time: " + elapsed + "s, Angle: " + angleDegrees + " degrees");
        m_module.setAngle(angleDegrees);
        setAngleOnModules(angleDegrees);
    }

    // Set the angles by calling setModuleStates.  Not sure if this works.
    private void setAngleOnModules(double angleDegrees) {
        // Retrieve current module states
        var currentStates = m_swerve.getStates();
        
        // Ensure the index is valid
        if (m_moduleIndex < 0 || m_moduleIndex >= currentStates.length) {
            throw new IllegalArgumentException("Invalid module index: " + m_moduleIndex);
        }
        
        // Preserve the current drive speed, update the angle
        var oldState = currentStates[m_moduleIndex];
        currentStates[m_moduleIndex] = new edu.wpi.first.math.kinematics.SwerveModuleState(
                oldState.speedMetersPerSecond,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(angleDegrees)
        );
        
        // Update module states (using open-loop control as false)
        m_swerve.setModuleStates(currentStates, true);
    }

    private SwerveModule getModuleByName(SwerveModule[] moduleList, String name) {
        for (SwerveModule module : moduleList) {
            if (module.configuration.name.equals(name)) {
                return module;
            }
        }
        throw new IllegalArgumentException("No module with name " + name + " found");
    }

    private int getModuleIndexByName(SwerveModule[] moduleList, String name) {
        int index = 0;

        for (SwerveModule module : moduleList) {
            if (module.configuration.name.equals(name)) {
                return index;
            }
            index++;
        }
        throw new IllegalArgumentException("No module with name " + name + " found");
    }

    @Override
    public boolean isFinished() {
        if (m_moduleIndex == -1) {
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
