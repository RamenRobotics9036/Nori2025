package frc.robot.commands.testcommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestSwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

class ModuleInfo {
    public int index;
    public SwerveModule obj;

    public ModuleInfo(int moduleIndex, SwerveModule moduleObj) {
        this.index = moduleIndex;
        this.obj = moduleObj;
    }
}

public class TestTurnWheel extends Command {
    private SwerveSubsystem m_swerveDriveSubsystem;
    private String m_moduleName;
    private Timer m_timer = new Timer();
    private static final double TURN_TIME_SECONDS = 0.5;
    private static final int CYCLES = 2; // Number of full back and forth cycles
    private ModuleInfo m_moduleInfo = null;
    private SwerveDrive m_swerve = null;

    // Constructor
    public TestTurnWheel(SwerveSubsystem swerveDriveSubsystem, String moduleName) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_moduleName = moduleName;

        addRequirements(m_swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        System.out.println("TestTurnWheel: initialize " + m_moduleName);

        m_swerve = m_swerveDriveSubsystem.getSwerveDrive();
        m_moduleInfo = getModuleByName(m_swerve.getModules(), m_moduleName);
    }

    @Override
    public void execute() {
        if (m_moduleInfo == null) {
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

        //m_swerveDriveSubsystem.getWheelTestContext().cancelRunningTestWithError("Oh no!");

        //System.out.println("Elapsed time: " + elapsed + "s, Angle: " + angleDegrees + " degrees");
        m_moduleInfo.obj.setAngle(angleDegrees);

        // In simulation, we need to update the current module state
        if (RobotBase.isSimulation()) {
            setAngleOnModules(angleDegrees);
        }
    }

    // Set the angles by calling setModuleStates.  Not sure if this works.
    private void setAngleOnModules(double angleDegrees) {
        // Retrieve current module states
        var currentStates = m_swerve.getStates();
        
        // Ensure the index is valid
        if (m_moduleInfo.index < 0 || m_moduleInfo.index >= currentStates.length) {
            throw new IllegalArgumentException("Invalid module index: " + m_moduleInfo.index);
        }
        
        // Preserve the current drive speed, update the angle
        var oldState = currentStates[m_moduleInfo.index];
        currentStates[m_moduleInfo.index] = new edu.wpi.first.math.kinematics.SwerveModuleState(
                oldState.speedMetersPerSecond,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(angleDegrees)
        );
        
        // Update module states (using open-loop control as false)
        m_swerve.setModuleStates(currentStates, true);
    }

    private ModuleInfo getModuleByName(SwerveModule[] moduleList, String name) {
        int index = 0;

        for (SwerveModule module : moduleList) {
            if (module.configuration.name.equals(name)) {
                return new ModuleInfo(index, module);
            }
            index++;
        }
        throw new IllegalArgumentException("No module with name " + name + " found");
    }

    @Override
    public boolean isFinished() {
        if (m_moduleInfo == null) {
            // End command if there isnt a module named appropriately
            return true;
        }

        double totalTime = 2 * TURN_TIME_SECONDS * CYCLES;

        // Safety check of N seconds
        if (totalTime > TestSwerveConstants.maxTimeSeconds) {
            throw new IllegalStateException("Total time: " + totalTime
                + ", Max allowed time: " + TestSwerveConstants.maxTimeSeconds);
        }
        if (m_timer.get() > TestSwerveConstants.maxTimeSeconds) {
            System.out.println("WARNING: TestTurnWheel timed out!");
            return true;
        }

        return m_timer.get() >= totalTime;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDriveSubsystem.stopSystem();
    }
}
