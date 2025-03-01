package frc.robot.commands.testcommands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TestSwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.testcommands.MyModuleInfo;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class TestSanityChecks extends Command {
    private SwerveSubsystem m_swerveDriveSubsystem;
    private String m_moduleName;
    private Timer m_timer = new Timer();
    private MyModuleInfo m_moduleInfo = null;
    private SwerveDrive m_swerve = null;
    private int m_runCount = 0;

    // Constructor
    public TestSanityChecks(SwerveSubsystem swerveDriveSubsystem, String moduleName) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_moduleName = moduleName;

        addRequirements(m_swerveDriveSubsystem);
    }

    @Override
    public void initialize() {
        m_timer.restart();

        System.out.println("TestSanityChecks: initialize " + m_moduleName);

        m_swerve = m_swerveDriveSubsystem.getSwerveDrive();
        m_moduleInfo = getModuleByName(m_swerve.getModules(), m_moduleName);
    }

    private boolean checkCanReadEncoder() {
        boolean isEncoderUnhealthy = m_moduleInfo.obj.getAbsoluteEncoderReadIssue();
        if (isEncoderUnhealthy) {
            String error_str = "TestSanityChecks: " + m_moduleName + " encoder cant be read.";
            System.out.println(error_str);
            m_swerveDriveSubsystem.getWheelTestContext().cancelRunningTestWithError(error_str);
            return false;
        }

        System.out.println("TestSanityChecks: " + m_moduleName + " encoder successfully read.");
        return true;
    }

    @Override
    public void execute() {
        m_runCount++;

        if (m_moduleInfo == null) {
            return;
        }

        if (!checkCanReadEncoder()) {
            return;
        }
    }

    private MyModuleInfo getModuleByName(SwerveModule[] moduleList, String name) {
        int index = 0;

        for (SwerveModule module : moduleList) {
            if (module.configuration.name.equals(name)) {
                return new MyModuleInfo(index, module);
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

        if (m_runCount >= 1) {
            // End command if it has run once
            return true;
        }

        if (m_timer.get() > TestSwerveConstants.maxTimeSeconds) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDriveSubsystem.stopSystem();
    }
}
