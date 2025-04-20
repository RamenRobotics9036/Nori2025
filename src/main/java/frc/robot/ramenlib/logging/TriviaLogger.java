package frc.robot.ramenlib.logging;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.OperatorConstants;
import java.util.function.Supplier;

/**
 * The TriviaLogger class is responsible for managing various logging functionalities
 * such as power, performance, and command logging in the robot system.
 */
public class TriviaLogger {
    // Singleton instance
    private static TriviaLogger m_instance = null;

    private DoubleLogEntry m_voltageLog = null;
    private IntegerLogEntry m_faultCount3V = null;
    private IntegerLogEntry m_faultCount5V = null;
    private IntegerLogEntry m_faultCount6V = null;
    private PDData m_pdData = null;
    private StructLogEntry<PDData> m_powerDistributionLog = null;

    @SuppressWarnings("unused")
    private CommandLogger m_commandLogger = null;
    private CallbackLogger m_callbackLoggerForCommands = null;

    /**
     * Private constructor to enforce singleton pattern.
     * Users must use getInstance() instead of creating new instances.
     */
    private TriviaLogger() {
        enableLogging();
    }

    /**
     * Get the singleton instance of TriviaLogger.
     * This is the only way to obtain a TriviaLogger instance.
     */
    public static synchronized TriviaLogger getInstance() {
        if (m_instance == null) {
            m_instance = new TriviaLogger();
        }
        return m_instance;
    }

    private boolean isNtLoggingEnabled() {
        // We ALWAYS log to roborio logs anything that was place on the NetworkTables:
        // If it was important enough to send over the NETWORK, logging it is nearly free.
        return true;
    }

    private boolean isDriverStationLoggingEnabled() {
        return isSimOrNotCompetitionMode();
    }

    private boolean isPerformanceDataLoggingEnabled() {
        return isSimOrNotCompetitionMode();
    }

    private boolean isPowerLoggingEnabled() {
        return isSimOrNotCompetitionMode();
    }

    private boolean isDetailedPowerLoggingEnabled() {
        if (!OperatorConstants.kAllowDetailedPowerLogging) {
            return false;
        }

        return isSimOrNotCompetitionMode();
    }

    private boolean isCommandLoggingEnabled() {
        return isSimOrNotCompetitionMode();
    }

    private boolean isSimOrNotCompetitionMode() {
        return RobotBase.isSimulation() || !OperatorConstants.kCompetitionMode;
    }

    private void enableLogging() {
        // Optionally enable logging of all NetworkTables data
        DataLogManager.logNetworkTables(isNtLoggingEnabled());

        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        if (isDriverStationLoggingEnabled()) {
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        // Custom logging
        initPowerLogging();
        initDetailedPowerLogging();
        initPerformanceDataLogging();
        initCommandLogging();

        System.out.println("TriviaLogging enabled!");
        String roboRioSn = RobotController.getSerialNumber();
        if (roboRioSn == null || roboRioSn.isEmpty()) {
            roboRioSn = "Unknown";
        }
        System.out.println("RoboRio SN: " + roboRioSn);
    }

    /**
     * Updates all logging functionalities, including power, detailed power,
     * performance data, and command logging. Called every 20ms on periodic
     * loop.
     */
    public void updateLogging() {
        updatePowerLogging();
        updateDetailedPowerLogging();
        updatePerformanceDataLogging();
        updateCommandLogging();
    }

    /**
     * Registers a callback for a subsystem to log the currently running command.
     *
     * @param subsystemName       The name of the subsystem. If null, "None" will be used.
     * @param commandNameSupplier A supplier that provides the name of the current command.
     */
    public void registerSubsystemCmdCallback(
        String subsystemName,
        Supplier<String> commandNameSupplier) {

        if (m_callbackLoggerForCommands != null) {
            String fullPath = "/my/Commands/BySubsystem/"
                + (subsystemName == null ? "None" : subsystemName);
            m_callbackLoggerForCommands.add(fullPath, commandNameSupplier);
        }
    }

    private void initPowerLogging() {
        if (isPowerLoggingEnabled()) {
            m_voltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/my/Power/Voltage");
            m_faultCount3V = new IntegerLogEntry(DataLogManager.getLog(), "/my/Power/FaultCount3V");
            m_faultCount5V = new IntegerLogEntry(DataLogManager.getLog(), "/my/Power/FaultCount5V");
            m_faultCount6V = new IntegerLogEntry(DataLogManager.getLog(), "/my/Power/FaultCount6V");
        }
    }

    private void initDetailedPowerLogging() {
        if (isDetailedPowerLoggingEnabled()) {
            // Enable power distribution logging
            m_pdData = PDData.create(1, ModuleType.kRev);
            m_powerDistributionLog = StructLogEntry
                .create(DataLogManager.getLog(), "/my/PowerDistribution", PDData.struct);
        }
    }

    private void initPerformanceDataLogging() {
        if (isPerformanceDataLoggingEnabled()) {
            // Nothing yet
        }
    }

    private void initCommandLogging() {
        if (isCommandLoggingEnabled()) {
            m_commandLogger = new CommandLogger();

            // Use this to register callbacks to log various Commands.
            m_callbackLoggerForCommands = new CallbackLogger();
        }
    }

    private void updatePowerLogging() {
        if (m_voltageLog != null) {
            // Note we use update so that it only logs on change.
            m_voltageLog.update(RobotController.getBatteryVoltage());
        }
        if (m_faultCount3V != null) {
            m_faultCount3V.update(RobotController.getFaultCount3V3());
        }
        if (m_faultCount5V != null) {
            m_faultCount5V.update(RobotController.getFaultCount5V());
        }
        if (m_faultCount6V != null) {
            m_faultCount6V.update(RobotController.getFaultCount6V());
        }
    }

    private void updateDetailedPowerLogging() {
        if (m_pdData != null) {
            m_pdData.update();
            m_powerDistributionLog.update(m_pdData);
        }
    }

    private void updatePerformanceDataLogging() {
        // Nothing yet
    }

    private void updateCommandLogging() {
        if (m_callbackLoggerForCommands != null) {
            m_callbackLoggerForCommands.update();
        }
    }
}
