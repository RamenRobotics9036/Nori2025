package frc.robot.logging;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.OperatorConstants;

public class TriviaLogger {
    // Singleton instance
    private static TriviaLogger m_instance = null;

    private DoubleLogEntry m_voltageLog = null;
    private DoubleLogEntry m_canBusUtilizationLog = null;
    private PDData m_pdData = null;
    private StructLogEntry<PDData> m_powerDistributionLog = null;

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

    private boolean isNTLoggingEnabled() {
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

    private boolean isSimOrNotCompetitionMode() {
        return RobotBase.isSimulation() || !OperatorConstants.kCompetitionMode;
    }
    
    private void enableLogging() {
        // Optionally enable logging of all NetworkTables data
        DataLogManager.logNetworkTables(isNTLoggingEnabled());

        // Starts recording to data log
        DataLogManager.start();

        // Record both DS control and joystick data
        if (isDriverStationLoggingEnabled()) {
            DriverStation.startDataLog(DataLogManager.getLog());
        }

        // Custom logging
        initPowerLogging();
        initPerformanceDataLogging();

        System.out.println("TriviaLogging enabled!");
    }

    public void updateLogging() {
        updatePowerLogging();
        updatePerformanceDataLogging();
    }
    
    private void initPowerLogging() {
        if (isPowerLoggingEnabled()) {
            m_voltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/my/Voltage");

            // Enable power distribution logging
            m_pdData = PDData.create(1, ModuleType.kRev);
            m_powerDistributionLog = StructLogEntry.create(DataLogManager.getLog(), "/my/PowerDistribution", PDData.struct);
        }
    }

    private void initPerformanceDataLogging() {
        if (isPerformanceDataLoggingEnabled()) {
            m_canBusUtilizationLog = new DoubleLogEntry(DataLogManager.getLog(), "/my/CAN_Bus_Utilization");
        }
    }

    private void updatePowerLogging() {
        if (m_voltageLog != null) {
            // Note we use update so that it only logs on change.
            m_voltageLog.update(RobotController.getBatteryVoltage());
        }
        if (m_pdData != null) {
            m_pdData.update();
            m_powerDistributionLog.update(m_pdData);
        }
    }

    private void updatePerformanceDataLogging() {
        if (m_canBusUtilizationLog != null) {
           // Note we use update so that it only logs on change.
            m_canBusUtilizationLog.update(RobotController.getCANStatus().percentBusUtilization);
        }
    }
}
