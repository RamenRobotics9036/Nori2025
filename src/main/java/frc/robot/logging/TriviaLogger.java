package frc.robot.logging;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.OperatorConstants;

public class TriviaLogger {
    private DoubleLogEntry m_voltageLog = null;
    private DoubleLogEntry m_canBusUtilizationLog = null;

    // Constructor
    public TriviaLogger() {
        enableLogging();
    }

    private boolean isNTLoggingEnabled() {
        // We ALWAYS log to roborio logs anything that was place on the NetworkTables:
        // If it was important enough to send over the NETWORK, logging it is nearly free.
        return true;
    }

    private boolean isDriverStationLoggingEnabled() {
        return !isCompetitionMode();
    }

    private boolean isPerformanceDataLoggingEnabled() {
        return !isCompetitionMode();
    }

    private boolean isPowerLoggingEnabled() {
        return !isCompetitionMode();
    }

    private boolean isCompetitionMode() {
        return OperatorConstants.kCompetitionMode;
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
    }

    private void updatePerformanceDataLogging() {
        if (m_canBusUtilizationLog != null) {
           // Note we use update so that it only logs on change.
            m_canBusUtilizationLog.update(RobotController.getCANStatus().percentBusUtilization);
        }
    }
}
