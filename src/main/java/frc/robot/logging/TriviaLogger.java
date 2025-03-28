package frc.robot.logging;

import java.util.function.Supplier;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
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
    private IntegerLogEntry m_FaultCount3V = null;
    private IntegerLogEntry m_FaultCount5V = null;
    private IntegerLogEntry m_FaultCount6V = null;
    private DoubleLogEntry m_canBusUtilizationLog = null;
    private IntegerLogEntry m_canBusReceiveErrorCountLog = null;
    private IntegerLogEntry m_canBusTransmitErrorCountLog = null;
    private StringLogEntry m_radioLEDStateLog = null;
    private PDData m_pdData = null;
    private StructLogEntry<PDData> m_powerDistributionLog = null;
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
        DataLogManager.logNetworkTables(isNTLoggingEnabled());

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
        String roboRioSN = RobotController.getSerialNumber();
        if (roboRioSN == null || roboRioSN.isEmpty()) {
            roboRioSN = "Unknown";
        }
        System.out.println("RoboRio SN: " + roboRioSN);
    }

    public void updateLogging() {
        updatePowerLogging();
        updateDetailedPowerLogging();
        updatePerformanceDataLogging();
        updateCommandLogging();
    }
    
    public void registerSubsystemCmdCallback(String subsystemName, Supplier<String> commandNameSupplier) {
        if (m_callbackLoggerForCommands != null) {
            String fullPath = "/my/Commands/BySubsystem/" + (subsystemName == null ? "None" : subsystemName);
            m_callbackLoggerForCommands.add(fullPath, commandNameSupplier);
        }
    }

    private void initPowerLogging() {
        if (isPowerLoggingEnabled()) {
            m_voltageLog = new DoubleLogEntry(DataLogManager.getLog(), "/my/Power/Voltage");
            m_FaultCount3V = new IntegerLogEntry(DataLogManager.getLog(), "/my/Power/FaultCount3V");
            m_FaultCount5V = new IntegerLogEntry(DataLogManager.getLog(), "/my/Power/FaultCount5V");
            m_FaultCount6V = new IntegerLogEntry(DataLogManager.getLog(), "/my/Power/FaultCount6V");
        }
    }

    private void initDetailedPowerLogging() {
        if (isDetailedPowerLoggingEnabled()) {
            // Enable power distribution logging
            m_pdData = PDData.create(1, ModuleType.kRev);
            m_powerDistributionLog = StructLogEntry.create(DataLogManager.getLog(), "/my/PowerDistribution", PDData.struct);
        }
    }

    private void initPerformanceDataLogging() {
        if (isPerformanceDataLoggingEnabled()) {
            m_canBusUtilizationLog = new DoubleLogEntry(DataLogManager.getLog(), "/my/CAN_Bus/Utilization");
            m_canBusReceiveErrorCountLog = new IntegerLogEntry(DataLogManager.getLog(), "/my/CAN_Bus/ReceiveErrorCount");
            m_canBusTransmitErrorCountLog = new IntegerLogEntry(DataLogManager.getLog(), "/my/CAN_Bus/TransmitErrorCount");
            m_radioLEDStateLog = new StringLogEntry(DataLogManager.getLog(), "/my/RadioLEDState");
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
        if (m_FaultCount3V != null) {
            m_FaultCount3V.update(RobotController.getFaultCount3V3());
        }
        if (m_FaultCount5V != null) {
            m_FaultCount5V.update(RobotController.getFaultCount5V());
        }
        if (m_FaultCount6V != null) {
            m_FaultCount6V.update(RobotController.getFaultCount6V());
        }
    }

    private void updateDetailedPowerLogging() {
        if (m_pdData != null) {
            m_pdData.update();
            m_powerDistributionLog.update(m_pdData);
        }
    }

    private void updatePerformanceDataLogging() {
        CANStatus canStatus;

        if (m_canBusUtilizationLog != null) {
            canStatus = RobotController.getCANStatus();

            // Note we use update so that it only logs on change.
            m_canBusUtilizationLog.update(canStatus.percentBusUtilization);
            m_canBusReceiveErrorCountLog.update(canStatus.receiveErrorCount);
            m_canBusTransmitErrorCountLog.update(canStatus.transmitErrorCount);
        }
        if (m_radioLEDStateLog != null) {
            m_radioLEDStateLog.update(RobotController.getRadioLEDState().toString());
        }
    }

    private void updateCommandLogging() {
        if (m_callbackLoggerForCommands != null) {
            m_callbackLoggerForCommands.update();
        }
    }
}
