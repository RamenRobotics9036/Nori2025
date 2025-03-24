package frc.robot.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class CallbackLogger {
    // Inner class for string log items
    private class LogItemString {
        private final Supplier<String> stringSupplier;
        private final StringLogEntry logEntryString;
        
        public LogItemString(Supplier<String> stringSupplier, StringLogEntry logEntryString) {
            this.stringSupplier = stringSupplier;
            this.logEntryString = logEntryString;
        }
        
        public void logNewValue() {
            // Note that we only log CHANGES
            String newValue = stringSupplier.get();
            logEntryString.update(newValue);
        }
    }
    
    // Inner class for double log items
    private class LogItemDouble {
        private final DoubleSupplier doubleSupplier;
        private final DoubleLogEntry logEntryDouble;
        
        public LogItemDouble(DoubleSupplier doubleSupplier, DoubleLogEntry logEntryDouble) {
            this.doubleSupplier = doubleSupplier;
            this.logEntryDouble = logEntryDouble;
        }
        
        public void logNewValue() {
            // Note that we only log CHANGES
            double newValue = doubleSupplier.getAsDouble();
            logEntryDouble.update(newValue);
        }
    }

    // Members
    private final DataLog m_dataLog;
    private final Map<String, LogItemDouble> doubleEntries = new HashMap<>();
    private final Map<String, LogItemString> stringEntries = new HashMap<>();

    // Constructor
    public CallbackLogger() {
        m_dataLog = DataLogManager.getLog();
    }

    /**
     * Adds a new numeric logging entry
     */
    public void add(String path, DoubleSupplier supplier) {
        DoubleLogEntry logEntry = new DoubleLogEntry(m_dataLog, path);
        doubleEntries.put(path, new LogItemDouble(supplier, logEntry));
    }
    
    /**
     * Adds a new string logging entry
     */
    public void add(String path, Supplier<String> supplier) {
        StringLogEntry logEntry = new StringLogEntry(m_dataLog, path);
        stringEntries.put(path, new LogItemString(supplier, logEntry));
    }
    
    /**
     * Updates and logs all entries
     */
    public void update() {
        // Log all DoubleSuppliers
        for (Map.Entry<String, LogItemDouble> entry : doubleEntries.entrySet()) {
            LogItemDouble logItem = entry.getValue();
            logItem.logNewValue();
        }

        // Log all String suppliers
        for (Map.Entry<String, LogItemString> entry : stringEntries.entrySet()) {
            LogItemString logItem = entry.getValue();
            logItem.logNewValue();
        }
    }
}
