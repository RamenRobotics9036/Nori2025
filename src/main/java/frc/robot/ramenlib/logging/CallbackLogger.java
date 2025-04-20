package frc.robot.ramenlib.logging;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * The CallbackLogger class provides functionality for logging numeric and string
 * values using suppliers. It supports logging changes to a data log.
 */
public class CallbackLogger {
    // Inner class for string log items
    private class LogItemString {
        private final Supplier<String> m_stringSupplier;
        private final StringLogEntry m_logEntryString;

        public LogItemString(Supplier<String> stringSupplier, StringLogEntry logEntryString) {
            this.m_stringSupplier = stringSupplier;
            this.m_logEntryString = logEntryString;
        }

        public void logNewValue() {
            // Note that we only log CHANGES
            String newValue = m_stringSupplier.get();
            m_logEntryString.update(newValue);
        }
    }

    // Inner class for double log items
    private class LogItemDouble {
        private final DoubleSupplier m_doubleSupplier;
        private final DoubleLogEntry m_logEntryDouble;

        public LogItemDouble(DoubleSupplier doubleSupplier, DoubleLogEntry logEntryDouble) {
            this.m_doubleSupplier = doubleSupplier;
            this.m_logEntryDouble = logEntryDouble;
        }

        public void logNewValue() {
            // Note that we only log CHANGES
            double newValue = m_doubleSupplier.getAsDouble();
            m_logEntryDouble.update(newValue);
        }
    }

    // Members
    private final DataLog m_dataLog;
    private final Map<String, LogItemDouble> m_doubleEntries = new HashMap<>();
    private final Map<String, LogItemString> m_stringEntries = new HashMap<>();

    /**
     * Constructor.
     */
    public CallbackLogger() {
        m_dataLog = DataLogManager.getLog();
    }

    /**
     * Adds a new numeric logging entry.
     */
    public void add(String path, DoubleSupplier supplier) {
        DoubleLogEntry logEntry = new DoubleLogEntry(m_dataLog, path);
        m_doubleEntries.put(path, new LogItemDouble(supplier, logEntry));
    }

    /**
     * Adds a new string logging entry.
     */
    public void add(String path, Supplier<String> supplier) {
        StringLogEntry logEntry = new StringLogEntry(m_dataLog, path);
        m_stringEntries.put(path, new LogItemString(supplier, logEntry));
    }

    /**
     * Updates and logs all entries.
     */
    public void update() {
        // Log all DoubleSuppliers
        for (Map.Entry<String, LogItemDouble> entry : m_doubleEntries.entrySet()) {
            LogItemDouble logItem = entry.getValue();
            logItem.logNewValue();
        }

        // Log all String suppliers
        for (Map.Entry<String, LogItemString> entry : m_stringEntries.entrySet()) {
            LogItemString logItem = entry.getValue();
            logItem.logNewValue();
        }
    }
}
