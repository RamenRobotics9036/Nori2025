package frc.robot.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CallbackLogger {
    private final Map<String, DoubleSupplier> doubleEntries = new HashMap<>();
    private final Map<String, Supplier<String>> stringEntries = new HashMap<>();

    /**
     * Adds a new numeric logging entry
     */
    public void add(String path, DoubleSupplier supplier) {
        doubleEntries.put(path, supplier);
    }
    
    /**
     * Adds a new string logging entry
     */
    public void add(String path, Supplier<String> supplier) {
        stringEntries.put(path, supplier);
    }
    
    /**
     * Updates and logs all entries
     */
    public void update() {
        // Log all DoubleSuppliers
        for (Map.Entry<String, DoubleSupplier> entry : doubleEntries.entrySet()) {
            String path = entry.getKey();
            double value = entry.getValue().getAsDouble();
            System.out.println(path + ": " + value);
        }

        // Log all String suppliers
        for (Map.Entry<String, Supplier<String>> entry : stringEntries.entrySet()) {
            String path = entry.getKey();
            String value = entry.getValue().get();
            System.out.println(path + ": " + value);
        }
    }
}
