package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;

import edu.wpi.first.wpilibj.DriverStation;

class DetectHistory {
    public static final int CAPACITY = 5;
    public static final double LOOKBACK_SECONDS = 0.5;
    private Deque<DetectedValue> m_detectedValues = new ArrayDeque<>(CAPACITY);

    // Constructor
    public DetectHistory() {
        // Empty
    }

    private void addToQueueEnd(DetectedValue detectedValue) {
        if (m_detectedValues.size() >= CAPACITY) {
            m_detectedValues.removeFirst();
        }

        m_detectedValues.addLast(detectedValue);
        //System.out.println("Added vision item...");
    }

    // Returns a DetectedValue if one found, otherwise null
    private DetectedValue getItemWithSameTargetAndRobotPose(DetectedValue value) {
        // Note that we dont care if timestamp is different, we just check if it
        // describes the same target location
        for (DetectedValue detectedValueIter : m_detectedValues) {
            if (detectedValueIter.absoluteTargetPose.equals(value.absoluteTargetPose)
                    && detectedValueIter.robotPose.equals(value.robotPose)) {
                return detectedValueIter;
            }
        }

        return null;
    }

    public void add(DetectedValue detectedValue) {
        DetectedValue existingItem = getItemWithSameTargetAndRobotPose(detectedValue);
        if (existingItem != null) {
            // Remove the existing item
            m_detectedValues.remove(existingItem);
        }

        // Add the new item to end of queue
        addToQueueEnd(detectedValue);
    }

    public int getSize() {
        return m_detectedValues.size();
    }

    private boolean isRecent(DetectedValue value) {
        double timeDiff = DriverStation.getMatchTime() - value.timeStamp;

        return timeDiff >= 0 && timeDiff <= LOOKBACK_SECONDS;
    }

    // Returns null if none found
    public DetectedValue getBestValue() {
        if (m_detectedValues.size() == 0) {
            return null;
        }

        // Print the queue, with index and ta value.  Use whitespace to align text
        // int index = 0;
        // for (DetectedValue iter : m_detectedValues) {
        //     if (isRecent(iter)) {
        //         System.out.printf("Item %d: ta = %.2f\n", index, iter.ta);
        //     }
        //     index++;
        // }

        // Return the item from the queue with the highest ta
        DetectedValue bestValue = null;
        int bestIndex = 0;
        int index = 0;
        for (DetectedValue detectedValueIter : m_detectedValues) {
            // We only allow values that are within ~0.5 a second
            if (isRecent(detectedValueIter)) {

                // All things being equal, we want the most recent item, so use <=
                if (bestValue == null || detectedValueIter.ta >= bestValue.ta) {
                    bestValue = detectedValueIter;
                    bestIndex = index;
                }
            }

            index++;
        }

        // if (bestValue != null) {
        //     System.out.println("Picked item: " + bestIndex);
        // }

        return bestValue;
    }
}
