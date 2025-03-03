package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;

class DetectHistory {
    private static final int CAPACITY = 5;
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

    // Returns null if none found
    public DetectedValue getBestValue() {
        if (m_detectedValues.size() == 0) {
            return null;
        }

        // Print the queue, with index and ta value.  Use whitespace to align text
        int index = 0;
        for (DetectedValue iter : m_detectedValues) {
            System.out.printf("Item %d: ta = %.2f\n", index, iter.ta);
            index++;
        }

        // Return the item from the queue with the highest ta
        DetectedValue bestValue = null;
        int bestIndex = 0;
        index = 0;
        for (DetectedValue detectedValueIter : m_detectedValues) {
            // All things being equal, we want the most recent item, so use <=
            if (bestValue == null || detectedValueIter.ta >= bestValue.ta) {
                bestValue = detectedValueIter;
                bestIndex = index;
            }

            index++;
        }

        if (bestValue != null) {
            System.out.println("Picked item: " + bestIndex);
        }

        return bestValue;
    }
}
