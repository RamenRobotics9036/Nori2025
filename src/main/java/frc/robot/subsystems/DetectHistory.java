package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

class DetectHistory {
    private int m_capacity;
    private double m_lookbackSec;
    private Deque<DetectedValue> m_detectedValues;

    // Constructor
    public DetectHistory(int capacity, double lookback_sec) {
        m_capacity = capacity;
        m_lookbackSec = lookback_sec;
        m_detectedValues = new ArrayDeque<>(m_capacity);
    }

    public DetectHistory() {
        this(VisionConstants.kHistoryLength, VisionConstants.kHistoryLookbackSec);
    }

    private void addToQueueEnd(DetectedValue detectedValue) {
        if (m_detectedValues.size() >= m_capacity) {
            m_detectedValues.removeFirst();
        }

        m_detectedValues.addLast(detectedValue);
    }

    // Returns a DetectedValue if one found, otherwise null
    private DetectedValue getFirstItemWithSameTargetAndRobotPose(DetectedValue value) {
        // Note that we dont care if timestamp is different, we just check if it
        // describes the same target location
        for (DetectedValue detectedValueIter : m_detectedValues) {
            if (detectedValueIter.getAbsoluteTargetPose().equals(value.getAbsoluteTargetPose())
                    && detectedValueIter.getRobotPose().equals(value.getRobotPose())) {
                return detectedValueIter;
            }
        }

        return null;
    }

    public void add(DetectedValue detectedValue) {
        DetectedValue existingItem = getFirstItemWithSameTargetAndRobotPose(detectedValue);
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

    public List<DetectedValue> getAsList() {
        return List.copyOf(m_detectedValues);
    }

    private boolean isRecent(DetectedValue value, double currentTime) {
        double timeDiff = currentTime - value.getTimeStamp();

        return timeDiff >= 0 && timeDiff <= m_lookbackSec;
    }

    // Returns null if none found
    // For testing, ability to override 'now'
    public DetectedValue getBestValue(double currentTime) {
        if (m_detectedValues.size() == 0) {
            return null;
        }

        // Return the item from the queue with the highest ta
        DetectedValue bestValue = null;
        int bestIndex = 0;
        int index = 0;
        for (DetectedValue detectedValueIter : m_detectedValues) {
            // We only allow values that are within ~0.5 a second
            if (isRecent(detectedValueIter, currentTime)) {

                // All things being equal, we want the most recent item, so use <=
                if (bestValue == null || detectedValueIter.getTa() >= bestValue.getTa()) {
                    bestValue = detectedValueIter;
                    bestIndex = index;
                }
            }

            index++;
        }

        if (bestValue != null) {
            //System.out.printf("Picked item: %d (time: %.2f)%n", bestIndex, bestValue.getTimeStamp());
        }

        return bestValue;
    }

    public DetectedValue getBestValue() {
        return getBestValue(Timer.getFPGATimestamp());
    }
}
