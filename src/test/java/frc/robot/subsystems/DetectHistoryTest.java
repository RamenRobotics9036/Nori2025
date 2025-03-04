package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.VisionHelpers.DetectHistory;
import frc.robot.subsystems.VisionHelpers.DetectedValue;

import static org.junit.jupiter.api.Assertions.*;

class DetectHistoryTest {
    private DetectHistory m_detectHistory;
    private static final int kTestHistoryLen = 5;
    private static final double kTestLookbackSec = 0.5;
    private double m_currentTime = 0.0;

    @BeforeEach
    void setup() {
        m_currentTime = 0.0;
        m_detectHistory = new DetectHistory(kTestHistoryLen, kTestLookbackSec);
    }

    // We use a fake clock, since using the real clock would make the tests
    // non-deterministic.
    private double getTestCurrentTime() {
        double result = m_currentTime;

        // Add a hundredth of a second
        m_currentTime += 0.01;

        return result;
    }

    //
    // Test DetectedValue
    //

    @Test
    void testNullTargetPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(null, createSamplePose2d(), 0.5, getTestCurrentTime());
        });
    }

    @Test
    void testNullRobotPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(createSamplePose2d(), null, 0.5, getTestCurrentTime());
        });
    }

    @Test
    void testEmptyTargetPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(new Pose2d(), createSamplePose2d(), 0.5, getTestCurrentTime());
        });
    }

    @Test
    void testEmptyRobotPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(createSamplePose2d(), new Pose2d(), 0.5, getTestCurrentTime());
        });
    }

    @Test
    void testTwoItemsShouldBeEqual() {
        DetectedValue valueA = createDetectedValueA();
        DetectedValue valueB = createDetectedValueA();

        assertEquals(valueA, valueB, "Expected values to be equal");
    }

    @Test
    void testTwoItemsDiffTargetPoseShouldNotBeEqual() {
        DetectedValue valueA = createDetectedValue(1, 1, 1);
        DetectedValue valueB = createDetectedValue(2, 1, 1);

        assertNotEquals(valueA, valueB, "Expected values to be different");
    }

    @Test
    void testTwoItemsDiffRobotPoseShouldNotBeEqual() {
        DetectedValue valueA = createDetectedValue(1, 1, 1);
        DetectedValue valueB = createDetectedValue(1, 2, 1);

        assertNotEquals(valueA, valueB, "Expected values to be different");
    }

    @Test
    void testTwoItemsDiffTaShouldNotBeEqual() {
        DetectedValue valueA = createDetectedValue(1, 1, 1);
        DetectedValue valueB = createDetectedValue(1, 1, 2);

        assertNotEquals(valueA, valueB, "Expected values to be different");
    }

    @Test
    void testTwoItemsWithDifferentTimeStampShouldBeEqual() {
        DetectedValue valueA = createDetectedValueA();
        DetectedValue valueB = createDetectedValueA();

        assertNotEquals(valueA.getTimeStamp(), valueB.getTimeStamp(), "Expected timestamps to be different");
        assertEquals(valueA, valueB, "Expected values to be equal");
    }

    //
    // Test DetectHistory
    //

    @Test
    void tesInitialSizeZero() {
        int num_items = m_detectHistory.getSize();
        assertEquals(0, num_items, "Expected 0 items in list initially.");
    }

    @Test
    void testGetBestEmpty() {
        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertNull(best, "Expected null when history is empty");
    }

    private Pose2d createSamplePose2d() {
        return new Pose2d(1.0, 2.0, new Rotation2d(Math.toRadians(90)));
    }

    private DetectedValue createDetectedValue(
        int multiplierTargetPose,
        int multiplierRobotPose,
        int multiplierTa,
        double currentTime) {

        return new DetectedValue(
            new Pose2d(1.0, 2.0 * (double)multiplierTargetPose, new Rotation2d(Math.toRadians(90))),
            new Pose2d(3.0 * (double)multiplierRobotPose, 4.0, new Rotation2d(Math.toRadians(180))),
            0.5 * (double)multiplierTa,
            currentTime);
    }

    private DetectedValue createDetectedValue(
        int multiplierTargetPose,
        int multiplierRobotPose,
        int multiplierTa) {
    
        return createDetectedValue(multiplierTargetPose, multiplierRobotPose, multiplierTa, getTestCurrentTime());
    }

    private DetectedValue createDetectedValueA() {
        return createDetectedValue(2, 3, 4);
    }

    private DetectedValue createDetectedValueB() {
        return createDetectedValue(5, 6, 7);
    }

    private DetectedValue createDetectedValueC() {
        return createDetectedValue(8, 9, 10);
    }

    @Test
    void testGetBestSingleValueShouldSucceed() {
        DetectedValue value = createDetectedValueA();
        m_detectHistory.add(value);

        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(createDetectedValueA(), best, "Expected best value to be the only value in history");
    }

    @Test
    void testGetBestValueShouldNotRemoveThatValue() {
        DetectedValue value = createDetectedValueA();
        m_detectHistory.add(value);

        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(createDetectedValueA(), best, "Expected best value to be the only value in history");
        assertEquals(1, m_detectHistory.getSize(), "Expected size to be 1 after getBestValue()");
    }

    @Test
    void testReturnLastItemIfItHasHighestTa() {
        DetectedValue valueA = createDetectedValue(1, 2, 1);
        DetectedValue valueB = createDetectedValue(2, 3, 2);
        DetectedValue valueC = createDetectedValue(4, 5, 3);

        m_detectHistory.add(valueA);
        m_detectHistory.add(valueB);
        m_detectHistory.add(valueC);

        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(valueC, best, "Expected best value to be the last value in history");
    }

    @Test
    void testReturnMiddleItemIfItHasHighestTa() {
        DetectedValue valueA = createDetectedValue(1, 2, 1);
        DetectedValue valueB = createDetectedValue(2, 3, 3);
        DetectedValue valueC = createDetectedValue(4, 5, 2);

        m_detectHistory.add(valueA);
        m_detectHistory.add(valueB);
        m_detectHistory.add(valueC);

        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(valueB, best, "Expected best value to be the last value in history");
    }

    @Test
    void testReturnFirstItemIfItHasHighestTa() {
        DetectedValue valueA = createDetectedValue(1, 2, 3);
        DetectedValue valueB = createDetectedValue(2, 3, 1);
        DetectedValue valueC = createDetectedValue(4, 5, 2);

        m_detectHistory.add(valueA);
        m_detectHistory.add(valueB);
        m_detectHistory.add(valueC);

        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(valueA, best, "Expected best value to be the last value in history");
    }

    @Test
    void testGetSingleStaleItemShouldReturnNull() {
        double itemTime = 0.0;
        double queryTime = 1.0;
        DetectedValue value = createDetectedValue(1, 1, 1, itemTime);
        m_detectHistory.add(value);

        DetectedValue best = m_detectHistory.getBestValue(queryTime);
        assertNull(best, "Expected null when history is stale");
    }

    @Test
    void testGetStaleFirstItemShouldReturnSecond() {
        // This first item has a BIGGER area (ta)
        DetectedValue valueA = createDetectedValue(1, 2, 2, 0.5);
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValue(2, 3, 1, 1.0);
        m_detectHistory.add(valueB);

        DetectedValue best = m_detectHistory.getBestValue(1.1);
        assertEquals(valueB, best, "Expected best value to be the second value in history");
    }

    @Test
    void testItemAtExactStaleTimeShouldBeReturned() {
        // This first item has a BIGGER area (ta)
        DetectedValue valueA = createDetectedValue(1, 2, 2, 0.5);
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValue(2, 3, 1, 1.0);
        m_detectHistory.add(valueB);

        DetectedValue best = m_detectHistory.getBestValue(1.0);
        assertEquals(valueA, best, "Expected best value to be the first value in history");
    }

    @Test
    void testMultipleItemsWithSameTaShouldReturnLast() {
        DetectedValue valueA = createDetectedValue(1, 2, 5);
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValue(3, 4, 5);
        m_detectHistory.add(valueB);

        DetectedValue valuC = createDetectedValue(5, 6, 4);
        m_detectHistory.add(valuC);

        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(valueB, best, "Expected best value to be the second value in history");
    }

    @Test
    void testAddingBeyondCapacityEvictsOldest() {
        DetectHistory customDetectHistory = new DetectHistory(2, 0.5);
 
        DetectedValue valueA = createDetectedValueA();
        DetectedValue valueB = createDetectedValueB();
        DetectedValue valueC = createDetectedValueC();

        customDetectHistory.add(valueA);
        customDetectHistory.add(valueB);
        customDetectHistory.add(valueC);

        assertEquals(2, customDetectHistory.getSize(), "Expected size to be 2 after adding 3 items");
        assertEquals(valueC, customDetectHistory.getBestValue(getTestCurrentTime()), "Expected best value to be the last value in history");
    }

    @Test
    void testAddingItemAlreadyInListRemovesItAndAddsItWithNewTimestamp() {
        DetectedValue valueA = createDetectedValue(1, 2, 5);
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValue(3, 4, 7);
        m_detectHistory.add(valueB);

        DetectedValue valueC = createDetectedValue(5, 6, 5);
        m_detectHistory.add(valueC);

        // At this point, B should be the best as it has highest ta
        DetectedValue best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(3, m_detectHistory.getSize(), "Expected size to be 3");
        assertEquals(valueB, best, "Expected best value to be valueB");

        // Now add a new item with same ta as B
        DetectedValue valueD = createDetectedValue(3, 4, 7);
        m_detectHistory.add(valueD);

        // At this point, D should be the best as it has highest ta
        best = m_detectHistory.getBestValue(getTestCurrentTime());
        assertEquals(3, m_detectHistory.getSize(), "Expected size to be 3");
        assertEquals(valueD, best, "Expected best value to be valueD");

        // Get it as a list, and double-check
        List<DetectedValue> result = m_detectHistory.getAsList();
        assertEquals(3, result.size(), "Expected size to be 3");
        assertEquals(valueA, result.get(0), "Got unexpected item");
        assertEquals(valueC, result.get(1), "Got unexpected item");
        assertEquals(valueD, result.get(2), "Got unexpected item");
    }
}
