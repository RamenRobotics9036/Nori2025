package frc.robot.subsystems;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static org.junit.jupiter.api.Assertions.*;

class DetectHistoryTest {
    private DetectHistory m_detectHistory;
    private static final int kTestHistoryLen = 5;
    private static final double kTestLookbackSec = 0.5;

    @BeforeEach
    void setup() {
        m_detectHistory = new DetectHistory(kTestHistoryLen, kTestLookbackSec);
    }

    //
    // Test DetectedValue
    //

    @Test
    void testNullTargetPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(null, createSamplePose2d(), 0.5);
        });
    }

    @Test
    void testNullRobotPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(createSamplePose2d(), null, 0.5);
        });
    }

    @Test
    void testEmptyTargetPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(new Pose2d(), createSamplePose2d(), 0.5);
        });
    }

    @Test
    void testEmptyRobotPoseShouldThrowException() {
        assertThrows(IllegalArgumentException.class, () -> {
            new DetectedValue(createSamplePose2d(), new Pose2d(), 0.5);
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
        valueB.timeStamp = 10.0;

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
        DetectedValue best = m_detectHistory.getBestValue();
        assertNull(best, "Expected null when history is empty");
    }

    private Pose2d createSamplePose2d() {
        return new Pose2d(1.0, 2.0, new Rotation2d(Math.toRadians(90)));
    }

    private DetectedValue createDetectedValue(
        int multiplierTargetPose,
        int multiplierRobotPose,
        int multiplierTa) {

        return new DetectedValue(
            new Pose2d(1.0, 2.0 * (double)multiplierTargetPose, new Rotation2d(Math.toRadians(90))),
            new Pose2d(3.0 * (double)multiplierRobotPose, 4.0, new Rotation2d(Math.toRadians(180))),
            0.5 * (double)multiplierTa);
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

        DetectedValue best = m_detectHistory.getBestValue();
        assertEquals(createDetectedValueA(), best, "Expected best value to be the only value in history");
    }

    @Test
    void testGetBestValueShouldNotRemoveThatValue() {
        DetectedValue value = createDetectedValueA();
        m_detectHistory.add(value);

        DetectedValue best = m_detectHistory.getBestValue();
        assertEquals(createDetectedValueA(), best, "Expected best value to be the only value in history");
        assertEquals(1, m_detectHistory.getSize(), "Expected size to be 1 after getBestValue()");
    }

    @Test
    void testReturnLastItemIfItHasHighestTa() {
        DetectedValue valueA = createDetectedValueA();
        DetectedValue valueB = createDetectedValueB();
        DetectedValue valueC = createDetectedValueC();

        valueA.ta = 0.5;
        valueB.ta = 0.6;
        valueC.ta = 0.7;

        m_detectHistory.add(valueA);
        m_detectHistory.add(valueB);
        m_detectHistory.add(valueC);

        DetectedValue best = m_detectHistory.getBestValue();
        assertEquals(valueC, best, "Expected best value to be the last value in history");
    }

    @Test
    void testReturnMiddleItemIfItHasHighestTa() {
        DetectedValue valueA = createDetectedValueA();
        DetectedValue valueB = createDetectedValueB();
        DetectedValue valueC = createDetectedValueC();

        valueA.ta = 0.5;
        valueB.ta = 0.7;
        valueC.ta = 0.6;

        m_detectHistory.add(valueA);
        m_detectHistory.add(valueB);
        m_detectHistory.add(valueC);

        DetectedValue best = m_detectHistory.getBestValue();
        assertEquals(valueB, best, "Expected best value to be the last value in history");
    }

    @Test
    void testReturnFirstItemIfItHasHighestTa() {
        DetectedValue valueA = createDetectedValueA();
        DetectedValue valueB = createDetectedValueB();
        DetectedValue valueC = createDetectedValueC();

        valueA.ta = 0.7;
        valueB.ta = 0.6;
        valueC.ta = 0.5;

        m_detectHistory.add(valueA);
        m_detectHistory.add(valueB);
        m_detectHistory.add(valueC);

        DetectedValue best = m_detectHistory.getBestValue();
        assertEquals(valueA, best, "Expected best value to be the last value in history");
    }

    @Test
    void testGetSingleStaleItemShouldReturnNull() {
        DetectedValue value = createDetectedValueA();
        value.timeStamp = -1.0;
        m_detectHistory.add(value);

        DetectedValue best = m_detectHistory.getBestValue();
        assertNull(best, "Expected null when history is stale");
    }

    @Test
    void testGetStaleFirstItemShouldReturnSecond() {
        DetectedValue valueA = createDetectedValueA();
        valueA.timeStamp = 0.5;
        valueA.ta = 1.0;
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValueB();
        valueB.timeStamp = 1.0;
        valueB.ta = 0.5;
        m_detectHistory.add(valueB);

        DetectedValue best = m_detectHistory.getBestValue(1.1);
        assertEquals(valueB, best, "Expected best value to be the second value in history");
    }

    @Test
    void testItemAtExactStaleTimeShouldBeReturned() {
        DetectedValue valueA = createDetectedValueA();
        valueA.timeStamp = 0.5;
        valueA.ta = 1.0; // This area is bigger than valueB
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValueB();
        valueB.timeStamp = 0.7;
        valueB.ta = 0.5;
        m_detectHistory.add(valueB);

        DetectedValue best = m_detectHistory.getBestValue(1.0);
        assertEquals(valueA, best, "Expected best value to be the second value in history");
    }

    @Test
    void testMultipleItemsWithSameTaShouldReturnLast() {
        DetectedValue valueA = createDetectedValueA();
        valueA.ta = 1.0;
        m_detectHistory.add(valueA);

        DetectedValue valueB = createDetectedValueB();
        valueB.ta = 1.0;
        m_detectHistory.add(valueB);

        DetectedValue valuC = createDetectedValueC();
        valuC.ta = 0.5;
        m_detectHistory.add(valuC);

        DetectedValue best = m_detectHistory.getBestValue();
        assertEquals(valueB, best, "Expected best value to be the second value in history");
    }

    @Test
    void testAddingBeyondCapacityEvictsOldest() {
        // $TODO: Also make sure size remains 5
        assertTrue(true);
    }

    @Test
    void testAddingItemAlreadyInListRemovesItAndAddsItWithNewTimestamp() {
        assertTrue(true);
    }
}
