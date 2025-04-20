package frc.robot.ramenlib.wheelcalibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A utility class that provides helper methods for angle calculations,
 * including distance between angles, normalization, and polarity checks.
 * Note that a wheel can be rotated 180 degrees, and it's still pointing in the same direction.
 * We refer to a calculation as 'ignoring polarity' when we ignore the ABSOLUTE direction
 * of the wheel.
 */
public final class AngleHelpers {
    private static final double maxDelta = 0.0001;
    private static final double kMaxDegreesWhenNear = 8.0;

    // Private constructor to prevent instantiation
    private AngleHelpers() {
        throw new UnsupportedOperationException(
            "This is a utility class and cannot be instantiated");
    }

    /**
     * Calculates the distance between two angles in degrees.
     *
     * @param angle1 The first angle to compare.
     * @param angle2 The second angle to compare.
     * @return The distance between the two angles in degrees.
     */
    public static double distanceBetweenAngles(Rotation2d angle1, Rotation2d angle2) {
        Rotation2d constrainedAngle1 = normalizeAngle(angle1, -180, 180);
        Rotation2d constrainedAngle2 = normalizeAngle(angle2, -180, 180);

        return distanceBetweenAnglesHelper(constrainedAngle1, constrainedAngle2, 180.0);
    }

    /**
     * Calculates the distance between two angles, ignoring their polarity.  (i.e. a wheel
     * can be rotated 180 degrees, and its still pointing in the same direction).
     *
     * @param angle1 The first angle to compare.
     * @param angle2 The second angle to compare.
     * @return The distance between the two angles, ignoring polarity.
     */
    public static double distanceBetweenAnglesIgnoringPolarity(
        Rotation2d angle1,
        Rotation2d angle2) {

        Rotation2d constrainedAngle1 = normalizeAngle(angle1, -90, 90);
        Rotation2d constrainedAngle2 = normalizeAngle(angle2, -90, 90);

        return distanceBetweenAnglesHelper(constrainedAngle1, constrainedAngle2, 90.0);
    }

    /**
     * Checks if two angles are near each other within a specified threshold.
     *
     * @param angle1 The first angle to compare.
     * @param angle2 The second angle to compare.
     * @return True if the angles are near each other; otherwise, false.
     */
    public static boolean isAngleNear(Rotation2d angle1, Rotation2d angle2) {
        double distance = distanceBetweenAngles(angle1, angle2);
        return distance <= kMaxDegreesWhenNear + maxDelta;
    }

    /**
     * Checks if two angles are near each other, ignoring polarity.
     * A wheel can be rotated 180 degrees, and its still pointing in the same direction. This
     * method ignores the wheel DIRECTION, and only checks the angle.
     *
     * @param angle1 The first angle to compare.
     * @param angle2 The second angle to compare.
     * @return True if the angles are near each other, ignoring polarity; otherwise, false.
     */
    public static boolean isAngleNearIgnoringPolarity(Rotation2d angle1, Rotation2d angle2) {
        double result = distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        return result <= kMaxDegreesWhenNear + maxDelta;
    }

    // Given angleA and a reference angle, return either angleA or angleA + 180 degrees,
    // whichever is closer to the reference angle.
    /**
     * Returns the angle (either the given angle or the given angle plus 180 degrees)
     * that is closest to the reference angle.
     *
     * @param angle The angle to compare.
     * @param referenceAngle The reference angle to compare against.
     * @return The closest angle to the reference angle.
     */
    public static Rotation2d getClosestAngleToReference(
        Rotation2d angle,
        Rotation2d referenceAngle) {

        // Make sure the angles are in their proper ranges
        angle = normalizeAngle(angle, -90, 90);
        referenceAngle = normalizeAngle(referenceAngle, -180, 180);
        Rotation2d angleRotated = normalizeAngle(
            Rotation2d.fromDegrees(angle.getDegrees() + 180.0),
            -180,
            180);

        double distance = distanceBetweenAngles(angle, referenceAngle);
        double distanceRotated = distanceBetweenAngles(angleRotated, referenceAngle);

        if (distance <= distanceRotated) {
            return angle;
        }
        else {
            return angleRotated;
        }
    }

    /**
     * Normalizes the given angle to be within the specified bounds.
     *
     * @param angle The angle to normalize.
     * @param lowerBound The lower bound of the range.
     * @param upperBound The upper bound of the range.
     * @return A Rotation2d object representing the normalized angle.
     */
    public static Rotation2d normalizeAngle(
        Rotation2d angle,
        double lowerBound,
        double upperBound) {

        double normalizedDegrees = MathUtil.inputModulus(
            angle.getDegrees(),
            lowerBound,
            upperBound);
        return Rotation2d.fromDegrees(normalizedDegrees);
    }

    // Returns the distance between two angles in degrees.
    private static double distanceBetweenAnglesHelper(
        Rotation2d angle1,
        Rotation2d angle2,
        double maxPositiveDegrees) {
        double angle1Degrees = angle1.getDegrees();
        double angle2Degrees = angle2.getDegrees();

        return Math.abs(
            MathUtil.inputModulus(
                angle1Degrees - angle2Degrees,
                -1.0 * maxPositiveDegrees,
                maxPositiveDegrees));
    }
}
