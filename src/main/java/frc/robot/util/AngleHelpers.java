package frc.robot.util;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public final class AngleHelpers {
    private final static double kMaxDegreesWhenNear = 8.0;
    private static final double maxDelta = 0.0001; // Allowable delta for floating point comparisons

    // Private constructor to prevent instantiation
    private AngleHelpers() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    // Returns the distance between two angles in degrees.
    private static double distanceBetweenAngles_Helper(Rotation2d angle1, Rotation2d angle2, double maxPositiveDegrees) {
        double angle1Degrees = angle1.getDegrees();
        double angle2Degrees = angle2.getDegrees();

        return Math.abs(MathUtil.inputModulus(
            angle1Degrees - angle2Degrees,
            -1.0 * maxPositiveDegrees,
            maxPositiveDegrees));
    }

    public static double distanceBetweenAngles720(Rotation2d angle1, Rotation2d angle2) {
        Rotation2d constrainedAngle1 = constrainNegative360ToPos360(angle1);
        Rotation2d constrainedAngle2 = constrainNegative360ToPos360(angle2);

        return distanceBetweenAngles_Helper(constrainedAngle1, constrainedAngle2, 360.0);
    }

    public static double distanceBetweenAngles(Rotation2d angle1, Rotation2d angle2) {
        Rotation2d constrainedAngle1 = constrainNegative180ToPos180(angle1);
        Rotation2d constrainedAngle2 = constrainNegative180ToPos180(angle2);

        return distanceBetweenAngles_Helper(constrainedAngle1, constrainedAngle2, 180.0);
    }

    public static double distanceBetweenAnglesIgnoringPolarity(Rotation2d angle1, Rotation2d angle2) {
        Rotation2d constrainedAngle1 = constrainNegative90ToPos90(angle1);
        Rotation2d constrainedAngle2 = constrainNegative90ToPos90(angle2);

        return distanceBetweenAngles_Helper(constrainedAngle1, constrainedAngle2, 90.0);
    }

    public static boolean isAngleNear(Rotation2d angle1, Rotation2d angle2) {
        double distance = distanceBetweenAngles(angle1, angle2);
        return distance <= kMaxDegreesWhenNear + maxDelta;
    }

    // A wheel can be rotated 180 degrees, and its still pointing in the same direction.  This
    // method ignores the wheel DIRECTION, and only checks the angle.
    public static boolean isAngleNearIgnoringPolarity(Rotation2d angle1, Rotation2d angle2) {
        double result = distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        return result <= kMaxDegreesWhenNear + maxDelta;
    }

    // Given angleA and a reference angle, return either angleA or angleA + 180 degrees,
    // whichever is closer to the reference angle.
    // allow720range is used in the case that the reference angle is in the -360 to 360 range.  Unfortunately,
    // some of the offsets are in this range, and we want to provide angles that are 'near' the existing offset
    // to make it easier to calibrate.
    public static Rotation2d getClosestAngleToReference(Rotation2d angleA, Rotation2d referenceAngle, boolean allow720range) {
        Rotation2d angleARotated;
        double distanceA;
        double distanceARotated;

        // Make sure the angles are in their proper ranges
        if (!allow720range) {
            angleA = constrainNegative90ToPos90(angleA);
            referenceAngle = constrainNegative180ToPos180(referenceAngle);
            angleARotated = constrainNegative180ToPos180(Rotation2d.fromDegrees(angleA.getDegrees() + 180.0));

            distanceA = distanceBetweenAngles(angleA, referenceAngle);
            distanceARotated = distanceBetweenAngles(angleARotated, referenceAngle);
        }
        else {
            angleA = constrainNegative180ToPos180(angleA);
            referenceAngle = constrainNegative360ToPos360(referenceAngle);
            angleARotated = constrainNegative360ToPos360(Rotation2d.fromDegrees(angleA.getDegrees() + 360.0));

            distanceA = distanceBetweenAngles720(angleA, referenceAngle);
            distanceARotated = distanceBetweenAngles720(angleARotated, referenceAngle);
        }

        if (distanceA <= distanceARotated) {
            return angleA;
        } else {
            return angleARotated;
        }
    }

    public static Rotation2d constrainNegative360ToPos360(Rotation2d angle) {
        double constrainedDegrees = MathUtil.inputModulus(
            angle.getDegrees(),
            -360.0, 
            360.0);
        return Rotation2d.fromDegrees(constrainedDegrees);
    }

    public static Rotation2d constrainNegative180ToPos180(Rotation2d angle) {
        double constrainedDegrees = MathUtil.inputModulus(
            angle.getDegrees(),
            -180.0, 
            180.0);
        return Rotation2d.fromDegrees(constrainedDegrees);
    }

    public static Rotation2d constrainNegative90ToPos90(Rotation2d angle) {
        double constrainedDegrees = MathUtil.inputModulus(
            angle.getDegrees(),
            -90.0, 
            90.0);
        return Rotation2d.fromDegrees(constrainedDegrees);
    }
}
