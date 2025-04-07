package frc.robot.util.ramenutils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.ramenutils.wheelcalibration.AngleHelpers;

public class AngleHelpersTest {
    private static final double maxDelta = 0.0001; // Allowable delta for floating point comparisons

    //
    // Test constrainNegative180ToPos180 method
    //

    @Test
    public void testConstrainWorks() {
        double angle = 5.0;

        double expected = angle;
        Rotation2d actual = AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(angle), -180, 180);
        assertEquals(expected, actual.getDegrees(), "The angle should be unchanged.");
    }

    @Test
    public void testContrain180ShouldReturn180() {
        double angle = 180.0;

        double expected = 180.0;
        Rotation2d actual = AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(angle), -180, 180);
        assertEquals(expected, actual.getDegrees(), "The angle should be unchanged.");
    }

    // NOTE: MathUtil.inputModulus does convert -180 to 180.  We can live with that.
    @Test
    public void testConstrainNegative180ShouldReturnNegative180() {
        double angle = -180.0;

        double expected = 180.0;
        Rotation2d actual = AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(angle), -180, 180);
        assertEquals(expected, actual.getDegrees(), "Expected 180 given implementation of MathUtil.inputModulus.");
    }

    @Test
    public void testConstrain181ShouldWork() {
        double angle = 181.0;

        double expected = -179.0;
        Rotation2d actual = AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(angle), -180, 180);
        assertEquals(expected, actual.getDegrees(), "The angle should wrap around.");
    }

    @Test
    public void testConstrainNegative181ShouldWork() {
        double angle = -181.0;

        double expected = 179.0;
        Rotation2d actual = AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(angle), -180, 180);
        assertEquals(expected, actual.getDegrees(), "The angle should wrap around.");
    }

    @Test
    public void testBigNegativeNumberShouldWork() {
        double angle = -1000.0;

        double expected = 80.0; // -1000 mod 360 = -280, which is 80 degrees in the positive direction.
        Rotation2d actual = AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(angle), -180, 180);
        assertEquals(expected, actual.getDegrees(), maxDelta, "The angle should wrap around correctly.");
    }

    //
    // Test distanceBetweenAngles method
    //
    
    @Test
    public void testDistanceBetweenAnglesBasicCase() {
        Rotation2d angle1 = Rotation2d.fromDegrees(30.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(45.0);
        
        double expected = 15.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be 15 degrees.");
    }

    @Test
    public void testDistanceBetweenAnglesSameAngle() {
        Rotation2d angle1 = Rotation2d.fromDegrees(90.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(90.0);
        
        double expected = 0.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should be 0 degrees for identical angles.");
    }

    @Test
    public void testDistanceBetweenAngles180DegreesApart() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(180.0);
        
        double expected = 180.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should be 180 degrees for opposite angles.");
    }

    @Test
    public void testDistanceBetweenAnglesWrappingAround() {
        Rotation2d angle1 = Rotation2d.fromDegrees(170.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-170.0);
        
        double expected = 20.0; // The shortest path is 20 degrees, not 340 degrees
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should account for wrapping around the circle.");
    }

    @Test
    public void testDistanceBetweenAnglesPositiveAndNegative() {
        Rotation2d angle1 = Rotation2d.fromDegrees(45.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-45.0);
        
        double expected = 90.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should be 90 degrees.");
    }

    @Test
    public void testDistanceBetweenAnglesBothPositive() {
        Rotation2d angle1 = Rotation2d.fromDegrees(30.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(120.0);
        
        double expected = 90.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be within the allowed delta.");
    }

    @Test
    public void testDistanceBetweenAnglesBothNegative() {
        Rotation2d angle1 = Rotation2d.fromDegrees(-30.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-120.0);
        
        double expected = 90.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be within the allowed delta.");
    }

    @Test
    public void testDistanceBetweenAnglesBoundaryValues() {
        Rotation2d angle1 = Rotation2d.fromDegrees(-180.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(180.0);
        
        double expected = 0.0; // These angles represent the same position
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should be 0 for -180 and 180 degrees.");
    }

    @Test
    public void testDistanceBetweenAnglesExtremeValues() {
        Rotation2d angle1 = Rotation2d.fromDegrees(720.0); // Equivalent to 0 degrees
        Rotation2d angle2 = Rotation2d.fromDegrees(-540.0); // Equivalent to 180 degrees
        
        double expected = 180.0;
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should handle extreme values correctly.");
    }

    @Test
    public void testDistanceBetweenAnglesNearBoundary() {
        Rotation2d angle1 = Rotation2d.fromDegrees(179.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-179.0);
        
        double expected = 2.0; // The shortest path is 2 degrees around the boundary
        double actual = AngleHelpers.distanceBetweenAngles(angle1, angle2);
        assertEquals(expected, actual, "The distance should find the shortest path around the circle.");
    }

    //
    // Test isAngleNear
    //
    
    @Test
    public void testIsAngleNearExactlyAtThreshold() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(8.0); // kMaxDegreesWhenNear is 8.0
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles exactly at threshold should be considered near.");
    }
    
    @Test
    public void testIsAngleNearJustOverThreshold() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(8.1); // Just over kMaxDegreesWhenNear
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles just over threshold should not be considered near.");
    }
    
    @Test
    public void testIsAngleNearJustUnderThreshold() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(7.9); // Just under kMaxDegreesWhenNear
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles just under threshold should be considered near.");
    }
    
    @Test
    public void testIsAngleNearIdenticalAngles() {
        Rotation2d angle1 = Rotation2d.fromDegrees(45.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(45.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Identical angles should be considered near.");
    }
    
    @Test
    public void testIsAngleNearFarApart() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(90.0);
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles far apart should not be considered near.");
    }
    
    @Test
    public void testIsAngleNearCrossingBoundary() {
        Rotation2d angle1 = Rotation2d.fromDegrees(178.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-178.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles near but crossing the +/-180 boundary should be considered near.");
    }
    
    @Test
    public void testIsAngleNearLargeValues() {
        Rotation2d angle1 = Rotation2d.fromDegrees(365.0); // Equivalent to 5 degrees
        Rotation2d angle2 = Rotation2d.fromDegrees(10.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Large angle values that are near when normalized should be considered near.");
    }
    
    @Test
    public void testIsAngleNearNegativeValues() {
        Rotation2d angle1 = Rotation2d.fromDegrees(-50.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-55.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Negative angles that are near should be considered near.");
    }
    
    @Test
    public void testIsAngleNear360DegreesApart() {
        Rotation2d angle1 = Rotation2d.fromDegrees(720.0); // Equivalent to 0 degrees
        Rotation2d angle2 = Rotation2d.fromDegrees(0.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles 360 degrees apart should be considered near as they're the same position.");
    }
    
    @Test
    public void testIsAngleNearOppositeDirections() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(180.0);
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNear(angle1, angle2);
        assertEquals(expected, actual, "Angles in opposite directions should not be considered near.");
    }

    //
    // Test distanceBetweenAnglesIgnoringPolarity
    //
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolaritySameAngle() {
        Rotation2d angle1 = Rotation2d.fromDegrees(45.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(45.0);
        
        double expected = 0.0;
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be 0 for identical angles.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarity180DegreesApart() {
        Rotation2d angle1 = Rotation2d.fromDegrees(45.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(225.0); // 45 + 180
        
        double expected = 0.0;
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be 0 for angles flipped 180 degrees.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarity90DegreesApart() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(90.0);
        
        double expected = 90.0;
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be 90 degrees for perpendicular angles.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarityMoreThan90DegreesApart() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(135.0);
        
        double expected = 45.0; // When ignoring polarity, 135° becomes equivalent to -45°
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should account for wheel flipping when >90 degrees apart.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarityWrappingAround() {
        Rotation2d angle1 = Rotation2d.fromDegrees(170.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-170.0);
        
        double expected = 20.0; // When considering flipping, these are 20 degrees apart
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should correctly handle angles that wrap around.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarityAtBoundaries() {
        Rotation2d angle1 = Rotation2d.fromDegrees(90.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-90.0);
        
        double expected = 0.0; // These are effectively the same direction when ignoring polarity
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be 0 for angles at opposite boundaries.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarityNearBoundaries() {
        Rotation2d angle1 = Rotation2d.fromDegrees(85.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(-85.0);
        
        double expected = 10.0; // When accounting for flipping, these are 10 degrees apart
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should correctly handle angles near boundaries.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarityExtremeValues() {
        Rotation2d angle1 = Rotation2d.fromDegrees(450.0); // Equivalent to 90 degrees
        Rotation2d angle2 = Rotation2d.fromDegrees(-270.0); // Equivalent to -270 degrees
        
        double expected = 0.0; // These are effectively the same direction when ignoring polarity
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should correctly handle extreme angle values.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolaritySmallDifference() {
        Rotation2d angle1 = Rotation2d.fromDegrees(5.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(10.0);
        
        double expected = 5.0;
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be correct for small angle differences.");
    }
    
    @Test
    public void testDistanceBetweenAnglesIgnoringPolarityEdgeCase() {
        Rotation2d angle1 = Rotation2d.fromDegrees(0.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(180.0);
        
        double expected = 0.0; // These are effectively the same direction when ignoring polarity
        double actual = AngleHelpers.distanceBetweenAnglesIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, maxDelta, "The distance should be 0 for opposite angles.");
    }

    //
    // Test isAngleNearIgnoringPolarity
    //

    @Test
    public void testIsNearIgnoringPolarityShouldSucceedIfWheelFlipped() {
        Rotation2d angle1 = Rotation2d.fromDegrees(85);
        Rotation2d angle2 = Rotation2d.fromDegrees(85.0 + 8.0 + 180.0);
        
        boolean expected = true; // These are effectively the same direction when ignoring polarity
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "The angles should be considered near when ignoring polarity.");
    }

    @Test
    public void testIsNearIgnoringPolarityShouldReturnFalseIfTooFar() {
        Rotation2d angle1 = Rotation2d.fromDegrees(85);
        Rotation2d angle2 = Rotation2d.fromDegrees(85.0 + 8.01 + 180.0);
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "Just a hair further than the threshold.");
    }

    @Test
    public void testNegativeIsNearIgnoringPolarityShouldSucceedIfWheelFlipped() {
        Rotation2d angle1 = Rotation2d.fromDegrees(-85);
        Rotation2d angle2 = Rotation2d.fromDegrees(-85.0 - 8.0 + 180.0);
        
        boolean expected = true; // These are effectively the same direction when ignoring polarity
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "The angles should be considered near when ignoring polarity.");
    }

    @Test
    public void testNegativeIsNearIgnoringPolarityShouldReturnFalseIfTooFar() {
        Rotation2d angle1 = Rotation2d.fromDegrees(-85);
        Rotation2d angle2 = Rotation2d.fromDegrees(-85.0 - 8.01 + 180.0);
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "Just a hair further than the threshold.");
    }

    @Test
    public void testIsNearIgnoringPolarityBasicCase() {
        Rotation2d angle1 = Rotation2d.fromDegrees(30.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(45.0);
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "The angles should not be considered near.");
    }

    @Test
    public void testIsNearIgnoringPolarityBasicCase2() {
        Rotation2d angle1 = Rotation2d.fromDegrees(30.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(38.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "The angles should be considered near.");
    }

    @Test
    public void testIsNearIgnoringPolarityBasicCase3() {
        Rotation2d angle1 = Rotation2d.fromDegrees(400.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(400.0);
        
        boolean expected = true;
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "The angles should be considered near.");
    }

    @Test
    public void testIsNearIgnoringPolarityBasicCase4() {
        Rotation2d angle1 = Rotation2d.fromDegrees(400.0);
        Rotation2d angle2 = Rotation2d.fromDegrees(410.0);
        
        boolean expected = false;
        boolean actual = AngleHelpers.isAngleNearIgnoringPolarity(angle1, angle2);
        assertEquals(expected, actual, "The angles should not be considered near.");
    }

    //
    // Test getClosestAngleToReference
    //
    
    @Test
    public void testGetClosestAngleToReferenceOriginalCloser() {
        Rotation2d angleA = Rotation2d.fromDegrees(30.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(45.0);
        
        double expected = 30.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Original angle should be closer to reference.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceRotatedCloser() {
        Rotation2d angleA = Rotation2d.fromDegrees(30.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(240.0); // Closer to 30+180=210 than to 30
        
        double expected = -150.0; // 30+180=210, constrained to -180 to 180 is -150
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Rotated angle should be closer to reference.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceBoundaryCase() {
        Rotation2d angleA = Rotation2d.fromDegrees(90.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(100.0);
        
        double expected = 90.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Angle at boundary should return correct result.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceNegativeBoundaryCase() {
        Rotation2d angleA = Rotation2d.fromDegrees(-90.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(-100.0);
        
        double expected = -90.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Negative angle at boundary should return correct result.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceIdenticalAngles() {
        Rotation2d angleA = Rotation2d.fromDegrees(45.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(45.0);
        
        double expected = 45.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Identical angles should return the original angle.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceOppositeSides() {
        Rotation2d angleA = Rotation2d.fromDegrees(0.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(179.0);
        
        double expected = 180.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Should return original angle when reference is on opposite side but closer.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceLargeValues() {
        Rotation2d angleA = Rotation2d.fromDegrees(400.0); // Equivalent to 40 degrees
        Rotation2d referenceAngle = Rotation2d.fromDegrees(210.0); // Constrained -180 to 180, equivalent to -150 degrees
        
        double expected = -140.0; // 40+180=220, constrained to -180 to 180 is -140
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Should handle large angle values correctly.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceNegativeValues() {
        Rotation2d angleA = Rotation2d.fromDegrees(-30.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(-45.0);
        
        double expected = -30.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Should handle negative angles correctly.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceCrossingBoundary() {
        Rotation2d angleA = Rotation2d.fromDegrees(170.0); // Constrained to -90 to 90, equivalent to -10 degrees
        Rotation2d referenceAngle = Rotation2d.fromDegrees(-170.0); // Constrained to -180 to 180, equivalent to -170 degrees
        
        double expected = 170.0; // -10+180=170, constrained to -180 to 180 is 170
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "Should handle angles that cross boundaries correctly.");
    }
    
    @Test
    public void testGetClosestAngleToReferenceExactly90DegreesApart() {
        Rotation2d angleA = Rotation2d.fromDegrees(0.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(90.0);
        
        double expected = 0.0; // Original angle is closer to reference than rotated (180)
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "When angles are exactly 90 degrees apart, should return original.");
    }

    @Test
    public void testGetClosestAngleToReference91DegreesApart() {
        Rotation2d angleA = Rotation2d.fromDegrees(0.0);
        Rotation2d referenceAngle = Rotation2d.fromDegrees(91.0);
        
        double expected = 180.0;
        Rotation2d actual = AngleHelpers.getClosestAngleToReference(angleA, referenceAngle, false);
        assertEquals(expected, actual.getDegrees(), maxDelta, "When angles are 91 degrees apart, should return rotated.");
    }
}
