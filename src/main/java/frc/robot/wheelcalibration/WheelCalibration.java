package frc.robot.wheelcalibration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import swervelib.SwerveModule;

public class WheelCalibration {
    private SwerveModule m_swerveModule;
    private double m_runningTotalAngles;
    private int m_numReadings;

    // Constructor
    public WheelCalibration(SwerveModule swerveModule) {
        m_swerveModule = swerveModule;
        resetReadingsHelper();
    }

    public boolean takeReading() {
        if (!isWheelStraight()) {
            System.out.println("ERROR: Expected takeReading() to only be called when wheel is straight.");
            return false;
        }

        // After we get the wheel reading, we may flip the polarity so that the reading
        // is close to configured offset.  This is so that we recommend an offset thats
        // in the same vacinity as the current configured offset.        
        Rotation2d currentAngle = readCurrentAbsoluteAngleWithOffset();
        currentAngle = AngleHelpers.getClosestAngleToReference(
            currentAngle,
            Rotation2d.fromDegrees(getConfigationOffsetDegrees()));

        recordReading(currentAngle);

        return true;
    }

    // Return false if the wheel is turned too far from straight.
    public boolean isWheelStraight() {
        Rotation2d currentAngleNoPolarity = AngleHelpers.normalizeAngle(
            readCurrentAbsoluteAngleWithOffset(), -90, 90);

        Rotation2d offsetAngleNoPolarity = AngleHelpers.normalizeAngle(
            Rotation2d.fromDegrees(getConfigationOffsetDegrees()), -90, 90);

        return AngleHelpers.isAngleNear(currentAngleNoPolarity, offsetAngleNoPolarity);
    }

    public boolean isConfigOffsetInGoodRange() {
        double offsetDegrees = getConfigationOffsetDegrees();
        if (offsetDegrees <= -180 || offsetDegrees >= 180) {
            double recommendedOffset = Math.round(AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(offsetDegrees), -180, 180).getDegrees() * 100000.0) / 100000.0;
            System.out.println("ERROR: " + getModuleName() + ": Config offset is not in correct range (-180, 180): " + offsetDegrees + " (instead use " + recommendedOffset + " degrees");
            return false;
        }

        return true;
    }

    private void recordReading(Rotation2d angle) {
        m_runningTotalAngles += angle.getDegrees();
        m_numReadings++;

        System.out.println("Reading: " + getModuleName() + " angle: " + Math.round(angle.getDegrees() * 10000.0) / 10000.0);
    }

    private void resetReadingsHelper() {
        m_runningTotalAngles = 0.0;
        m_numReadings = 0;
    }

    public void resetReadings() {
        resetReadingsHelper();
    }

    public double getAverageReading() {
        if (m_numReadings == 0) {
            System.out.println("ERROR: No readings taken yet.");
            return -1.0;
        }

        return m_runningTotalAngles / m_numReadings;
    }

    // Returns an absolute value
    public double getChangeInDegrees() {
        double averageReading = getAverageReading();
        if (averageReading == -1.0) {
            return -1.0;
        }

        double offsetDegrees = getConfigationOffsetDegrees();
        double changeInDegrees = Math.abs(averageReading - offsetDegrees);

        return changeInDegrees;
    }

    public String getModuleName() {
        return m_swerveModule.configuration.name;
    }

    // Absolute position, constrained to -180 to 180 degrees.
    private Rotation2d readCurrentRawAbsoluteAngle() {
        SwerveModuleState moduleState = m_swerveModule.getState();
        Rotation2d encoderAngle = moduleState.angle;

        return AngleHelpers.normalizeAngle(encoderAngle, -180, 180);
    }

    // Constrained -180 to 180 degrees.
    private Rotation2d readCurrentAbsoluteAngleWithOffset() {
        double resultDegrees = readCurrentRawAbsoluteAngle().getDegrees() + getConfigationOffsetDegrees();
        return AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(resultDegrees), -180, 180);
    }

    // Can be a negative value.  Note that we dont constrain the return value.  However,
    // somethign is very fishy if the configuration offset is greater than 360 degrees.
    private double getConfigationOffsetDegrees() {
        double offsetDegrees = m_swerveModule.configuration.angleOffset;

        if (offsetDegrees <= -360 || offsetDegrees >= 360) {
            System.out.println("Error: Avoid offset angle >360 degrees in config: " + offsetDegrees);
        }

        return offsetDegrees;
    }
}
