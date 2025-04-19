package frc.robot.wheelcalibration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swervelib.SwerveModule;

/**
 * The WheelCalibration class is responsible for calibrating the swerve module's wheel angles.
 * It provides methods to take readings, check alignment, and calculate angle offsets.
 */
public class WheelCalibration {
    private SwerveModule m_swerveModule;
    private double m_runningTotalAngles;
    private int m_numReadings;

    /**
     * Constructor for WheelCalibration.
     *
     * @param swerveModule The swerve module to be calibrated.
     */
    public WheelCalibration(SwerveModule swerveModule) {
        m_swerveModule = swerveModule;
        resetReadingsHelper();
    }

    /**
     * Takes a reading of the current wheel angle if the wheel is straight.
     * If the wheel is not straight, an error message is printed and the method returns false.
     *
     * @return true if the reading was successfully taken, false otherwise.
     */
    public boolean takeReading() {
        if (!isWheelStraight()) {
            System.out
                .println("ERROR: Expected takeReading() to only be called when wheel is straight.");
            return false;
        }

        // After we get the wheel reading, we may flip the polarity so that the reading
        // is close to configured offset. This is so that we recommend an offset thats
        // in the same vacinity as the current configured offset.
        Rotation2d currentAngle = readCurrentAbsoluteAngleWithOffset();
        currentAngle = AngleHelpers.getClosestAngleToReference(
            currentAngle,
            Rotation2d.fromDegrees(getConfigationOffsetDegrees()));

        recordReading(currentAngle);

        return true;
    }

    // Return false if the wheel is turned too far from straight.
    /**
     * Checks if the wheel is straight by comparing the current angle with the
     * configured offset angle.
     *
     * @return true if the wheel is straight, false otherwise.
     */
    public boolean isWheelStraight() {
        Rotation2d currentAngleNoPolarity = AngleHelpers.normalizeAngle(
            readCurrentAbsoluteAngleWithOffset(),
            -90,
            90);

        Rotation2d offsetAngleNoPolarity = AngleHelpers.normalizeAngle(
            Rotation2d.fromDegrees(getConfigationOffsetDegrees()),
            -90,
            90);

        return AngleHelpers.isAngleNear(currentAngleNoPolarity, offsetAngleNoPolarity);
    }

    /**
     * Checks if the configuration offset is within the valid range (-180, 180).
     * If not, it prints an error message with a recommended offset value.
     *
     * @return true if the configuration offset is in the valid range, false otherwise.
     */
    public boolean isConfigOffsetInGoodRange() {
        double offsetDegrees = getConfigationOffsetDegrees();
        if (offsetDegrees <= -180 || offsetDegrees >= 180) {
            double recommendedOffset = Math.round(
                AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(offsetDegrees), -180, 180)
                    .getDegrees() * 100000.0)
                / 100000.0;
            System.out.println(
                "ERROR: "
                    + getModuleName()
                    + ": Config offset is not in correct range (-180, 180): "
                    + offsetDegrees
                    + " (instead use "
                    + recommendedOffset
                    + " degrees");
            return false;
        }

        return true;
    }

    private void recordReading(Rotation2d angle) {
        m_runningTotalAngles += angle.getDegrees();
        m_numReadings++;

        System.out.println(
            "Reading: "
                + getModuleName()
                + " angle: "
                + Math.round(angle.getDegrees() * 10000.0) / 10000.0);
    }

    private void resetReadingsHelper() {
        m_runningTotalAngles = 0.0;
        m_numReadings = 0;
    }

    /**
     * Resets the readings by clearing the running total and the number of readings.
     */
    public void resetReadings() {
        resetReadingsHelper();
    }

    /**
     * Calculates and returns the average of the recorded wheel angle readings.
     * If no readings have been taken, it prints an error message and returns -1.0.
     *
     * @return The average wheel angle reading, or -1.0 if no readings are available.
     */
    public double getAverageReading() {
        if (m_numReadings == 0) {
            System.out.println("ERROR: No readings taken yet.");
            return -1.0;
        }

        return m_runningTotalAngles / m_numReadings;
    }

    // Returns an absolute value
    /**
     * Calculates the absolute change in degrees between the average reading and the
     * configuration offset.  If no readings are available, it returns -1.0.
     *
     * @return The absolute change in degrees, or -1.0 if no readings are available.
     */
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
        double resultDegrees = readCurrentRawAbsoluteAngle().getDegrees()
            + getConfigationOffsetDegrees();
        return AngleHelpers.normalizeAngle(Rotation2d.fromDegrees(resultDegrees), -180, 180);
    }

    private double getConfigationOffsetDegrees() {
        return m_swerveModule.configuration.angleOffset;
    }
}
