package frc.robot.ramenutils.wheelcalibration;

import swervelib.SwerveDrive;
import swervelib.SwerveModule;

public class CalibrationOrchestrator {
    private final int m_numWheels;
    private final WheelCalibration[] m_wheelCalibrations;
    private int m_numReadings;
    private int m_numFailedReadings;

    // Constructor
    public CalibrationOrchestrator(SwerveDrive swerveDrive) {
        m_wheelCalibrations = createWheelCalibrations(swerveDrive);
        m_numWheels = m_wheelCalibrations.length;
    }

    private void resetReadingsHelper() {
        m_numReadings = 0;
        m_numFailedReadings = 0;

        for (int i = 0; i < m_numWheels; i++) {
            m_wheelCalibrations[i].resetReadings();
        }
    }

    private static WheelCalibration[] createWheelCalibrations(SwerveDrive swerveDrive) {
        SwerveModule[] swerveModules = swerveDrive.getModules();

        int numWheels = swerveModules.length;
        WheelCalibration[] wheelCalibrations = new WheelCalibration[numWheels];

        for (int i = 0; i < numWheels; i++) {
            wheelCalibrations[i] = new WheelCalibration(swerveModules[i]);
        }

        return wheelCalibrations;
    }

    private boolean areAllWheelsStraight() {
        boolean areAllWheelsStraight = true;
        for (int i = 0; i < m_numWheels; i++) {
            WheelCalibration wheel = m_wheelCalibrations[i];
            if (!wheel.isWheelStraight()) {
                areAllWheelsStraight = false;
                break;
            }
        }

        return areAllWheelsStraight;
    }

    public void takeReading() {
        if (!areAllWheelsStraight()) {
            System.out.println("ERROR: Expected takeReading() to only be called when all wheels are straight.");
            m_numFailedReadings++;
            return;
        }

        boolean gotReading = true;
        for (int i = 0; i < m_numWheels; i++) {
            WheelCalibration wheel = m_wheelCalibrations[i];
            if (!wheel.takeReading()) {
                System.out.println("ERROR: Reading should have succeeded since all wheels straight.");
                gotReading = false;
            }
        }
        if (gotReading) {
            m_numReadings++;
        } else {
            m_numFailedReadings++;
        }

        System.out.println("Num readings: " + m_numReadings + " num failed readings: " + m_numFailedReadings);
    }

    public void showReport() {
        if (m_numReadings == 0) {
            System.out.println("No readings taken yet!");
            return;
        }

        System.out.println("-------------------------");
        System.out.println("Calibration Report:");
        System.out.println("--------------------------");

        for (int i = 0; i < m_numWheels; i++) {
            WheelCalibration wheel = m_wheelCalibrations[i];
            System.out.println(wheel.getModuleName() + " (" + i + ") average angle: " 
            + Math.round(wheel.getAverageReading() * 10000.0) / 10000.0 
            + " (change degrees: " + Math.round(wheel.getChangeInDegrees() * 100.0) / 100.0 + ")");
        }
    }

    public void resetReadings() {
        resetReadingsHelper();
        System.out.println("Reset readings for all wheels.");
    }
}
