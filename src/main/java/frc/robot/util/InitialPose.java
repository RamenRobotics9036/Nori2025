package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

/**
 * Utility class for determining the initial pose of the robot based on FMS data
 */
public class InitialPose {    
    /**
     * Determines if the FMS is connected and reporting alliance information
     * @return true if FMS is properly connected
     */
    private static boolean isFmsReady() {
        return DriverStation.isFMSAttached() && DriverStation.getAlliance().isPresent();
    }

    /**
     * Gets the current alliance color from the FMS
     * @return The alliance color (Red or Blue), default to Blue.
     */
    private static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }
    
    /**
     * Calculates the initial pose based on alliance
     * @return The initial Pose2d for the robot
     */
    public static Pose2d getInitialPose() {
        // Default pose if FMS data is not available
        Pose2d defaultPose = Constants.FieldConstants.kCenterStationPose;
        
        // Check if FMS is connected and reporting alliance
        if (!isFmsReady()) {
            System.out.println("Error: FMS not ready, using default pose");
            return defaultPose;
        }
        
        Alliance alliance = getAlliance();
        
        // If alliance is RED, we need to mirror the pose across the field
        boolean isRed = (alliance == Alliance.Red);
        
        // Get the appropriate station pose based on position
        Pose2d stationPose = Constants.FieldConstants.kCenterStationPose;

        // If we're on red alliance, mirror the pose
        if (isRed) {
            // Mirror across the field width
            double fieldWidth = VisionConstants.kTagLayout.getFieldWidth();

            return new Pose2d(
                stationPose.getX(),
                fieldWidth - stationPose.getY(),
                Rotation2d.fromDegrees(-stationPose.getRotation().getDegrees())
            );
        } else {
            // Blue alliance, use pose as-is
            return stationPose;
        }
    }
}
