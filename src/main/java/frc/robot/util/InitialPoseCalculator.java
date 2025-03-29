package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.VisionConstants;

/**
 * Utility class for determining the initial pose of the robot based on FMS data
 */
public final class InitialPoseCalculator {    
    /**
     * Private constructor to prevent instantiation of this utility class
     */
    private InitialPoseCalculator() {
        throw new UnsupportedOperationException("This is a utility class and cannot be instantiated");
    }

    /**
     * Determines if the FMS is connected and reporting alliance information
     * @return true if FMS is properly connected
     */
    private static boolean isFmsReady() {
        return DriverStation.isFMSAttached() && DriverStation.getAlliance().isPresent();
    }

    /**
     * Gets the current alliance color from the FMS
     * @return The alliance color (Red or Blue), default to Red.
     */
    public static Alliance getAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }
    
    /**
     * Calculates the initial pose based on alliance
     * @return The initial Pose2d for the robot
     */
    public static Pose2d getCalculatedInitialPose(Pose2d nonmirroredInitialPose) {
        // Check if FMS is connected and reporting alliance
        if (!isFmsReady()) {
            System.out.println("Warning: FMS not ready, using drivestation Alliance");
        }

        Alliance alliance = getAlliance();
        
        // If alliance is RED, we need to mirror the pose across the field
        boolean isRed = (alliance == Alliance.Red);

        Pose2d result = nonmirroredInitialPose;

        // If we're on red alliance, mirror the pose
        if (isRed) {
            // Mirror across the field length
            double fieldLength = VisionConstants.kTagLayout.getFieldLength();

            result = new Pose2d(
                fieldLength - nonmirroredInitialPose.getX(),
                nonmirroredInitialPose.getY(),
                Rotation2d.fromDegrees(nonmirroredInitialPose.getRotation().getDegrees() + 180.0)
            );
        }

        return result;
    }
}
