package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

public class DetectedValue {
    public Pose2d absoluteTargetPose;
    public Pose2d robotPose;
    public double ta; // Area / confidence
    public double timeStamp;

    // Constructor
    public DetectedValue(Pose2d absoluteTargetPoseIn, Pose2d robotPoseIn, double taIn) {
        if (absoluteTargetPoseIn == null) {
            throw new IllegalArgumentException("absoluteTargetPoseIn cannot be null");
        }
        if (robotPoseIn == null) {
            throw new IllegalArgumentException("robotPoseIn cannot be null");
        }
        if (absoluteTargetPoseIn.equals(new Pose2d())) {
            throw new IllegalArgumentException("absoluteTargetPoseIn cannot be zero");
        }
        if (robotPoseIn.equals(new Pose2d())) {
            throw new IllegalArgumentException("robotPoseIn cannot be zero");
        }

        absoluteTargetPose = absoluteTargetPoseIn;
        robotPose = robotPoseIn;
        ta = taIn;
        timeStamp = DriverStation.getMatchTime();
    }
}
