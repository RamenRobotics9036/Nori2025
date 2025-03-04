package frc.robot.subsystems.VisionHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.Utils;

public class DetectedValue {
    private Pose2d m_absoluteTargetPose;
    private Pose2d m_robotPose;
    private double m_ta; // Area / confidence
    private double m_timeStamp;

    // Constructor
    // For testing, ability to override 'now'
    public DetectedValue(Pose2d absoluteTargetPoseIn, Pose2d robotPoseIn, double taIn, double currentTime) {
        if (absoluteTargetPoseIn == null) {
            Utils.throwGeneric("absoluteTargetPoseIn cannot be null");
        }
        if (robotPoseIn == null) {
            Utils.throwGeneric("robotPoseIn cannot be null");
        }
        if (absoluteTargetPoseIn.equals(new Pose2d())) {
            Utils.throwGeneric("absoluteTargetPoseIn cannot be zero");
        }
        if (robotPoseIn.equals(new Pose2d())) {
            Utils.throwGeneric("robotPoseIn cannot be zero");
        }

        m_absoluteTargetPose = absoluteTargetPoseIn;
        m_robotPose = robotPoseIn;
        m_ta = taIn;
        m_timeStamp = currentTime;
    }

    //
    // Getters
    //

    public Pose2d getAbsoluteTargetPose() {
        return m_absoluteTargetPose;
    }

    public Pose2d getRobotPose() {
        return m_robotPose;
    }

    public double getTa() {
        return m_ta;
    }

    public double getTimeStamp() {
        return m_timeStamp;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        DetectedValue other = (DetectedValue) obj;
        return m_absoluteTargetPose.equals(other.m_absoluteTargetPose) &&
               m_robotPose.equals(other.m_robotPose) &&
               Double.compare(other.m_ta, m_ta) == 0;
    }
}
