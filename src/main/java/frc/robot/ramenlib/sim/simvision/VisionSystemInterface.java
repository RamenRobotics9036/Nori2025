package frc.robot.ramenlib.sim.simvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VisionSystemInterface {
    void updatePoses();
    double getTX();
    double getTY();
    double getTA();
    boolean isDetecting();
    int getID();
    Pose3d getAbsoluteTargetPose();
    Pose3d getRelativeTargetPose();
    Pose2d getRobotPose();
}
