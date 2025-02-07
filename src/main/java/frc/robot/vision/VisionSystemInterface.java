package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public interface VisionSystemInterface {
    void initShuffleboad(ShuffleboardTab tab);
    void updatePose();
    double getTX();
    double getTY();
    double getTA();
    boolean isDetecting();
    double getID();
    Pose3d getTargetPose();
    Pose2d getRobotPose();
}
