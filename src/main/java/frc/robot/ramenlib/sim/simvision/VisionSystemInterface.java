package frc.robot.ramenlib.sim.simvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

/**
 * This interface defines the methods for a vision system in a robotics simulation.
 * It provides methods to update poses, retrieve target information, and get the robot's pose.
 */
public interface VisionSystemInterface {
    /**
     * Updates the poses of the vision system.
     */
    void updatePoses();

    /**
     * Retrieves the horizontal offset from the crosshair to the target.
     *
     * @return the horizontal offset in degrees.
     */
    @SuppressWarnings("AbbreviationAsWordInNameCheck")
    double getTX();

    /**
     * Retrieves the vertical offset from the crosshair to the target.
     *
     * @return the vertical offset in degrees.
     */
    @SuppressWarnings("AbbreviationAsWordInNameCheck")
    double getTY();

    /**
     * Retrieves the target area as a percentage of the image.
     *
     * @return the target area percentage.
     */
    @SuppressWarnings("AbbreviationAsWordInNameCheck")
    double getTA();

    /**
     * Checks if the vision system is currently detecting a target.
     *
     * @return true if a target is detected, false otherwise.
     */
    boolean isDetecting();

    /**
     * Retrieves the unique identifier for the target.
     *
     * @return the unique identifier as an integer.
     */
    @SuppressWarnings("AbbreviationAsWordInNameCheck")
    int getID();

    /**
     * Retrieves the absolute pose of the target in 3D space.
     *
     * @return the absolute target pose as a Pose3d object.
     */
    Pose3d getAbsoluteTargetPose();

    /**
     * Retrieves the relative pose of the target with respect to the robot.
     *
     * @return the relative target pose as a Pose3d object.
     */
    Pose3d getRelativeTargetPose();

    /**
     * Retrieves the current pose of the robot.
     *
     * @return the robot's pose as a Pose2d object.
     */
    Pose2d getRobotPose();
}
