/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.ramenutils.sim.simvision;

import java.util.List;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.VisionConstants;
import frc.robot.ramenutils.sim.SimConstants.VisionSimConstants;
import frc.robot.Robot;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
 
public class VisionSim {
    // Simulation
    private PhotonCamera m_camera;
    private PhotonCameraSim m_cameraSim;
    private VisionSystemSim m_visionSim;
    private Optional<PhotonTrackedTarget> m_bestTarget = Optional.empty();
 
    public VisionSim() {
        if (!Robot.isSimulation()) {
            throw new RuntimeException("VisionSim should only be used in simulation");
        }

        m_camera = new PhotonCamera(VisionSimConstants.kCameraName);

        // Create the vision system simulation which handles cameras and targets on the field.
        m_visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
        m_visionSim.addAprilTags(VisionConstants.kTagLayout);
        // Create simulated camera properties. These can be set to mimic your actual camera.
        var cameraProp = new SimCameraProperties();
        cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
        cameraProp.setCalibError(0.35, 0.10);
        cameraProp.setFPS(70);
        cameraProp.setAvgLatencyMs(30);
        cameraProp.setLatencyStdDevMs(10);
        // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
        // targets.
        m_cameraSim = new PhotonCameraSim(m_camera, cameraProp);
        // Add the simulated camera to view the targets on this simulated field.
        m_visionSim.addCamera(m_cameraSim, VisionSimConstants.kRobotToCam);

        // We disable the wire frame since we never open the PhotonVision local webview
        m_cameraSim.enableDrawWireframe(false);
    }
 
    public void simulationPeriodic(Pose2d robotSimPose) {
        m_visionSim.update(robotSimPose);
        calcNewBestTarget();
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) m_visionSim.resetRobotPose(pose);
    }
 
    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return m_visionSim.getDebugField();
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        return m_bestTarget;
    }

    // If there's new information from the camera, we process it, and SAVE the best
    // target.  That way, we still see that target even if there's no new camera info
    // to read.
    private void calcNewBestTarget() {
        // Read in relevant data from the Camera
        List<PhotonPipelineResult> pipeline_result_list = m_camera.getAllUnreadResults();
        if (pipeline_result_list.isEmpty()) {
            return;
        }

        // Camera processed a new frame since last
        // Get the last one in the list.
        PhotonPipelineResult most_recent = pipeline_result_list.get(pipeline_result_list.size() - 1);
        if (!most_recent.hasTargets()) {
            // Save that we saw NO targets
            m_bestTarget = Optional.empty();
            return;
        }

        PhotonTrackedTarget bestTarget = most_recent.getBestTarget();
        m_bestTarget = Optional.of(bestTarget);
    }
}
