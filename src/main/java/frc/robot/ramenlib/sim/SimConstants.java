package frc.robot.ramenlib.sim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Class to hold constants that are simulation specific.
 */
public final class SimConstants {
    /**
     * Vision simulation constants for camera and pose estimation.
     */
    public static final class VisionSimConstants {
        public static final String kCameraName = "RAMEN SIM CAMERA";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
        // pitched upward.
        // NOTE: For reefscape, set camPitch to 15 degrees, since 30 degrees was too high to detect
        // the april tags near the coral reefs.
        private static final double camPitch = Units.degreesToRadians(15.0);
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, -camPitch, 0));

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    /**
     * Constants related to simulation-specific command behavior.
     */
    public static final class SimCommandConstants {
        public static final boolean kDisableMostCommandsInSim = true;
    }
}
