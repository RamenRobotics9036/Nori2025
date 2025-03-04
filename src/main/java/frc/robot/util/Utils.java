package frc.robot.util;

import frc.robot.Robot;

public class Utils {
    public static void throwGeneric(String message) {
        // On the real robot, we just print the message
        System.err.println(message);

        if (Robot.isSimulation()) {
            throw new RuntimeException(message);
        }
    }
}
