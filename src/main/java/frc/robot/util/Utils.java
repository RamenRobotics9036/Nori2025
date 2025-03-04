package frc.robot.util;

import frc.robot.Robot;

public class Utils {
    public static void throwGeneric(String message) {
        // On the real robot, we just print the message
        System.out.println("*** " + message);

        // Print the call stack
        for (StackTraceElement element : Thread.currentThread().getStackTrace()) {
            System.out.println(element);
        }
        System.out.println("*** END");
        System.out.println("_");

        if (Robot.isSimulation()) {
            throw new RuntimeException(message);
        }
    }
}
