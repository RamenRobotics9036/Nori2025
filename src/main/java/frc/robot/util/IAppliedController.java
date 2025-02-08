package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IAppliedController {
    double getLeftY();
    double getRightY();
    double getLeftX();
    double getRightX();

    double getLeftTriggerAxis();
    double getRightTriggerAxis();

    public Trigger start();
    public Trigger a();
    public Trigger b();
}
