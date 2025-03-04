package frc.robot.commands.testcommands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;

// Shared state for test commands.

public class WheelTestContext {
    // Shared handle for the overall test sequence.
    private Command m_cancellableCommand = null;

    private Alert m_alertSwerveTestSucceeded = null;
    private Alert m_alertSwerveTestFailed = null;
    private boolean m_shuffleInitialized = false;
    private final String m_generalFailText = "Swerve test failed";
    private Boolean m_testFailed = false;

    // Constructor
    public WheelTestContext() {
        reset();
    }

    private void setAlertSucceeded() {
        if (m_shuffleInitialized) {
            m_alertSwerveTestSucceeded.set(true);
            m_alertSwerveTestFailed.set(false);
        }
    }

    private void setAlertFailed(String errorMsg) {
        if (errorMsg == null) {
            errorMsg = m_generalFailText;
        }

        if (m_shuffleInitialized) {
            m_alertSwerveTestSucceeded.set(false);

            m_alertSwerveTestFailed.setText(errorMsg);
            m_alertSwerveTestFailed.set(true);
        }
    }

    private void setAlertCleared() {
        if (m_shuffleInitialized) {
            m_alertSwerveTestSucceeded.set(false);
            m_alertSwerveTestFailed.set(false);
        }
    }

    public void initShuffleboard() {
        m_alertSwerveTestSucceeded = new Alert("Swerve test succeeded", AlertType.kInfo);
        m_alertSwerveTestFailed = new Alert(m_generalFailText, AlertType.kError);
        setAlertCleared();

        m_shuffleInitialized = true;
    }

    public void setCancellableCommand(Command command) {
        m_cancellableCommand = command;
    }

    // Reset context on the current test run.  But do NOT reset cancellableCommand,
    // since this object is reused across multiple test runs.
    public void reset() {
        m_testFailed = false;
        setAlertCleared();
    }

    public boolean getResult() {
        return m_testFailed;
    }

    public void cancelRunningTestWithError(String errorMsg) {
        m_testFailed = true;

        System.err.println("Swerve test failed: " + errorMsg);

        if (m_cancellableCommand != null) {
            m_cancellableCommand.cancel();
        }
        else {
            System.err.println("Expected m_cancellableCommand to be set");
        }

        setAlertFailed(errorMsg);
    }

    public void testSucceeded() {
        m_testFailed = false;
        setAlertSucceeded();
    }
}
