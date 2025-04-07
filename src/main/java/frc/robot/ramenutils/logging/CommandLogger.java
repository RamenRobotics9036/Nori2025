package frc.robot.ramenutils.logging;

import java.util.Set;
import java.util.stream.Collectors;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class CommandLogger {
    private StringLogEntry m_commandsLog = null;

    // Constructor
    public CommandLogger() {
        m_commandsLog = new StringLogEntry(DataLogManager.getLog(), "/my/Commands/SchedulerCommandLog");
        setupLoggingCallbacks();
    }

    private void setupLoggingCallbacks() {
        CommandScheduler.getInstance().onCommandInitialize(
            command -> {
                // Log when a command is initialized
                m_commandsLog.append("Initialized: " + getCommandDescription(command));
        });

        CommandScheduler.getInstance().onCommandInterrupt(
            (command, interruptingCommand) -> {
                String message = "Interrupted: " + getSimpleCommandName(command);
                if (interruptingCommand.isPresent()) {
                    message += " by " + getSimpleCommandName(interruptingCommand.get());
                }
                m_commandsLog.append(message);
        });

        CommandScheduler.getInstance().onCommandFinish(
            command -> {
                // Log when a command is finished
                m_commandsLog.append("Finished:    " + getSimpleCommandName(command));
            });
    }

    private String getSimpleCommandName(Command command) { 
        String name = command.getName(); 
        if (name == null || name.isEmpty()) { 
            return "UNKNOWN";
        } 

        return "[" + name + "]"; 
    } 
 
    private String getCommandDescription(Command command) { 
        StringBuilder sb = new StringBuilder(); 

        String name = getSimpleCommandName(command); 
        sb.append(name); 

        Set<Subsystem> requirements = command.getRequirements(); 
        if (requirements != null && !requirements.isEmpty()) { 
            sb.append(" (Requires:"); 
            sb.append(requirements.stream() 
                .map(Subsystem::getName) 
                .collect(Collectors.joining(","))); 
            sb.append(")"); 
        } 

        return sb.toString(); 
    } 
}
