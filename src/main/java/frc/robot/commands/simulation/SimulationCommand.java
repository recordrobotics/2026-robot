package frc.robot.commands.simulation;

import edu.wpi.first.wpilibj2.command.Command;

public interface SimulationCommand {

    /**
     * Attached this simulation command to a sensor command ONLY IF IN SIMULATION MODE!
     *
     * @param command the command to attach to
     * @return Either a new command group or the original command if not in simulation mode
     */
    Command simulateFor(Command command);
}
