package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.KillableSubsystem;

public class KillSpecified extends Command {

    private KillableSubsystem[] subsytems;
    private Boolean shouldContinuouslyExecute;

    /**
     * Kills all subsystems inputted
     *
     * @param subsystems the subsystems to kill
     */
    public KillSpecified(KillableSubsystem... subsystems) {
        this(false, subsystems);
    }

    /**
     * Kills all subsystems inputted
     *
     * @param shouldContinuouslyExecute whether or not the command should run continuously
     * @param subsystems                the subsystems to kill
     */
    public KillSpecified(Boolean shouldContinuouslyExecute, KillableSubsystem... subsystems) {
        addRequirements(subsystems);
        this.subsytems = subsystems;
        this.shouldContinuouslyExecute = shouldContinuouslyExecute;
    }

    @Override
    public void initialize() {
        for (KillableSubsystem subsystem : subsytems) {
            subsystem.kill();
        }
    }

    @Override
    public void execute() {
        for (KillableSubsystem subsystem : subsytems) {
            subsystem.kill();
        }
    }

    @Override
    public void end(boolean interrupted) {
        /* nothing to do on end */
    }

    @Override
    public boolean isFinished() {
        return !shouldContinuouslyExecute;
    }
}
