package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CoralIntakeFromGround;

/**
 * @deprecated This is an old control scheme command and will be removed
 */
@Deprecated(forRemoval = true)
public class CoralIntakeFromGroundToggled extends Command {

    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public static boolean isGoingToL1 = false;

    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public CoralIntakeFromGroundToggled() {
        addRequirements(ToggleRequirements.coralIntakeMoveToggleRequirement);
    }

    @Override
    public void initialize() {
        new CoralIntakeFromGround().handleInterrupt(this::cancel).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (!isGoingToL1) {
            new CoralIntakeFromGroundUp().handleInterrupt(this::cancel).schedule();
        }
    }
}
