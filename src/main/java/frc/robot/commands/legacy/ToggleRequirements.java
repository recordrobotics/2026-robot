package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * @deprecated This is an old control scheme command and will be removed
 */
@Deprecated(forRemoval = true)
public final class ToggleRequirements {

    public static class ElevatorMoveToggleRequirement extends SubsystemBase {}

    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public static final ElevatorMoveToggleRequirement elevatorMoveToggleRequirement =
            new ElevatorMoveToggleRequirement();

    public static class CoralIntakeMoveToggleRequirement extends SubsystemBase {}

    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public static final CoralIntakeMoveToggleRequirement coralIntakeMoveToggleRequirement =
            new CoralIntakeMoveToggleRequirement();

    private ToggleRequirements() {}
}
