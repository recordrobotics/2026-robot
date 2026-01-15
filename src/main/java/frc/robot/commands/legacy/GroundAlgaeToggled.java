package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMoveThenAlgaeGrabEnd;
import frc.robot.subsystems.ElevatorHead.GamePiece;

/**
 * @deprecated This is an old control scheme command and will be removed
 */
@Deprecated(forRemoval = true)
public class GroundAlgaeToggled extends Command {
    private ElevatorHeight targetHeight;

    private boolean isRunning = false;

    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public GroundAlgaeToggled(ElevatorHeight targetHeight) {
        this.targetHeight = targetHeight;
        addRequirements(ToggleRequirements.elevatorMoveToggleRequirement);
    }

    @Override
    public void initialize() {
        isRunning = true;
        ElevatorMoveThenAlgaeGrab.create(targetHeight, false)
                .andThen(new ElevatorMoveThenAlgaeGrabEnd(targetHeight, true))
                .andThen(() -> isRunning = false)
                .handleInterrupt(this::cancel)
                .schedule();
    }

    @Override
    public boolean isFinished() {
        return !isRunning;
    }

    @Override
    public void end(boolean interrupted) {
        isRunning = false;
        if (interrupted) {
            new ElevatorMoveThenAlgaeGrabEnd(
                            RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.ALGAE)
                                    ? targetHeight
                                    : ElevatorHeight.BOTTOM,
                            true)
                    .handleInterrupt(this::cancel)
                    .schedule();
        }
    }
}
