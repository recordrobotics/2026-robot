package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMove;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;

/**
 * @deprecated This is an old control scheme command and will be removed
 */
@Deprecated(forRemoval = true)
public final class ElevatorMoveThenAlgaeGrab extends SequentialCommandGroup {

    private Command algaeGrabberLightsCommand;

    private ElevatorMoveThenAlgaeGrab(ElevatorHeight targetHeight, boolean withProxy) {
        algaeGrabberLightsCommand = RobotContainer.lights.algaeGrabber.runPattern(Constants.Lights.PULSATING_ORANGE);

        addRequirements(RobotContainer.elevatorHead);

        addCommands(
                new ScheduleCommand(RobotContainer.lights
                        .elevator
                        .runPattern(Constants.Lights.elevatorPattern)
                        .onlyWhile(this::isScheduled)),
                withProxy ? new ElevatorMove(targetHeight).asProxy() : new ElevatorMove(targetHeight),
                new ScheduleCommand(algaeGrabberLightsCommand),
                new InstantCommand(
                        () -> {
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                                RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_GROUND);
                            else RobotContainer.elevatorHead.set(AlgaeGrabberStates.INTAKE_REEF);
                        },
                        RobotContainer.elevatorHead),
                new WaitUntilCommand(
                        () -> RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.ALGAE)),
                new InstantCommand(
                        () -> {
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.GROUND_ALGAE)
                                RobotContainer.elevatorHead.set(AlgaeGrabberStates.HOLD_GROUND);
                            else RobotContainer.elevatorHead.set(AlgaeGrabberStates.HOLD_REEF);
                        },
                        RobotContainer.elevatorHead),
                new InstantCommand(algaeGrabberLightsCommand::cancel),
                new ScheduleCommand(RobotContainer.lights
                        .algaeGrabber
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.PULSATING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
    }

    private void handleInterrupt() {
        algaeGrabberLightsCommand.cancel();
        RobotContainer.elevatorHead.set(AlgaeGrabberStates.OFF);
    }

    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public static Command create(ElevatorHeight targetHeight, boolean withProxy) {
        ElevatorMoveThenAlgaeGrab cmd = new ElevatorMoveThenAlgaeGrab(targetHeight, withProxy);
        return cmd.handleInterrupt( // refers to the method that registers a callback on interrupt
                cmd::handleInterrupt); // refers to the method that handles the interrupt
    }
}
