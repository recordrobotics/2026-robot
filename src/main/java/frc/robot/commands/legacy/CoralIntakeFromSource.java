package frc.robot.commands.legacy;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.commands.ElevatorMove;
import frc.robot.commands.simulation.CoralIntakeToElevator;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;
import frc.robot.utils.CommandUtils;

/**
 * @deprecated This is an old control scheme command and will be removed
 */
@Deprecated(forRemoval = true)
public class CoralIntakeFromSource extends SequentialCommandGroup {
    /**
     * @deprecated This is an old control scheme command and will be removed
     */
    @Deprecated(forRemoval = true)
    public CoralIntakeFromSource(boolean useProxy) {
        addCommands(
                new ScheduleCommand(RobotContainer.lights
                        .elevator
                        .runPattern(Constants.Lights.elevatorPattern)
                        .onlyWhile(this::isScheduled)),
                new ScheduleCommand(RobotContainer.lights
                        .coralIntake
                        .runPattern(Constants.Lights.coralIntakePattern)
                        .onlyWhile(this::isScheduled)),
                new ScheduleCommand(RobotContainer.lights
                        .coralShooter
                        .runPattern(Constants.Lights.elevatorHeadPattern)
                        .onlyWhile(this::isScheduled)),
                new ScheduleCommand(RobotContainer.lights
                        .stateVisualizer
                        .runPattern(Constants.Lights.PULSATING_ORANGE)
                        .onlyWhile(this::isScheduled)),
                Commands.sequence(
                                new InstantCommand(
                                        () -> RobotContainer.coralIntake.set(CoralIntakeState.SOURCE),
                                        RobotContainer.coralIntake),
                                // start moving elevator to intake position
                                CommandUtils.maybeProxy(useProxy, new ElevatorMove(ElevatorHeight.INTAKE)),
                                new InstantCommand(
                                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.INTAKE),
                                        RobotContainer.elevatorHead),
                                new WaitUntilCommand(() -> RobotContainer.elevator.atGoal()),
                                new CoralIntakeToElevator()
                                        .simulateFor(new WaitUntilCommand(() -> RobotContainer.elevatorHead
                                                .getGamePiece()
                                                .atLeast(GamePiece.CORAL))),
                                Commands.sequence(
                                                new InstantCommand(
                                                        () -> RobotContainer.coralIntake.set(CoralIntakeState.UP),
                                                        RobotContainer.coralIntake),
                                                // move coral a set distance
                                                new InstantCommand(
                                                        () -> RobotContainer.elevatorHead.moveBy(
                                                                Constants.ElevatorHead.CORAL_INTAKE_DISTANCE),
                                                        RobotContainer.elevatorHead),
                                                new WaitUntilCommand(() -> RobotContainer.elevatorHead
                                                        .getGamePiece()
                                                        .atLeast(GamePiece.CORAL_POSITIONED)))
                                        .onlyWhile(() -> RobotContainer.elevatorHead
                                                .getGamePiece()
                                                .atLeast(GamePiece.CORAL)))
                        .repeatedly()
                        .onlyWhile(() ->
                                !RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL_POSITIONED)),
                new InstantCommand(
                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.OFF), RobotContainer.elevatorHead),
                new ScheduleCommand(RobotContainer.lights
                        .elevator
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.coralIntake.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.coralShooter.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.PULSATING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
    }
}
