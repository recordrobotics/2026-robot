package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorHead.AlgaeGrabberStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;

public class ElevatorMoveThenAlgaeGrabEnd extends SequentialCommandGroup {
    public ElevatorMoveThenAlgaeGrabEnd(ElevatorHeight targetHeight, boolean withProxy) {
        addRequirements(RobotContainer.elevatorHead);

        addCommands(
                Commands.either(
                        Commands.either(
                                new InstantCommand(
                                        () -> RobotContainer.elevatorHead.set(AlgaeGrabberStates.HOLD_GROUND),
                                        RobotContainer.elevatorHead),
                                new InstantCommand(
                                        () -> RobotContainer.elevatorHead.set(AlgaeGrabberStates.HOLD_REEF),
                                        RobotContainer.elevatorHead),
                                () -> targetHeight == ElevatorHeight.GROUND_ALGAE),
                        new InstantCommand(
                                () -> RobotContainer.elevatorHead.set(AlgaeGrabberStates.OFF),
                                RobotContainer.elevatorHead),
                        () -> RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.ALGAE)),
                withProxy
                        ? Commands.either(
                                        new ElevatorMove(ElevatorHeight.GROUND_ALGAE),
                                        new ElevatorMove(ElevatorHeight.BOTTOM),
                                        () -> targetHeight == ElevatorHeight.GROUND_ALGAE)
                                .asProxy()
                        : Commands.either(
                                new ElevatorMove(ElevatorHeight.GROUND_ALGAE),
                                new ElevatorMove(ElevatorHeight.BOTTOM),
                                () -> targetHeight == ElevatorHeight.GROUND_ALGAE),
                new ScheduleCommand(RobotContainer.lights
                        .elevator
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.PULSATING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
    }
}
