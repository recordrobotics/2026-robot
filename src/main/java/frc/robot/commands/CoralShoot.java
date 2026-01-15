package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorHead.CoralShooterStates;
import frc.robot.subsystems.ElevatorHead.GamePiece;

public class CoralShoot extends SequentialCommandGroup {

    private static boolean failedToShoot = false;

    public CoralShoot() {
        addRequirements(RobotContainer.elevatorHead);

        addCommands(
                new InstantCommand(
                        () -> {
                            setFailedToShoot(false);
                            if (RobotContainer.elevator.getNearestHeight() == ElevatorHeight.L4
                                    || RobotContainer.elevator.getNearestHeight() == ElevatorHeight.BARGE_ALGAE)
                                RobotContainer.elevatorHead.set(CoralShooterStates.OUT_BACKWARD);
                            else RobotContainer.elevatorHead.set(CoralShooterStates.OUT_FORWARD);
                        },
                        RobotContainer.elevatorHead),
                // Make sure coral left
                new WaitUntilCommand(() ->
                                !RobotContainer.elevatorHead.getGamePiece().atLeast(GamePiece.CORAL))
                        .raceWith(new WaitCommand(Constants.ElevatorHead.SHOOT_STALL_TIME)
                                .andThen(new WaitUntilCommand(() -> {
                                    if (RobotContainer.elevator.getNearestHeight() != ElevatorHeight.L4) return false;

                                    boolean stalled = Math.abs(RobotContainer.elevatorHead.getVelocity())
                                            < Constants.ElevatorHead.SHOOT_STALL_THRESHOLD;

                                    if (stalled) {
                                        setFailedToShoot(true);
                                    }

                                    return stalled;
                                }))),
                new InstantCommand(
                        () -> RobotContainer.elevatorHead.set(CoralShooterStates.OFF), RobotContainer.elevatorHead));
    }

    private static void setFailedToShoot(boolean value) {
        failedToShoot = value;
    }

    public static boolean failedToShoot() {
        return failedToShoot;
    }
}
