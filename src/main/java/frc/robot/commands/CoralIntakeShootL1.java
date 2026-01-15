package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;

public class CoralIntakeShootL1 extends SequentialCommandGroup {

    public CoralIntakeShootL1() {
        addCommands(
                new ScheduleCommand(RobotContainer.lights
                        .coralIntake
                        .runPattern(Constants.Lights.coralIntakePattern)
                        .onlyWhile(this::isScheduled)),
                new ScheduleCommand(RobotContainer.lights
                        .stateVisualizer
                        .runPattern(Constants.Lights.coralScorePattern)
                        .onlyWhile(this::isScheduled)),
                new InstantCommand(
                        () -> RobotContainer.coralIntake.set(CoralIntakeState.L1_SCORE), RobotContainer.coralIntake),
                new WaitCommand(Constants.CoralIntake.SHOOT_TIME),
                new InstantCommand(
                        () -> RobotContainer.coralIntake.set(CoralIntakeState.UP), RobotContainer.coralIntake),
                new ScheduleCommand(RobotContainer.lights
                        .coralIntake
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.FLASHING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
    }
}
