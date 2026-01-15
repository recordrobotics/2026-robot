package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.legacy.CoralIntakeFromGroundToggled;
import frc.robot.commands.legacy.ToggleRequirements;
import frc.robot.subsystems.CoralIntake.CoralIntakeState;

public class CoralIntakeFromGroundUpL1 extends SequentialCommandGroup {
    public CoralIntakeFromGroundUpL1() {
        addRequirements(RobotContainer.coralIntake);

        addCommands(
                new InstantCommand(() -> CoralIntakeFromGroundToggled.isGoingToL1 = false),
                // raise the arm
                new InstantCommand(
                        () -> RobotContainer.coralIntake.set(CoralIntakeState.UP),
                        RobotContainer.coralIntake,
                        ToggleRequirements.coralIntakeMoveToggleRequirement),
                new WaitUntilCommand(() -> RobotContainer.coralIntake.armAtGoal()),
                new ScheduleCommand(RobotContainer.lights
                        .coralIntake
                        .runPattern(Constants.Lights.PULSATING_ORANGE)
                        .onlyWhile(this::isScheduled)),
                new ScheduleCommand(RobotContainer.lights
                        .elevator
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.coralIntake.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.coralShooter.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.PULSATING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)));
    }
}
