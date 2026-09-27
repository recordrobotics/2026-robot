package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shootorchestrator.ShootOrchestrator.FeedMode;

public class KidsShoot extends SequentialCommandGroup {

    public KidsShoot() {
        addCommands(
                Commands.runOnce(
                        () -> {
                            RobotContainer.shootOrchestrator.setFeedMode(FeedMode.AUTO);
                        },
                        RobotContainer.shooter),
                Commands.waitUntil(() ->
                                RobotContainer.feeder.isTopBeamBroken() || RobotContainer.feeder.isBottomBeamBroken())
                        .andThen(Commands.waitSeconds(0.15))
                        .withTimeout(2),
                Commands.runOnce(
                        () -> {
                            RobotContainer.shootOrchestrator.setFeedMode(FeedMode.DISABLED);
                        },
                        RobotContainer.shooter));
    }
}
