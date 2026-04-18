package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.util.Optional;

public class KidsShoot extends SequentialCommandGroup {

    public KidsShoot(double sweepPos) {
        addCommands(
                Commands.runOnce(
                        () -> {
                            RobotContainer.shootOrchestrator.turretAngleOverride = Optional.of(sweepPos);

                            RobotContainer.shootOrchestrator.setEnableShooting(true);
                        },
                        RobotContainer.shooter),
                Commands.waitUntil(() ->
                                RobotContainer.feeder.isTopBeamBroken() || RobotContainer.feeder.isBottomBeamBroken())
                        .andThen(Commands.waitSeconds(0.15))
                        .withTimeout(2),
                Commands.runOnce(
                        () -> {
                            RobotContainer.shootOrchestrator.setEnableShooting(false);
                        },
                        RobotContainer.shooter));
    }
}
