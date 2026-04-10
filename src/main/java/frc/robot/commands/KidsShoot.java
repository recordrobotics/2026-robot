package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class KidsShoot extends SequentialCommandGroup {

    public KidsShoot() {
        addRequirements(RobotContainer.shooter);
        addCommands(
                Commands.runOnce(
                        () -> {
                            RobotContainer.shootOrchestrator.setEnableShooting(true);
                        },
                        RobotContainer.shooter),
                Commands.waitSeconds(2),
                Commands.runOnce(
                        () -> {
                            RobotContainer.shootOrchestrator.setEnableShooting(false);
                        },
                        RobotContainer.shooter));
    }
}
