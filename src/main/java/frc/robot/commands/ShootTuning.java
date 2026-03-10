package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShootTuning extends SequentialCommandGroup {

    public ShootTuning() {
        double hoodStep =
                (Constants.Shooter.HOOD_MIN_POSITION_RADIANS - Constants.Shooter.HOOD_MAX_POSITION_RADIANS) / 10.0;
        double velocityStep = (18 - 6) / 10.0;

        addRequirements(RobotContainer.shooter);
        addCommands(new InstantCommand(
                () -> {
                    RobotContainer.shootOrchestrator.setEnableShooting(true);
                },
                RobotContainer.shooter));

        for (double angle = Constants.Shooter.HOOD_MAX_POSITION_RADIANS;
                angle >= Constants.Shooter.HOOD_MIN_POSITION_RADIANS;
                angle += hoodStep) {
            for (double velocity = 6; velocity <= 18; velocity += velocityStep) {
                final double finalAngle = angle;
                final double finalVelocity = velocity;
                addCommands(
                        new InstantCommand(
                                () -> {
                                    RobotContainer.shootOrchestrator.hoodAngleOverride = finalAngle;
                                    RobotContainer.shootOrchestrator.shootVelocityOverride = finalVelocity;
                                },
                                RobotContainer.shooter),
                        new WaitUntilCommand(() ->
                                RobotContainer.feeder.isBottomBeamBroken() || RobotContainer.feeder.isTopBeamBroken()),
                        new WaitCommand(0.5));
            }
        }

        addCommands(new InstantCommand(
                () -> {
                    RobotContainer.shootOrchestrator.setEnableShooting(false);
                },
                RobotContainer.shooter));
    }
}
