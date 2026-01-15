package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimbBurst extends SequentialCommandGroup {
    public ClimbBurst() {
        addRequirements(RobotContainer.climber);
        addCommands(
                new InstantCommand(() -> RobotContainer.lights
                        .stateVisualizer
                        .runPattern(Constants.Lights.cagePattern)
                        .schedule()),
                new InstantCommand(() -> RobotContainer.climber.setBurst(Constants.Climber.CLIMB_BURST_ROTATIONS)),
                new WaitUntilCommand(() -> RobotContainer.climber.atGoal()),
                new InstantCommand(() -> RobotContainer.lights
                        .elevator
                        .runPattern(Constants.Lights.FLASHING_GREEN)
                        .alongWith(RobotContainer.lights.algaeGrabber.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.coralIntake.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.coralShooter.runPattern(Constants.Lights.FLASHING_GREEN))
                        .alongWith(RobotContainer.lights.stateVisualizer.runPattern(Constants.Lights.PULSATING_GREEN))
                        .withTimeout(Constants.Lights.SUCCESS_FLASH_TIME)
                        .schedule()));
    }
}
