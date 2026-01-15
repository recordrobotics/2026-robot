package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorHeight;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElevatorArm;

public class ElevatorMove extends Command {

    private static final double STALL_VELOCITY_THRESHOLD = 0.03;
    private static final double STALL_TIME_THRESHOLD = 0.1;

    private ElevatorHeight targetHeight;
    private double lastMoveTime = 0;
    private double initialElevatorHeight;
    private double initialArmAngle;

    private final double elevatorHeightThreshold;
    private final double elevatorVelocityThreshold;
    private final double armAngleThreshold;
    private final double armVelocityThreshold;

    public ElevatorMove(ElevatorHeight targetHeight) {
        this(
                targetHeight,
                Constants.Elevator.AT_GOAL_POSITION_TOLERANCE,
                Constants.Elevator.AT_GOAL_VELOCITY_TOLERANCE,
                ElevatorArm.POSITION_TOLERANCE,
                ElevatorArm.VELOCITY_TOLERANCE);
    }

    public ElevatorMove(
            ElevatorHeight targetHeight,
            double elevatorHeightThreshold,
            double elevatorVelocityThreshold,
            double armAngleThreshold,
            double armVelocityThreshold) {
        this.targetHeight = targetHeight;
        this.elevatorHeightThreshold = elevatorHeightThreshold;
        this.armAngleThreshold = armAngleThreshold;
        this.elevatorVelocityThreshold = elevatorVelocityThreshold;
        this.armVelocityThreshold = armVelocityThreshold;
        addRequirements(RobotContainer.elevator, RobotContainer.elevatorArm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastMoveTime = Timer.getTimestamp();

        initialElevatorHeight = RobotContainer.elevator.getCurrentHeight();
        initialArmAngle = RobotContainer.elevatorArm.getArmAngle();

        RobotContainer.elevator.moveTo(targetHeight);
        RobotContainer.elevatorArm.set(targetHeight.getArmAngle());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.elevator.moveTo(targetHeight);
        RobotContainer.elevatorArm.set(targetHeight.getArmAngle());

        if (Math.abs(RobotContainer.elevator.getCurrentVelocity()) >= STALL_VELOCITY_THRESHOLD
                || RobotContainer.elevator.atGoal(elevatorHeightThreshold, elevatorVelocityThreshold)) {
            lastMoveTime = Timer.getTimestamp();
        } else {
            // If the elevator is not moving, check if it has been stationary for too long
            double currentTime = Timer.getTimestamp();
            if (currentTime - lastMoveTime > STALL_TIME_THRESHOLD) { // elevator stalled
                // Reset the elevator and arm to their initial positions
                RobotContainer.elevator.set(initialElevatorHeight);
                RobotContainer.elevatorArm.set(initialArmAngle);
                cancel();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        /* nothing to do on end */
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return RobotContainer.elevator.atGoal(elevatorHeightThreshold, elevatorVelocityThreshold)
                && RobotContainer.elevatorArm.atGoal(armAngleThreshold, armVelocityThreshold);
    }
}
