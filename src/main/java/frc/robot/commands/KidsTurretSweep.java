package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


import java.util.Optional;

public class KidsTurretSweep extends Command {

    private static final double SWEEP_HALF_RANGE = Units.degreesToRadians(45);
    private static final double SWEEP_STEP = SWEEP_HALF_RANGE * 2 / (2.0 / 0.02); // full ±45° in 2 seconds

    private double sweepPosition;
    private int sweepDirection;

    public KidsTurretSweep() {
        addRequirements(RobotContainer.shooter);
    }

    /**
     * Returns the command chain to use as the scheduled command.
     * .repeatedly() restarts only on natural finish (never, since isFinished is always false).
     * .finallyDo() runs exactly once when the whole chain is interrupted externally.
     */
    public static Command createCommand() {
        return new KidsTurretSweep()
                .repeatedly()
                .finallyDo(() -> {
                    SmartDashboard.putBoolean("TurretSweep", false);
                })
                ;
    }

    @Override
    public void initialize() {
        sweepPosition = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations()); // Start from current angle to avoid jumps
        sweepDirection = 1;
    }

    @Override
    public void execute() {

            sweepPosition += sweepDirection * SWEEP_STEP;
            if (sweepPosition >= SWEEP_HALF_RANGE) {
                sweepPosition = SWEEP_HALF_RANGE;
                sweepDirection = -1;
            } else if (sweepPosition <= -SWEEP_HALF_RANGE) {
                sweepPosition = -SWEEP_HALF_RANGE;
                sweepDirection = 1;
            }

        RobotContainer.shootOrchestrator.turretAngleOverride = Optional.of(sweepPosition);
        SmartDashboard.putNumber("KidsTurretSweep/TurretAngle", sweepPosition);
    }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     RobotContainer.shootOrchestrator.setEnableShooting(false);
    //     RobotContainer.shootOrchestrator.overrideShootAngleVelocity = false;
    //     RobotContainer.shootOrchestrator.turretAngleOverride = Optional.empty();
    // }
}
