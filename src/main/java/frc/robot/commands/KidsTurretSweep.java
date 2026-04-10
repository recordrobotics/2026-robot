package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import java.util.Optional;

public class KidsTurretSweep extends SequentialCommandGroup {

    private static final double SWEEP_HALF_RANGE = Units.degreesToRadians(45);
    private static final double SWEEP_STEP = SWEEP_HALF_RANGE * 2 / (2.0 / 0.02); // full ±45° in 2 seconds

    public static boolean kidsShootActive = false;
    private double sweepPos;
    private int sweepDir;

    public KidsTurretSweep() {
        addRequirements(RobotContainer.shooter);
        sweepPos = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations());
        addCommands(new RunCommand(
                        () -> {
                            sweepPos += sweepDir * SWEEP_STEP;
                            if (sweepPos >= SWEEP_HALF_RANGE) {
                                sweepPos = SWEEP_HALF_RANGE;
                                sweepDir = -1;
                            } else if (sweepPos <= -SWEEP_HALF_RANGE) {
                                sweepPos = -SWEEP_HALF_RANGE;
                                sweepDir = 1;
                            }
                            RobotContainer.shootOrchestrator.turretAngleOverride = Optional.of(sweepPos);
                            SmartDashboard.putNumber("KidsTurretSweep/TurretAngle", sweepPos);
                        },
                        RobotContainer.shooter)
                .until(() -> RobotContainer.getControl().getKidShoot()));

        addCommands(new KidsShoot());
    }

    public static Command createCommand() {
        return new KidsTurretSweep().finallyDo(() -> RobotContainer.turretSweepEnabled.set(false));
    }
}
