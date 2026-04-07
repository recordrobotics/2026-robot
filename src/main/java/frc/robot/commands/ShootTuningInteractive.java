package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.Turret;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

public class ShootTuningInteractive extends SequentialCommandGroup {

    private static final double VELOCITY_MIN = 6.0;
    private static final double VELOCITY_MAX = 18.0;
    private static final double TURRET_MIN_RAD = Units.rotationsToRadians(
            Constants.Turret.ROTATION_MIN_POSITION_MOTOR_ROTATIONS + Turret.MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
    private static final double TURRET_MAX_RAD = Units.rotationsToRadians(
            Constants.Turret.ROTATION_MAX_POSITION_MOTOR_ROTATIONS + Turret.MOTOR_TO_PHYSICAL_OFFSET_ROTATIONS);
    private static final double SWEEP_SECONDS = 2.0;

    public ShootTuningInteractive() {
        BooleanSupplier lockButton = () -> DashboardUI.Overview.getControl().isShooterInvertPressed();

        addRequirements(RobotContainer.shooter);
        addCommands(
                new InstantCommand(
                        () -> {
                            RobotContainer.shootOrchestrator.setEnableShooting(true);
                            RobotContainer.shootOrchestrator.overrideShootAngleVelocity = true;
                        },
                        RobotContainer.shooter),
                new OscillateAndLock(
                        TURRET_MIN_RAD,
                        TURRET_MAX_RAD,
                        SWEEP_SECONDS,
                        v -> {
                            RobotContainer.shootOrchestrator.turretAngleOverride = Optional.of(v);
                            RobotContainer.turret.setTarget(v, 0, 0);
                        },
                        lockButton,
                        "TurretAngle"),
                new WaitCommand(0.3),
                new OscillateAndLock(
                        Constants.Shooter.HOOD_MIN_POSITION_RADIANS,
                        Constants.Shooter.HOOD_MAX_POSITION_RADIANS,
                        SWEEP_SECONDS,
                        v -> RobotContainer.shootOrchestrator.hoodAngleOverride = v,
                        lockButton,
                        "HoodAngle"),
                new WaitCommand(0.3),
                new OscillateAndLock(
                        VELOCITY_MIN,
                        VELOCITY_MAX,
                        SWEEP_SECONDS,
                        v -> RobotContainer.shootOrchestrator.shootVelocityOverride = v,
                        lockButton,
                        "Velocity"),
                new WaitCommand(0.3),
                new WaitUntilCommand(
                        () -> RobotContainer.feeder.isBottomBeamBroken() || RobotContainer.feeder.isTopBeamBroken()),
                new WaitCommand(0.5));
    }

    private static class OscillateAndLock extends Command {

        private final double min;
        private final double max;
        private final double step;
        private final DoubleConsumer applier;
        private final BooleanSupplier button;
        private final String label;

        private double currentValue;
        private int direction;
        private boolean lastButtonState;

        OscillateAndLock(
                double min,
                double max,
                double sweepSeconds,
                DoubleConsumer applier,
                BooleanSupplier button,
                String label) {
            this.min = min;
            this.max = max;
            this.step = (max - min) / (sweepSeconds / 0.02);
            this.applier = applier;
            this.button = button;
            this.label = label;
        }

        @Override
        public void initialize() {
            currentValue = min;
            direction = 1;
            lastButtonState = button.getAsBoolean();
        }

        @Override
        public void execute() {
            currentValue += direction * step;
            if (currentValue >= max) {
                currentValue = max;
                direction = -1;
            } else if (currentValue <= min) {
                currentValue = min;
                direction = 1;
            }
            applier.accept(currentValue);
            SmartDashboard.putNumber("InteractiveTuning/" + label, currentValue);
        }

        @Override
        public boolean isFinished() {
            boolean currentButtonState = button.getAsBoolean();
            boolean risingEdge = currentButtonState && !lastButtonState;
            lastButtonState = currentButtonState;
            return risingEdge;
        }
    }
}
