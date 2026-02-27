package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Spindexer.SpindexerState;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.ProjectileSimulationUtils;
import java.util.Optional;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

public class ShootOrchestrator extends ManagedSubsystemBase {

    private static final double HUB_X = 4.625594;
    private static final double HUB_Y = 4.03463;
    private static final double HUB_Z = Units.feetToMeters(6);
    private static final double HUB_RADIUS = Units.inchesToMeters(20);

    private Translation3d target;
    private boolean shootingEnabled = false;

    public ShootOrchestrator() {
        setTarget(new Translation3d(HUB_X, HUB_Y, HUB_Z));
    }

    public void setEnableShooting(boolean enable) {
        this.shootingEnabled = enable;
    }

    public final void setTarget(Translation3d target) {
        this.target = target;
    }

    public Translation3d getTarget() {
        return target;
    }

    public Optional<ShooterState> getShooterState() {
        Pose2d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        Pose3d fuelReleasePose = new Pose3d(robotPose)
                .transformBy(new Transform3d(
                        RobotContainer.model.fuelManager.getShooterFuelReleasePosition(), Rotation3d.kZero));
        Translation2d relativeTarget =
                target.toTranslation2d().minus(fuelReleasePose.getTranslation().toTranslation2d());

        double groundDistance = relativeTarget.getNorm();
        double targetHeight = target.getZ();
        double maxHeight = Units.feetToMeters(8);
        double height = maxHeight - fuelReleasePose.getTranslation().getZ();

        double G = GamePieceProjectile.GRAVITY;

        double timeOfFlight = (Math.sqrt(2 * G * height) + Math.sqrt(2 * G * (maxHeight - targetHeight))) / G;
        double velocity = Math.sqrt((groundDistance * groundDistance) / (timeOfFlight * timeOfFlight) + 2 * G * height);
        double angle = Math.atan(timeOfFlight * Math.sqrt(2 * G * height) / groundDistance);

        if (Double.isNaN(velocity) || Double.isNaN(angle)) {
            return Optional.empty();
        }
        return Optional.of(new ShooterState(angle, shootingEnabled ? shooterMPSFromFuelVelocity(velocity) : 0));
    }

    public static double fuelVelocityFromShooterMPS(double shooterMPS) {
        return shooterMPS * 0.8;
    }

    public static double shooterMPSFromFuelVelocity(double fuelVelocity) {
        return fuelVelocity / 0.8;
    }

    Translation3d[] trajectory = new Translation3d[48];

    private void updateTrajectory(Pose2d robotPose, Pose3d fuelReleasePose) {
        Translation3d velocity = new Translation3d(
                        fuelVelocityFromShooterMPS(RobotContainer.shooter.getFlywheelVelocityMps()), 0, 0)
                .rotateBy(new Rotation3d(
                        0,
                        -RobotContainer.shooter.getHoodAngle(),
                        Units.rotationsToRadians(RobotContainer.turret.getPositionRotations())));

        Translation2d velocity2d = ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                fuelReleasePose.toPose2d().getTranslation(),
                RobotContainer.drivetrain.getSwerveDriveSimulation().getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                robotPose.getRotation(),
                velocity.toTranslation2d());
        velocity = new Translation3d(velocity2d.getX(), velocity2d.getY(), velocity.getZ());

        for (int i = 0; i < trajectory.length; i++) {
            double dt = 0.02;
            trajectory[i] = fuelReleasePose.getTranslation();
            for (int j = 0; j < 3; j++) {
                fuelReleasePose = new Pose3d(
                        fuelReleasePose.getTranslation().plus(velocity.times(dt)), fuelReleasePose.getRotation());
                velocity = velocity.plus(new Translation3d(0, 0, -GamePieceProjectile.GRAVITY).times(dt));
            }
        }
    }

    public void periodicManaged() {
        Pose2d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        Pose3d fuelReleasePose = new Pose3d(robotPose)
                .transformBy(new Transform3d(
                        RobotContainer.model.fuelManager.getShooterFuelReleasePosition(), Rotation3d.kZero));

        if (target != null) {
            Translation2d relativeTarget = target.toTranslation2d()
                    .minus(fuelReleasePose.getTranslation().toTranslation2d());
            Rotation2d fieldTargetRotation = new Rotation2d(Math.atan2(relativeTarget.getY(), relativeTarget.getX()));
            Rotation2d turretTargetRotation = fieldTargetRotation.minus(robotPose.getRotation());
            RobotContainer.turret.setTarget(turretTargetRotation.getRadians());
            RobotContainer.shooter.setTargetState(getShooterState().orElse(new ShooterState(0, 0)));

            Translation3d velocity = new Translation3d(
                            fuelVelocityFromShooterMPS(RobotContainer.shooter.getFlywheelVelocityMps()), 0, 0)
                    .rotateBy(new Rotation3d(
                            0,
                            -RobotContainer.shooter.getHoodAngle(),
                            Units.rotationsToRadians(RobotContainer.turret.getPositionRotations())));

            Translation2d velocity2d = ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                    fuelReleasePose.toPose2d().getTranslation(),
                    RobotContainer.drivetrain
                            .getSwerveDriveSimulation()
                            .getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    robotPose.getRotation(),
                    velocity.toTranslation2d());
            velocity = new Translation3d(velocity2d.getX(), velocity2d.getY(), velocity.getZ());

            Translation3d stepPose = fuelReleasePose.getTranslation();
            double dt = 0.02;
            int maxSteps = 500;
            int steps = 0;
            boolean movedUp = false;
            while ((!movedUp || stepPose.getZ() > target.getZ()) && stepPose.getZ() > 0 && steps < maxSteps) {
                stepPose = stepPose.plus(velocity.times(dt));
                velocity = velocity.plus(new Translation3d(0, 0, -GamePieceProjectile.GRAVITY).times(dt));
                if (stepPose.getZ() > target.getZ()) {
                    movedUp = true;
                }
                steps++;
            }

            double distanceFromTarget = stepPose.toTranslation2d().getDistance(target.toTranslation2d());
            Logger.recordOutput("ShootOrchestrator/DistanceFromTarget", distanceFromTarget);

            boolean onTarget = distanceFromTarget < HUB_RADIUS;
            RobotContainer.spindexer.setState((onTarget && shootingEnabled) ? SpindexerState.ON : SpindexerState.OFF);
            RobotContainer.feeder.setState((onTarget && shootingEnabled) ? FeederState.ON : FeederState.OFF);
        }

        updateTrajectory(robotPose, fuelReleasePose);

        Logger.recordOutput("ShootOrchestrator/Target", new Pose3d(target, Rotation3d.kZero));
        Logger.recordOutput("ShootOrchestrator/Trajectory", trajectory);
        Logger.recordOutput("ShootOrchestrator/ShootingEnabled", shootingEnabled);
    }
}
