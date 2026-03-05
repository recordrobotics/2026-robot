package frc.robot.subsystems.shootorchestrator;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Spindexer.SpindexerState;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.ProjectileSimulationUtils;
import java.util.Optional;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

public class ShootOrchestrator extends ManagedSubsystemBase {
    public static final double ALLIANCE_ZONE_THRESHOLD_X = 4.0; // TODO make correct

    private static final Translation3d BLUE_HUB_POSITION = new Translation3d(4.625594, 4.03463, Units.feetToMeters(6));
    private static final Translation3d RED_HUB_POSITION = new Translation3d(
            FlippingUtil.fieldSizeX - BLUE_HUB_POSITION.getX(),
            FlippingUtil.fieldSizeY - BLUE_HUB_POSITION.getY(),
            BLUE_HUB_POSITION.getZ());
    private static final double HUB_RADIUS_METERS = Units.inchesToMeters(20);
    private static final Translation3d BLUE_PASSING_TARGET_HP_SIDE = new Translation3d(2.752, 1.664, 0.0);
    private static final Translation3d BLUE_PASSING_TARGET_DEPOT_SIDE = new Translation3d(
            BLUE_PASSING_TARGET_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_PASSING_TARGET_HP_SIDE.getY(), 0.0);
    private static final Translation3d RED_PASSING_TARGET_HP_SIDE = new Translation3d(
            FlippingUtil.fieldSizeX - BLUE_PASSING_TARGET_HP_SIDE.getX(),
            FlippingUtil.fieldSizeY - BLUE_PASSING_TARGET_HP_SIDE.getY(),
            0.0);
    private static final Translation3d RED_PASSING_TARGET_DEPOT_SIDE = new Translation3d(
            FlippingUtil.fieldSizeX - BLUE_PASSING_TARGET_DEPOT_SIDE.getX(),
            FlippingUtil.fieldSizeY - BLUE_PASSING_TARGET_DEPOT_SIDE.getY(),
            0.0);
    private static final double PASSING_ACCEPTABLE_RADIUS_METERS = BLUE_PASSING_TARGET_HP_SIDE.getY();

    public enum FeedMode {
        AUTO,
        ALWAYS,
        DISABLED
    }

    Translation3d[] trajectory = new Translation3d[48];

    private FeedMode feedMode = FeedMode.ALWAYS;

    private boolean aimingAtHub = false;
    private Translation3d target;
    private boolean shootingEnabled = false;

    public ShootOrchestrator() {
        // nothing to do
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

        double timeOfFlight = (Math.sqrt(2 * GamePieceProjectile.GRAVITY * height)
                        + Math.sqrt(2 * GamePieceProjectile.GRAVITY * (maxHeight - targetHeight)))
                / GamePieceProjectile.GRAVITY;
        double velocity = Math.sqrt((groundDistance * groundDistance) / (timeOfFlight * timeOfFlight)
                + 2 * GamePieceProjectile.GRAVITY * height);
        double angle = Math.atan(timeOfFlight * Math.sqrt(2 * GamePieceProjectile.GRAVITY * height) / groundDistance);

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

    private void updateTrajectory(Pose2d robotPose, Pose3d fuelReleasePose) {
        Translation3d velocity = new Translation3d(
                        fuelVelocityFromShooterMPS(RobotContainer.shooter.getFlywheelVelocityMps()), 0, 0)
                .rotateBy(new Rotation3d(
                        0,
                        -RobotContainer.shooter.getHoodAngle(),
                        Units.rotationsToRadians(RobotContainer.turret.getPositionRotations())));

        Translation2d velocity2d = ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                fuelReleasePose.toPose2d().getTranslation(),
                new ChassisSpeeds(),
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

    public static boolean isInAllianceZone() {
        return DriverStationUtils.getCurrentAlliance() == Alliance.Blue
                ? RobotContainer.poseSensorFusion.getEstimatedPosition().getX() < ALLIANCE_ZONE_THRESHOLD_X
                : RobotContainer.poseSensorFusion.getEstimatedPosition().getX()
                        > FlippingUtil.fieldSizeX - ALLIANCE_ZONE_THRESHOLD_X;
    }

    private void setAutomatedTarget() {
        if (!isInAllianceZone()) { // return closest passing target based on alliance and
            // position on field
            aimingAtHub = false;
            if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) {
                setTarget(
                        RobotContainer.poseSensorFusion.getEstimatedPosition().getY() < FlippingUtil.fieldSizeY / 2
                                ? BLUE_PASSING_TARGET_HP_SIDE
                                : BLUE_PASSING_TARGET_DEPOT_SIDE);
            } else {
                setTarget(
                        RobotContainer.poseSensorFusion.getEstimatedPosition().getY() > FlippingUtil.fieldSizeY / 2
                                ? RED_PASSING_TARGET_HP_SIDE
                                : RED_PASSING_TARGET_DEPOT_SIDE);
            }
        } else {
            setTarget(DriverStationUtils.getCurrentAlliance() == Alliance.Blue ? BLUE_HUB_POSITION : RED_HUB_POSITION);
            aimingAtHub = true;
        }
    }

    @Override
    public void periodicManaged() {
        setAutomatedTarget();

        Pose2d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        Pose3d fuelReleasePose = new Pose3d(robotPose)
                .transformBy(new Transform3d(
                        RobotContainer.model.fuelManager.getShooterFuelReleasePosition(), Rotation3d.kZero));

        if (target != null) {
            Translation2d relativeTarget = target.toTranslation2d()
                    .minus(fuelReleasePose.getTranslation().toTranslation2d());
            Rotation2d fieldTargetRotation = new Rotation2d(Math.atan2(relativeTarget.getY(), relativeTarget.getX()));
            Rotation2d turretTargetRotation = fieldTargetRotation.minus(robotPose.getRotation());

            Translation3d velocity = new Translation3d(
                            fuelVelocityFromShooterMPS(RobotContainer.shooter.getFlywheelVelocityMps()), 0, 0)
                    .rotateBy(new Rotation3d(
                            0,
                            -RobotContainer.shooter.getHoodAngle(),
                            Units.rotationsToRadians(RobotContainer.turret.getPositionRotations())));

            Translation2d velocity2d = ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                    fuelReleasePose.toPose2d().getTranslation(),
                    ChassisSpeeds.fromRobotRelativeSpeeds(
                            RobotContainer.drivetrain.getChassisSpeeds(), robotPose.getRotation()),
                    robotPose.getRotation(),
                    velocity.toTranslation2d());
            velocity = new Translation3d(velocity2d.getX(), velocity2d.getY(), velocity.getZ());

            RobotContainer.turret.setTarget(
                    turretTargetRotation.getRadians(),
                    -RobotContainer.drivetrain.getChassisSpeeds().omegaRadiansPerSecond,
                    -RobotContainer.drivetrain.getChassisAcceleration().omegaRadiansPerSecond);
            RobotContainer.shooter.setTargetState(getShooterState().orElse(new ShooterState(0, 0)));

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

            boolean onTarget = true;
            // distanceFromTarget < (aimingAtHub ? HUB_RADIUS_METERS : PASSING_ACCEPTABLE_RADIUS_METERS);
            RobotContainer.spindexer.setState((onTarget && shootingEnabled) ? SpindexerState.ON : SpindexerState.OFF);
            RobotContainer.feeder.setState((onTarget && shootingEnabled) ? FeederState.ON : FeederState.OFF);
        }

        updateTrajectory(robotPose, fuelReleasePose);

        Logger.recordOutput("ShootOrchestrator/Target", new Pose3d(target, Rotation3d.kZero));
        Logger.recordOutput("ShootOrchestrator/Trajectory", trajectory);
        Logger.recordOutput("ShootOrchestrator/ShootingEnabled", shootingEnabled);
    }
}
