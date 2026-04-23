package frc.robot.subsystems.shootorchestrator;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Spindexer.SpindexerState;
import frc.robot.subsystems.shootorchestrator.ShotCalculator.ShotCalculation;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.PositionedSubsystem.PositionStatus;
import frc.robot.utils.ProjectileSimulationUtils;
import frc.robot.utils.SimpleMath;
import java.util.Optional;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ShootOrchestrator extends ManagedSubsystemBase {
    public static final double ALLIANCE_ZONE_THRESHOLD_X = 4.0; // TODO make correct

    private static final Translation3d BLUE_HUB_POSITION = new Translation3d(4.625594, 4.03463, Units.feetToMeters(6));
    private static final Translation3d RED_HUB_POSITION = new Translation3d(
            FlippingUtil.fieldSizeX - BLUE_HUB_POSITION.getX(),
            FlippingUtil.fieldSizeY - BLUE_HUB_POSITION.getY(),
            BLUE_HUB_POSITION.getZ());

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

    private static final Translation2d BLUE_TRENCH_HP_SIDE = new Translation2d(4.622, 0.644493);
    private static final Translation2d BLUE_TRENCH_DEPOT_SIDE =
            new Translation2d(BLUE_TRENCH_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_TRENCH_HP_SIDE.getY());
    private static final Translation2d RED_TRENCH_HP_SIDE = new Translation2d(
            FlippingUtil.fieldSizeX - BLUE_TRENCH_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_TRENCH_HP_SIDE.getY());
    private static final Translation2d RED_TRENCH_DEPOT_SIDE = new Translation2d(
            FlippingUtil.fieldSizeX - BLUE_TRENCH_DEPOT_SIDE.getX(),
            FlippingUtil.fieldSizeY - BLUE_TRENCH_DEPOT_SIDE.getY());

    private static final ShotCalculator hubCalculator = new HubRegressionCalculator();
    private static final ShotCalculator passingCalculator = new PassingRegressionCalculator();

    private static final double BOTTOM_BEAM_TIME_TO_BALL_HIT = 0.06;
    private static final double TOP_BEAM_TIME_TO_BALL_HIT = 0.04;

    private static final double TRENCH_WIDTH_METERS = 1.361281;
    private static final double TRENCH_OFFSET_METERS = 0.5;

    private static final ShotCalculation FIXED_SHOT_CALCULATION = hubCalculator.calculateShot(2.944349, 0);
    private static final double FIXED_TURRET_ANGLE_RADIANS = Units.degreesToRadians(90);
    private static final double FIXED_SHOOTER_ANGLE_RADIANS =
            FIXED_SHOT_CALCULATION.shootAngleRadians() - Constants.Shooter.HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS;
    private static final double FIXED_FUEL_VELOCITY = FIXED_SHOT_CALCULATION.fuelVelocityMagnitudeMps();

    private static final LoggedNetworkNumber shotFeedforward = new LoggedNetworkNumber("SHOTFEED", 1.224808013371447);
    private static final LoggedNetworkNumber hoodAngleDashboardOverride =
            new LoggedNetworkNumber("HOOD_ANGLE", Constants.Shooter.HOOD_MAX_POSITION_RADIANS);
    private static final LoggedNetworkNumber shootVelocityDashboardOverride =
            new LoggedNetworkNumber("SHOOT_VELOCITY", 0);
    private static final LoggedNetworkBoolean shootOverride = new LoggedNetworkBoolean("SHOOT_OVERRIDE", false);
    private static final LoggedNetworkNumber shootAngleOffset = new LoggedNetworkNumber("SHOOT_ANGLE_OFFSET", 0);
    private static final LoggedNetworkBoolean limitBallHeight =
            new LoggedNetworkBoolean("Shooter/LimitBallHeight", false);
    private static final LoggedNetworkNumber ballHeightLimit = new LoggedNetworkNumber("Shooter/BallHeightLimit", 0.0);

    public enum FeedMode {
        AUTO,
        ALWAYS,
        DISABLED
    }

    public double hoodAngleOverride = Constants.Shooter.HOOD_MAX_POSITION_RADIANS;
    public Optional<Double> turretAngleOverride = Optional.empty();
    public double shootVelocityOverride = 0;
    public boolean overrideShootAngleVelocity = false;

    Translation3d[] trajectory = new Translation3d[48];

    private FeedMode feedMode = FeedMode.AUTO;

    private ShotTarget target;
    private boolean shootingEnabled = false;
    private boolean fixedMode = false;
    private boolean useFixedShooting = false;

    private double lastShotYaw = 0;
    private boolean hasLastShotYaw = false;

    private double timeAtBallHit = 0;
    private double shooterFeedforward = 0;

    public record ShotTarget(Translation3d position, ShotCalculator shotCalculator) {}

    public ShootOrchestrator() {
        Logger.recordOutput("TRENCHES", new Translation2d[] {
            BLUE_TRENCH_HP_SIDE, BLUE_TRENCH_DEPOT_SIDE, RED_TRENCH_HP_SIDE, RED_TRENCH_DEPOT_SIDE
        });
    }

    public void setEnableShooting(boolean enable) {
        this.shootingEnabled = enable;
    }

    public void setFixedMode(boolean fixed) {
        this.fixedMode = fixed;
        if (!fixed) {
            useFixedShooting = false;
        }
    }

    public void setFeedMode(FeedMode mode) {
        this.feedMode = mode;
    }

    public final void setTarget(ShotTarget target) {
        this.target = target;
    }

    public ShotTarget getTarget() {
        return target;
    }

    private void updateTrajectory(Pose2d robotPose, Pose3d fuelReleasePose) {
        if (target == null) {
            for (int i = 0; i < trajectory.length; i++) {
                trajectory[i] = fuelReleasePose.getTranslation();
            }
            return;
        }

        Translation3d velocity = new Translation3d(
                        target.shotCalculator.flywheelToFuelVelocity(RobotContainer.shooter.getFlywheelVelocityMps()),
                        0,
                        0)
                .rotateBy(new Rotation3d(
                        0,
                        -RobotContainer.shooter.getHoodAngle(),
                        Units.rotationsToRadians(RobotContainer.turret.getPositionRotations())));

        Translation2d velocity2d = ProjectileSimulationUtils.calculateInitialProjectileVelocityMPS(
                fuelReleasePose.toPose2d().getTranslation(),
                RobotContainer.drivetrain.getChassisSpeeds(),
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
            if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) {
                setTarget(new ShotTarget(
                        RobotContainer.poseSensorFusion.getEstimatedPosition().getY() < FlippingUtil.fieldSizeY / 2
                                ? BLUE_PASSING_TARGET_HP_SIDE
                                : BLUE_PASSING_TARGET_DEPOT_SIDE,
                        passingCalculator));
            } else {
                setTarget(new ShotTarget(
                        RobotContainer.poseSensorFusion.getEstimatedPosition().getY() > FlippingUtil.fieldSizeY / 2
                                ? RED_PASSING_TARGET_HP_SIDE
                                : RED_PASSING_TARGET_DEPOT_SIDE,
                        passingCalculator));
            }
            useFixedShooting = false;
        } else {
            setTarget(new ShotTarget(
                    DriverStationUtils.getCurrentAlliance() == Alliance.Blue ? BLUE_HUB_POSITION : RED_HUB_POSITION,
                    hubCalculator));
            useFixedShooting = fixedMode;
        }
    }

    private void feedforwardShooter() {
        double currentTime = Timer.getTimestamp();

        if (RobotContainer.feeder.isBottomBeamBroken()) {
            timeAtBallHit = currentTime + BOTTOM_BEAM_TIME_TO_BALL_HIT;
        }

        if (RobotContainer.feeder.isTopBeamBroken()) {
            timeAtBallHit = currentTime + TOP_BEAM_TIME_TO_BALL_HIT;
        }

        if (timeAtBallHit > currentTime) {
            double timeLeft = timeAtBallHit - currentTime;
            if (timeLeft <= 0.06 && timeLeft >= 0.02) {
                shooterFeedforward = shotFeedforward.get();
            }
        } else {
            shooterFeedforward = 0;
        }
    }

    public boolean isInTrench(Translation2d start, Translation2d end, Translation2d trench) {
        double minX = trench.getX() - TRENCH_OFFSET_METERS;
        double maxX = trench.getX() + TRENCH_OFFSET_METERS;
        double minY = trench.getY() - TRENCH_WIDTH_METERS / 2.0;
        double maxY = trench.getY() + TRENCH_WIDTH_METERS / 2.0;

        boolean startInside =
                start.getX() >= minX && start.getX() <= maxX && start.getY() >= minY && start.getY() <= maxY;
        boolean endInside = end.getX() >= minX && end.getX() <= maxX && end.getY() >= minY && end.getY() <= maxY;
        if (startInside || endInside) {
            return true;
        }

        // Slab intersection (segment param t in [0, 1])
        double tMin = 0.0;
        double tMax = 1.0;
        final double eps = 1e-9;

        // X slab
        double dx = end.getX() - start.getX();
        if (Math.abs(dx) < eps) {
            if (start.getX() < minX || start.getX() > maxX) {
                return false;
            }
        } else {
            double tx1 = (minX - start.getX()) / dx;
            double tx2 = (maxX - start.getX()) / dx;
            if (tx1 > tx2) {
                double temp = tx1;
                tx1 = tx2;
                tx2 = temp;
            }
            tMin = Math.max(tMin, tx1);
            tMax = Math.min(tMax, tx2);
            if (tMin > tMax) {
                return false;
            }
        }

        // Y slab
        double dy = end.getY() - start.getY();
        if (Math.abs(dy) < eps) {
            if (start.getY() < minY || start.getY() > maxY) {
                return false;
            }
        } else {
            double ty1 = (minY - start.getY()) / dy;
            double ty2 = (maxY - start.getY()) / dy;
            if (ty1 > ty2) {
                double temp = ty1;
                ty1 = ty2;
                ty2 = temp;
            }
            tMin = Math.max(tMin, ty1);
            tMax = Math.min(tMax, ty2);
            if (tMin > tMax) {
                return false;
            }
        }

        return tMax >= 0.0 && tMin <= 1.0;
    }

    public boolean isInTrench(Translation2d start, Translation2d end) {
        return isInTrench(start, end, BLUE_TRENCH_HP_SIDE)
                || isInTrench(start, end, BLUE_TRENCH_DEPOT_SIDE)
                || isInTrench(start, end, RED_TRENCH_HP_SIDE)
                || isInTrench(start, end, RED_TRENCH_DEPOT_SIDE);
    }

    @Override
    public void periodicManaged() {
        setAutomatedTarget();
        feedforwardShooter();

        Pose2d robotPose = RobotContainer.poseSensorFusion.getEstimatedPosition();
        Translation3d fuelReleaseOffset = RobotContainer.model.fuelManager.getShooterFuelReleasePosition();
        Pose3d fuelReleasePose =
                new Pose3d(robotPose).transformBy(new Transform3d(fuelReleaseOffset, Rotation3d.kZero));

        ChassisSpeeds robotSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                RobotContainer.drivetrain.getChassisSpeeds(), robotPose.getRotation());

        double omega = robotSpeeds.omegaRadiansPerSecond;
        double releaseVx = robotSpeeds.vxMetersPerSecond - omega * fuelReleaseOffset.getY();
        double releaseVy = robotSpeeds.vyMetersPerSecond + omega * fuelReleaseOffset.getX();
        Transform2d fuelReleaseVelocity = new Transform2d(releaseVx, releaseVy, new Rotation2d(omega));
        Vector<N2> fuelReleaseVelocityVector =
                fuelReleaseVelocity.getTranslation().toVector();

        if (target != null) {
            Translation2d relativeTarget = target.position
                    .toTranslation2d()
                    .minus(fuelReleasePose.getTranslation().toTranslation2d());

            double distanceToTarget = relativeTarget.getNorm();
            Vector<N2> normTargetVector = relativeTarget.toVector().div(distanceToTarget);
            double radialVelocity = fuelReleaseVelocityVector.dot(normTargetVector);

            Vector<N2> tangentialVelocity = fuelReleaseVelocityVector.minus(normTargetVector.times(radialVelocity));

            ShotCalculation shotCalculation = target.shotCalculator.calculateShot(distanceToTarget, radialVelocity);

            double targetYaw = Math.atan2(relativeTarget.getY(), relativeTarget.getX());
            double targetPitch = shotCalculation.shootAngleRadians();
            double targetSpeed = shotCalculation.fuelVelocityMagnitudeMps();

            Vector<N3> shotVector = new Translation3d(targetSpeed, 0.0, 0.0)
                    .rotateBy(new Rotation3d(0.0, -targetPitch, targetYaw))
                    .toVector()
                    .minus(new Translation3d(new Translation2d(tangentialVelocity)).toVector());

            double shotYaw = Math.atan2(shotVector.get(1), shotVector.get(0));

            double errorBound = Math.PI;
            double shotYawVelocity =
                    hasLastShotYaw ? MathUtil.inputModulus(shotYaw - lastShotYaw, -errorBound, errorBound) / 0.02 : 0;
            lastShotYaw = shotYaw;
            hasLastShotYaw = true;

            if (!RobotContainer.intake.isNearStartPosition()) {
                if (useFixedShooting) {
                    RobotContainer.turret.setTarget(FIXED_TURRET_ANGLE_RADIANS, 0, 0);
                } else if (turretAngleOverride.isPresent()) {
                    RobotContainer.turret.setTarget(turretAngleOverride.get(), 0, 0);
                } else {
                    RobotContainer.turret.setTarget(
                            shotYaw - robotPose.getRotation().getRadians(),
                            shotYawVelocity - RobotContainer.drivetrain.getChassisSpeeds().omegaRadiansPerSecond,
                            -RobotContainer.drivetrain.getChassisAcceleration().omegaRadiansPerSecond);
                }
            } else {
                double turretPos = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations());
                RobotContainer.turret.setTarget(
                        Math.copySign(Constants.Turret.STARTING_POSITION_RADIANS, turretPos), 0, 0);
            }

            double timeUntilHoodDown =
                    RobotContainer.shooter.getTimeUntilHoodAt(Constants.Shooter.HOOD_MAX_POSITION_RADIANS)
                            + 0.1 /* latency compensation */;

            Logger.recordOutput("ShootOrchestrator/HoodTime", timeUntilHoodDown);

            Pose2d robotPoseTrench = SimpleMath.integrateChassisSpeeds(
                    RobotContainer.poseSensorFusion.getEstimatedPosition(),
                    RobotContainer.drivetrain.getChassisSpeeds(),
                    timeUntilHoodDown);
            Translation3d hoodPosition = RobotContainer.model.fuelManager.getShooterHoodPosition();
            Pose3d hoodPoseTrench =
                    new Pose3d(robotPoseTrench).transformBy(new Transform3d(hoodPosition, Rotation3d.kZero));
            Pose3d hoodPoseCurrent = new Pose3d(robotPose).transformBy(new Transform3d(hoodPosition, Rotation3d.kZero));

            boolean isInTrench = isInTrench(
                    hoodPoseCurrent.getTranslation().toTranslation2d(),
                    hoodPoseTrench.getTranslation().toTranslation2d());
            Logger.recordOutput("ShootOrchestrator/IsInTrench", isInTrench);

            Logger.recordOutput("ShootOrchestrator/Trench", hoodPoseTrench);

            if (shootingEnabled) {
                if (!shootOverride.get()) {
                    double shotPitch = Math.atan2(shotVector.get(2), Math.hypot(shotVector.get(0), shotVector.get(1)))
                            - Constants.Shooter.HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS;
                    shotPitch += shootAngleOffset.get();
                    RobotContainer.shooter.setTargetState(new ShooterState(
                            isInTrench
                                    ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS
                                    : (useFixedShooting ? FIXED_SHOOTER_ANGLE_RADIANS : shotPitch),
                            target.shotCalculator.fuelToFlywheelVelocity(
                                    useFixedShooting ? FIXED_FUEL_VELOCITY : shotVector.norm()),
                            shooterFeedforward));
                } else if (overrideShootAngleVelocity) {
                    RobotContainer.shooter.setTargetState(new ShooterState(
                            isInTrench ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS : hoodAngleOverride,
                            shootVelocityOverride,
                            shooterFeedforward));
                } else {
                    double hoodAngle = RobotContainer.shooter.getHoodAngle();
                    double angleDeg =
                            Units.radiansToDegrees(hoodAngle + Constants.Shooter.HOOD_FUEL_EXIT_ANGLE_OFFSET_RADIANS);
                    double currentFlywheelVelocity = target.shotCalculator.fuelToFlywheelVelocity(
                            useFixedShooting ? FIXED_FUEL_VELOCITY : shotVector.norm());
                    double targetFlywheelVelocity = currentFlywheelVelocity;
                    if (limitBallHeight.get()) {
                        targetFlywheelVelocity = Math.min(
                                currentFlywheelVelocity,
                                target.shotCalculator.fuelToFlywheelVelocity(
                                        calculateBallVelocityForHeight(angleDeg, ballHeightLimit.get())));
                    }
                    RobotContainer.shooter.setTargetState(new ShooterState(
                            (overrideShootAngleVelocity ? hoodAngleOverride : hoodAngleDashboardOverride.get()),
                            targetFlywheelVelocity,
                            shooterFeedforward));
                }
            } else {
                RobotContainer.shooter.setTargetState(
                        new ShooterState(Constants.Shooter.HOOD_MAX_POSITION_RADIANS, 0, 0));
            }

            boolean onTarget = !isInTrench
                    && RobotContainer.turret.atGoal()
                    && RobotContainer.shooter.isAtTargetState()
                    && !RobotContainer.intake.isNearStartPosition()
                    && RobotContainer.turret.getPositionStatus() == PositionStatus.KNOWN
                    && RobotContainer.shooter.getPositionStatus() == PositionStatus.KNOWN;
            SpindexerState spindexerState =
                    switch (feedMode) {
                        case AUTO -> (onTarget && shootingEnabled) ? SpindexerState.ON : SpindexerState.OFF;
                        case ALWAYS -> shootingEnabled ? SpindexerState.ON : SpindexerState.OFF;
                        case DISABLED -> SpindexerState.OFF;
                    };
            FeederState feederState =
                    switch (feedMode) {
                        case AUTO -> (onTarget && shootingEnabled) ? FeederState.ON : FeederState.OFF;
                        case ALWAYS -> shootingEnabled ? FeederState.ON : FeederState.OFF;
                        case DISABLED -> FeederState.OFF;
                    };

            RobotContainer.spindexer.setState(
                    RobotContainer.getControl().isUnstuckSpindexerPressed() ? SpindexerState.UNSTUCK : spindexerState);
            RobotContainer.feeder.setState(
                    RobotContainer.getControl().isUnstuckSpindexerPressed() ? FeederState.UNSTUCK : feederState);

            Logger.recordOutput("ShootOrchestrator/OnTarget", onTarget);
        }

        updateTrajectory(robotPose, fuelReleasePose);

        Logger.recordOutput(
                "ShootOrchestrator/Target",
                target == null ? Pose3d.kZero : new Pose3d(target.position, Rotation3d.kZero));
        Logger.recordOutput("ShootOrchestrator/Trajectory", trajectory);
        Logger.recordOutput("ShootOrchestrator/ShootingEnabled", shootingEnabled);
    }

    /**
     * Calculate the required ball velocity in meters per second based on the hood angle and target
     * height.
     *
     * @param angleDegrees the hood angle in degrees
     * @param height the target height in meters
     * @return the required ball velocity in meters per second
     */
    private static double calculateBallVelocityForHeight(double angleDegrees, double height) {
        return 7;
    }
}
