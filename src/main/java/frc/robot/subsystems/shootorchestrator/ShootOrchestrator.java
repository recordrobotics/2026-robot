package frc.robot.subsystems.shootorchestrator;

import com.pathplanner.lib.util.FlippingUtil;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.dashboard.DashboardUI;
import frc.robot.subsystems.Feeder.FeederState;
import frc.robot.subsystems.Shooter.ShooterState;
import frc.robot.subsystems.Spindexer.SpindexerState;
import frc.robot.subsystems.shootorchestrator.ShotCalculator.ShotCalculation;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.ProjectileSimulationUtils;
import frc.robot.utils.SimpleMath;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.littletonrobotics.junction.Logger;

public class ShootOrchestrator extends ManagedSubsystemBase {
    public static final double ALLIANCE_ZONE_THRESHOLD_X = 4.0; // TODO make correct

    private static final Translation3d BLUE_HUB_POSITION = new Translation3d(4.625594, 4.03463, Units.feetToMeters(6));
    private static final Translation3d RED_HUB_POSITION = new Translation3d(
            FlippingUtil.fieldSizeX - BLUE_HUB_POSITION.getX(),
            FlippingUtil.fieldSizeY - BLUE_HUB_POSITION.getY(),
            BLUE_HUB_POSITION.getZ());
    private static final double HUB_RADIUS_METERS = Units.inchesToMeters(20); // TODO use hexagon?
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

    private static final Translation2d BLUE_TRENCH_HP_SIDE = new Translation2d(4.572, 0.664);
    private static final Translation2d BLUE_TRENCH_DEPOT_SIDE =
            new Translation2d(BLUE_TRENCH_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_TRENCH_HP_SIDE.getY());
    private static final Translation2d RED_TRENCH_HP_SIDE = new Translation2d(
            FlippingUtil.fieldSizeX - BLUE_TRENCH_HP_SIDE.getX(), FlippingUtil.fieldSizeY - BLUE_TRENCH_HP_SIDE.getY());
    private static final Translation2d RED_TRENCH_DEPOT_SIDE = new Translation2d(
            FlippingUtil.fieldSizeX - BLUE_TRENCH_DEPOT_SIDE.getX(),
            FlippingUtil.fieldSizeY - BLUE_TRENCH_DEPOT_SIDE.getY());

    private static final ShotCalculator hubCalculator = new HubRegressionCalculator();
    private static final ShotCalculator passingCalculator = new HubRegressionCalculator();

    private static final double BOTTOM_BEAM_TIME_TO_BALL_HIT = 0.06;
    private static final double TOP_BEAM_TIME_TO_BALL_HIT = 0.04;

    private static final double TRENCH_WIDTH_METERS = 1.361281;
    private static final double TRENCH_OFFSET_METERS = 0.3;

    public enum FeedMode {
        AUTO,
        ALWAYS,
        DISABLED
    }

    public double hoodAngleOverride = Constants.Shooter.HOOD_MAX_POSITION_RADIANS;
    public double shootVelocityOverride = 0;
    public boolean overrideShootAngleVelocity = false;

    Translation3d[] trajectory = new Translation3d[48];

    private FeedMode feedMode = FeedMode.ALWAYS;

    private ShotTarget target;
    private boolean shootingEnabled = false;

    private double lastShotYaw = 0;
    private boolean hasLastShotYaw = false;

    private double timeAtBallHit = 0;
    private double shooterFeedforward = 0;

    public record ShotTarget(Translation3d position, ShotCalculator shotCalculator) {}

    public ShootOrchestrator() {
        // nothing to do
        SmartDashboard.putNumber("SHOTTFED", 1.224808013371447);

        SmartDashboard.putNumber("HOOD_ANGLE", hoodAngleOverride);
        SmartDashboard.putNumber("SHOOT_VELOCITY", shootVelocityOverride);
        SmartDashboard.putBoolean("SHOOT_OVERRIDE", false);

        Logger.recordOutput("TRENCHES", new Translation2d[] {
            BLUE_TRENCH_HP_SIDE, BLUE_TRENCH_DEPOT_SIDE, RED_TRENCH_HP_SIDE, RED_TRENCH_DEPOT_SIDE
        });

        SmartDashboard.putNumber("SHOOT_ANGLE_OFFSET", 0);
    }

    public void setEnableShooting(boolean enable) {
        this.shootingEnabled = enable;
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
        } else {
            setTarget(new ShotTarget(
                    DriverStationUtils.getCurrentAlliance() == Alliance.Blue ? BLUE_HUB_POSITION : RED_HUB_POSITION,
                    hubCalculator));
        }
    }

    private void feedforwardShooter() {
        double f = SmartDashboard.getNumber("SHOTTFED", 1.224808013371447);
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
                shooterFeedforward = f;
            }
        } else {
            shooterFeedforward = 0;
        }
    }

    public boolean isInTrench(Pose2d pose, Translation2d trench) {
        return Math.abs(pose.getY() - trench.getY()) < TRENCH_WIDTH_METERS / 2.0
                && Math.abs(pose.getX() - trench.getX()) < TRENCH_OFFSET_METERS;
    }

    public boolean isInTrench(Pose2d pose) {
        return isInTrench(pose, BLUE_TRENCH_HP_SIDE)
                || isInTrench(pose, BLUE_TRENCH_DEPOT_SIDE)
                || isInTrench(pose, RED_TRENCH_HP_SIDE)
                || isInTrench(pose, RED_TRENCH_DEPOT_SIDE);
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

            double shotYawVelocity = hasLastShotYaw ? (shotYaw - lastShotYaw) / 0.02 : 0;
            lastShotYaw = shotYaw;
            hasLastShotYaw = true;

            if (!RobotContainer.intake.isNearStartPosition()) {
                RobotContainer.turret.setTarget(
                        shotYaw - robotPose.getRotation().getRadians(),
                        shotYawVelocity - RobotContainer.drivetrain.getChassisSpeeds().omegaRadiansPerSecond,
                        -RobotContainer.drivetrain.getChassisAcceleration().omegaRadiansPerSecond);
            } else {
                double turretPos = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations());
                RobotContainer.turret.setTarget(
                        Math.copySign(Constants.Turret.STARTING_POSITION_RADIANS, turretPos), 0, 0);
            }

            Pose2d robotPoseTrench = SimpleMath.integrateChassisSpeeds(
                    RobotContainer.poseSensorFusion.getEstimatedPosition(),
                    RobotContainer.drivetrain.getChassisSpeeds(),
                    0.2);
            Pose3d fuelReleasePoseTrench =
                    new Pose3d(robotPoseTrench).transformBy(new Transform3d(fuelReleaseOffset, Rotation3d.kZero));

            boolean isInTrench = isInTrench(fuelReleasePoseTrench.toPose2d());
            Logger.recordOutput("ShootOrchestrator/IsInTrench", isInTrench);

            if (shootingEnabled) {
                if (!SmartDashboard.getBoolean("SHOOT_OVERRIDE", false)) {
                    double shotPitch = Math.atan2(shotVector.get(2), Math.hypot(shotVector.get(0), shotVector.get(1)));
                    shotPitch += SmartDashboard.getNumber("SHOOT_ANGLE_OFFSET", 0);
                    RobotContainer.shooter.setTargetState(new ShooterState(
                            isInTrench ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS : shotPitch,
                            target.shotCalculator.fuelToFlywheelVelocity(shotVector.norm()),
                            shooterFeedforward));
                } else if (overrideShootAngleVelocity) {
                    RobotContainer.shooter.setTargetState(new ShooterState(
                            isInTrench ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS : hoodAngleOverride,
                            shootVelocityOverride,
                            shooterFeedforward));
                } else {
                    double hoodAngle =
                            SmartDashboard.getNumber("HOOD_ANGLE", Constants.Shooter.HOOD_MAX_POSITION_RADIANS);
                    double flyVel = SmartDashboard.getNumber("SHOOT_VELOCITY", 0);

                    RobotContainer.shooter.setTargetState(new ShooterState(
                            isInTrench ? Constants.Shooter.HOOD_MAX_POSITION_RADIANS : hoodAngle,
                            flyVel,
                            shooterFeedforward));
                }
            } else {
                RobotContainer.shooter.setTargetState(
                        new ShooterState(Constants.Shooter.HOOD_MAX_POSITION_RADIANS, 0, 0));
            }

            boolean onTarget = !isInTrench
                    && RobotContainer.turret.atGoal()
                    && RobotContainer.shooter.isAtTargetState()
                    && !RobotContainer.intake.isNearStartPosition();
            SpindexerState spindexerState = (onTarget && shootingEnabled) ? SpindexerState.ON : SpindexerState.OFF;
            FeederState feederState = (onTarget && shootingEnabled) ? FeederState.ON : FeederState.OFF;

            RobotContainer.spindexer.setState(
                    DashboardUI.Overview.getControl().isUnstuckSpindexerPressed()
                            ? SpindexerState.UNSTUCK
                            : spindexerState);
            RobotContainer.feeder.setState(
                    DashboardUI.Overview.getControl().isUnstuckSpindexerPressed() ? FeederState.UNSTUCK : feederState);

            Logger.recordOutput("ShootOrchestrator/OnTarget", onTarget);
        }

        updateTrajectory(robotPose, fuelReleasePose);

        Logger.recordOutput(
                "ShootOrchestrator/Target",
                target == null ? Pose3d.kZero : new Pose3d(target.position, Rotation3d.kZero));
        Logger.recordOutput("ShootOrchestrator/Trajectory", trajectory);
        Logger.recordOutput("ShootOrchestrator/ShootingEnabled", shootingEnabled);
    }
}
