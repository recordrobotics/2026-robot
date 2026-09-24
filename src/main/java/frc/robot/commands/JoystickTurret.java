package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.subsystems.shootorchestrator.ShootOrchestrator.FeedMode;
import java.util.Optional;

public class JoystickTurret extends Command {

    private static final double KID_FLYWHEEL_MIN_VELOCITY = 0;
    private static final double KID_FLYWHEEL_MAX_VELOCITY = 28;

    private double turretAngle;
    private double hoodAngle;
    private double flywheelVelocity;

    private double turretMinRot = Units.degreesToRadians(-90);
    private double turretMaxRot = Units.degreesToRadians(90);

    public JoystickTurret() {
        addRequirements(RobotContainer.turret, RobotContainer.shooter);
    }

    @Override
    public void initialize() {
        turretAngle = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations());
        hoodAngle = Units.rotationsToRadians(RobotContainer.shooter.getHoodAngle());
        RobotContainer.shootOrchestrator.overrideShootAngleVelocity = true;
        RobotContainer.shootOrchestrator.setEnableShooting(true);
        RobotContainer.shootOrchestrator.setFeedMode(FeedMode.DISABLED);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Turret Angle
        AbstractControl controls = RobotContainer.getControl();
        double manualTurretAngle = controls.getKidRawDriverInput().getRotation().getRadians();
        double deltaTA = manualTurretAngle * 0.05;
        turretAngle += deltaTA;
        turretAngle = MathUtil.clamp(turretAngle, turretMinRot, turretMaxRot);

        RobotContainer.shootOrchestrator.turretAngleOverride = Optional.of(turretAngle);

        // Hood Angle
        double manualHoodAngle = controls.getKidRawDriverInput().getY();
        double deltaHA = manualHoodAngle * 0.05;
        hoodAngle += deltaHA;
        hoodAngle = MathUtil.clamp(
                hoodAngle, Constants.Shooter.HOOD_MIN_POSITION_RADIANS, Constants.Shooter.HOOD_MAX_POSITION_RADIANS);

        RobotContainer.shootOrchestrator.hoodAngleOverride = hoodAngle;

        // Flywheel Velocity
        flywheelVelocity = controls.getKidsSpeedLevel() * KID_FLYWHEEL_MAX_VELOCITY;
        flywheelVelocity = MathUtil.clamp(flywheelVelocity, KID_FLYWHEEL_MIN_VELOCITY, KID_FLYWHEEL_MAX_VELOCITY);

        RobotContainer.shootOrchestrator.shootVelocityOverride = flywheelVelocity;

        // Shoot
        boolean shootingEnabled = controls.getKidShoot();
        if (shootingEnabled) {
            RobotContainer.shootOrchestrator.setFeedMode(FeedMode.AUTO);
        } else {
            RobotContainer.shootOrchestrator.setFeedMode(FeedMode.DISABLED);
        }
    }

    // Runs when the command ends
    @Override
    public void end(boolean interrupted) {
        RobotContainer.shootOrchestrator.setEnableShooting(false);
        RobotContainer.shootOrchestrator.turretAngleOverride = Optional.empty();
        RobotContainer.shootOrchestrator.overrideShootAngleVelocity = false;
        RobotContainer.shootOrchestrator.setFeedMode(FeedMode.AUTO);
    }
}
