package frc.robot.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

@SuppressWarnings({"java:S109"})
public class KidJoystick {
    public record KidsShootTarget(
            Rotation2d turretAngleFieldRelative, double hoodAngleRadians, double flywheelVelocityMps) {}

    private static final LoggedNetworkNumber kidShotMinVelocity = new LoggedNetworkNumber("Kid Shot Min Velocity", 0.5);
    private static final LoggedNetworkNumber kidShotMaxVelocity = new LoggedNetworkNumber("Kid Shot Max Velocity", 28);
    private static final LoggedNetworkNumber turretMinRotationRadians =
            new LoggedNetworkNumber("Kid Shot Turret Min Rotation Radians", -Math.PI);
    private static final LoggedNetworkNumber turretMaxRotationRadians =
            new LoggedNetworkNumber("Kid Shot Turret Max Rotation Radians", Math.PI);

    private double turretTargetRadians;
    private double hoodAngleRadians;
    private double flywheelVelocityMps;

    private Joystick kidJoystick;

    public KidJoystick(int kidJoystickPort) {
        kidJoystick = new Joystick(kidJoystickPort);
        turretTargetRadians = Units.rotationsToRadians(RobotContainer.turret.getPositionRotations());
        hoodAngleRadians = RobotContainer.shooter.getHoodAngleRadians();
        flywheelVelocityMps = kidShotMinVelocity.get();
        RobotContainer.shootOrchestrator.setEnableShooting(true);
    }

    public void execute() {
        // Turret Angle
        double deltaTurret = RobotContainer.kidControl.getKidSpin() * 0.05;
        turretTargetRadians += deltaTurret;
        if (turretTargetRadians < turretMinRotationRadians.get()) {
            turretTargetRadians = turretMaxRotationRadians.get();
        } else if (turretTargetRadians > turretMaxRotationRadians.get()) {
            turretTargetRadians = turretMinRotationRadians.get();
        }
        // Hood Angle
        double manualHoodAngle = RobotContainer.kidControl.getKidY();
        double deltaHA = manualHoodAngle * 0.05;
        hoodAngleRadians += deltaHA;
        hoodAngleRadians = MathUtil.clamp(
                hoodAngleRadians,
                Constants.Shooter.HOOD_MIN_POSITION_RADIANS,
                Constants.Shooter.HOOD_MAX_POSITION_RADIANS);

        // Flywheel Velocity
        flywheelVelocityMps = SimpleMath.remap(
                RobotContainer.kidControl.getKidsSpeedLevel(),
                0,
                1,
                kidShotMinVelocity.get(),
                kidShotMaxVelocity.get());
    }

    public Transform2d getKidRawDriverInput() {
        // Returns the raw driver input as a Transform2d
        return new Transform2d(getKidX(), getKidY(), Rotation2d.fromRadians(getKidSpin()));
    }

    public boolean getKidShoot() {
        return kidJoystick.getRawButton(1);
    }

    public boolean getKidShootPressed() {
        return kidJoystick.getRawButtonPressed(1);
    }

    public Double getKidX() {
        double unsquaredX = SimpleMath.applyThresholdAndSensitivity(
                kidJoystick.getX(), Constants.Control.JOYSTICK_XY_THRESHOLD, Constants.Control.JOYSTICK_XY_SENSITIVITY);
        return Math.copySign(Math.pow(unsquaredX, Constants.Control.JOYSTICK_XY_EXPONENT), unsquaredX);
    }

    public Double getKidY() {
        double unsquaredY = SimpleMath.applyThresholdAndSensitivity(
                kidJoystick.getY(), Constants.Control.JOYSTICK_XY_THRESHOLD, Constants.Control.JOYSTICK_XY_SENSITIVITY);
        return Math.copySign(Math.pow(unsquaredY, Constants.Control.JOYSTICK_XY_EXPONENT), unsquaredY);
    }

    public Double getKidSpin() {
        // Gets raw twist value
        double unsquaredSpin = SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(kidJoystick.getTwist(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
        // Squares the input while preserving the sign to allow for finer control at low speeds
        return Math.copySign(Math.pow(unsquaredSpin, Constants.Control.JOYSTICK_SPIN_EXPONENT), unsquaredSpin);
    }

    public double getKidsSpeedLevel() {
        return SimpleMath.remap(kidJoystick.getRawAxis(3), 1, -1, 0, 1);
    }

    public Rotation2d getTurretTarget() {
        return new Rotation2d(turretTargetRadians);
    }

    public double getHoodAngleRadians() {
        return hoodAngleRadians;
    }

    public double getFlywheelVelocityMps() {
        return flywheelVelocityMps;
    }
}
