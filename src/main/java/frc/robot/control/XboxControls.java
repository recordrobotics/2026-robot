package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

@SuppressWarnings({"java:S109"})
public class XboxControls implements AbstractControl {

    private static final double SLOW_SPEED = 2.0;

    private XboxController xbox;

    private ProfiledPIDController spinController = new ProfiledPIDController(
            Constants.Control.SPIN_KP, 0, Constants.Control.SPIN_KD, Constants.Control.SPIN_CONSTRAINTS);
    private double spinOutput = 0;
    private double spinTarget = 0;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    public XboxControls(int xboxPort) {
        xbox = new XboxController(xboxPort);
        spinController.enableContinuousInput(-Math.PI, Math.PI);
    }

    private Pair<Double, Double> getXYStickOutput() {
        return new Pair<>(xbox.getLeftX(), xbox.getLeftY());
    }

    @Override
    public void update() {
        // calculate velocity from the target spin position
        Pair<Boolean, Rotation2d> targetPair = getSpinRotation2d();
        boolean hasTarget = targetPair.getFirst();
        Rotation2d target = targetPair.getSecond();
        if (!hasTarget) {
            target = new Rotation2d(spinTarget);
        } else {
            spinTarget = target.getRadians();
        }
        spinOutput = spinController.calculate(
                RobotContainer.poseSensorFusion
                        .getEstimatedPosition()
                        .getRotation()
                        .getRadians(),
                target.getRadians());

        Pair<Double, Double> xy = getXYOriented();

        double maxSpeed = isSlowSpeedPressed() ? SLOW_SPEED : Constants.Swerve.MAX_MODULE_SPEED;
        double x = xy.getFirst() * maxSpeed;
        double y = xy.getSecond() * maxSpeed;

        velocity = new Transform2d(x, y, new Rotation2d(spinOutput));
        acceleration = new Transform2d(
                        velocity.getTranslation()
                                .minus(lastVelocity.getTranslation())
                                .div(RobotContainer.ROBOT_PERIODIC),
                        velocity.getRotation().minus(lastVelocity.getRotation()))
                .div(RobotContainer.ROBOT_PERIODIC);
        jerk = new Transform2d(
                        acceleration
                                .getTranslation()
                                .minus(lastAcceleration.getTranslation())
                                .div(RobotContainer.ROBOT_PERIODIC),
                        acceleration.getRotation().minus(lastAcceleration.getRotation()))
                .div(RobotContainer.ROBOT_PERIODIC);

        lastVelocity = velocity;
        lastAcceleration = acceleration;
    }

    @Override
    public DrivetrainControl getDrivetrainControl() {
        return DrivetrainControl.createFieldRelative(
                velocity,
                acceleration,
                jerk,
                RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());
    }

    @Override
    public Transform2d getRawDriverInput() {
        Pair<Double, Double> xy = getXYRaw();
        // Returns the raw driver input as a Transform2d
        return new Transform2d(xy.getFirst(), xy.getSecond(), new Rotation2d(spinOutput));
    }

    public Pair<Double, Double> getXYRaw() {
        Pair<Double, Double> xy = getXYStickOutput();
        double unsquaredX = SimpleMath.applyThresholdAndSensitivity(
                xy.getFirst(), Constants.Control.JOYSTICK_XY_THRESHOLD, Constants.Control.JOYSTICK_XY_SENSITIVITY);
        double unsquaredY = SimpleMath.applyThresholdAndSensitivity(
                xy.getSecond(), Constants.Control.JOYSTICK_XY_THRESHOLD, Constants.Control.JOYSTICK_XY_SENSITIVITY);
        // Squares the inputs while preserving the sign to allow for finer control at low speeds
        double x = Math.copySign(Math.pow(unsquaredX, Constants.Control.JOYSTICK_XY_EXPONENT), unsquaredX);
        double y = Math.copySign(Math.pow(unsquaredY, Constants.Control.JOYSTICK_XY_EXPONENT), unsquaredY);

        return new Pair<>(x, y);
    }

    public Pair<Double, Double> getXYOriented() {
        Pair<Double, Double> xy = getXYRaw();
        return AbstractControl.orientXY(new Pair<>(xy.getFirst(), xy.getSecond()));
    }

    public Pair<Boolean, Rotation2d> getSpinRotation2d() {
        double x = xbox.getRightStickButton() ? xbox.getLeftX() : xbox.getRightX();
        double y = xbox.getRightStickButton() ? xbox.getLeftY() : xbox.getRightY();
        double angle = -Math.atan2(y, x) + Math.PI / 2;

        if (DriverStationUtils.getCurrentAlliance() == Alliance.Red) {
            angle += Math.PI;
        }

        double magnitude = Math.hypot(x, y);
        if (magnitude
                < (xbox.getRightStickButton()
                        ? Constants.Control.JOYSTICK_ABSOLUTE_SPIN_THRESHOLD_PACMAN
                        : Constants.Control.JOYSTICK_ABSOLUTE_SPIN_THRESHOLD)) {
            return new Pair<>(false, new Rotation2d());
        }
        return new Pair<>(true, new Rotation2d(angle));
    }

    @Override
    public boolean isPoseResetTriggered() {
        return xbox.getRawButtonPressed(7);
    }

    @Override
    public boolean isKillTriggered() {
        return xbox.getRawButton(8);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        xbox.setRumble(type, value);
    }

    @Override
    public boolean isForceIntakePressed() {
        return xbox.getLeftTriggerAxis() > 0.75;
    }

    @Override
    public boolean isClimbPressed() {
        return xbox.getAButton();
    }

    @Override
    public boolean isReverseIntakePressed() {
        return xbox.getLeftBumperButton();
    }

    @Override
    public boolean isShooterInvertPressed() {
        return xbox.getRightTriggerAxis() > 0.75;
    }

    @Override
    public boolean isUnstuckSpindexerPressed() {
        return xbox.getRightBumperButton();
    }

    @Override
    public boolean isSlowSpeedPressed() {
        return xbox.getLeftStickButton();
    }

    @Override
    public String toDisplayName() {
        return "Xbox";
    }
}
