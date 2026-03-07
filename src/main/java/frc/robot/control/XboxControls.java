package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

@SuppressWarnings({"java:S109"})
public class XboxControls implements AbstractControl {

    private XboxController xbox;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    public XboxControls(int xboxPort) {
        xbox = new XboxController(xboxPort);
    }

    private Pair<Double, Double> getXYStickOutput() {
        return new Pair<>(xbox.getLeftX(), xbox.getLeftY());
    }

    private double getSpinStickOutput() {
        return xbox.getRightX();
    }

    @Override
    public void update() {
        Pair<Double, Double> xy = getXYOriented();

        double x = xy.getFirst() * Constants.Swerve.MAX_MODULE_SPEED;
        double y = xy.getSecond() * Constants.Swerve.MAX_MODULE_SPEED;

        velocity = new Transform2d(x, y, new Rotation2d(getSpin() * Constants.Swerve.MAX_ANGULAR_SPEED_RADIANS));
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
        return new Transform2d(xy.getFirst(), xy.getSecond(), Rotation2d.fromRadians(getSpin()));
    }

    public boolean isAutoAlignTriggered() {
        return false;
    }

    public boolean isAutoAlignNearTriggered() {
        return false;
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

    public Double getSpin() {
        // Gets raw twist value
        double unsquaredSpin = SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(getSpinStickOutput(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
        // Squares the input while preserving the sign to allow for finer control at low speeds
        return Math.copySign(Math.pow(unsquaredSpin, Constants.Control.JOYSTICK_SPIN_EXPONENT), unsquaredSpin) / 2;
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
    public boolean isShooterInvertPressed() {
        return xbox.getRightTriggerAxis() > 0.75;
    }

    @Override
    public boolean isUnstuckSpindexerPressed() {
        return xbox.getRightBumperButton();
    }

    @Override
    public String toDisplayName() {
        return "Xbox";
    }
}
