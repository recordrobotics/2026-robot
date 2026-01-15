package frc.robot.control;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

/**
 * @deprecated This is an old control scheme and will be removed
 */
@Deprecated(forRemoval = true)
@SuppressWarnings({"java:S109"})
public class JoystickXbox implements AbstractControl {

    private static final double SHOULDER_TRIGGER_THRESHOLD = 0.3;

    private static final double HALF_SPEED_DIRECTIONAL_DIVIDER = 3;
    private static final double HALF_SPEED_SPIN_DIVIDER = 2;

    private static final LinearVelocity MANUAL_ELEVATOR_VELOCITY_MAX =
            Centimeters.of(50).per(Second);
    private static final AngularVelocity MANUAL_ELEVATOR_ARM_VELOCITY_MAX =
            Degrees.of(180).per(Second);
    private static final double MANUAL_AXIS_DEADBAND = 0.1;

    private Joystick joystick;
    private XboxController xboxController;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    /**
     * @deprecated This is an old control scheme and will be removed
     */
    @Deprecated(forRemoval = true)
    public JoystickXbox(int joystickPort, int xboxPort) {
        joystick = new Joystick(joystickPort);
        xboxController = new XboxController(xboxPort);
    }

    @Override
    public void update() {
        Pair<Double, Double> xy;
        if (isCoralIntakeRelativeDriveTriggered()
                || isElevatorRelativeDriveTriggered()
                || isClimbRelativeDriveTriggered()) {
            xy = getXYRaw();
        } else {
            xy = getXYOriented();
        }

        double x = xy.getFirst() * getDirectionalSpeedLevel();
        double y = xy.getSecond() * getDirectionalSpeedLevel();

        if (isCoralIntakeRelativeDriveTriggered()) {
            y = -y;
        } else if (isElevatorRelativeDriveTriggered()) {
            double temp = y;
            y = -x;
            x = -temp;
        } else if (isClimbRelativeDriveTriggered()) {
            double temp = y;
            y = x;
            x = temp;
        }

        velocity = new Transform2d(x, y, new Rotation2d(getSpin() * getSpinSpeedLevel()));
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
    public Transform2d getRawDriverInput() {
        Pair<Double, Double> xy = getXYRaw();
        // Returns the raw driver input as a Transform2d
        return new Transform2d(xy.getFirst(), xy.getSecond(), Rotation2d.fromRadians(getSpin()));
    }

    @Override
    public DrivetrainControl getDrivetrainControl() {
        if (isElevatorRelativeDriveTriggered()
                || isCoralIntakeRelativeDriveTriggered()
                || isClimbRelativeDriveTriggered()) {
            return DrivetrainControl.createRobotRelative(velocity, acceleration, jerk);
        } else {
            return DrivetrainControl.createFieldRelative(
                    velocity,
                    acceleration,
                    jerk,
                    RobotContainer.poseSensorFusion.getEstimatedPosition().getRotation());
        }
    }

    public boolean isAutoAlignTriggered() {
        return joystick.getRawButton(9) || isAutoAlignNearTriggered();
    }

    private boolean isAutoAlignNearTriggered() {
        return joystick.getRawButton(7);
    }

    @Override
    public boolean isElevatorRelativeDriveTriggered() {
        return joystick.getRawButton(8) || joystick.getRawButton(2);
    }

    @Override
    public boolean isCoralIntakeRelativeDriveTriggered() {
        return joystick.getRawButton(10);
    }

    @Override
    public boolean isClimbRelativeDriveTriggered() {
        return joystick.getRawButton(12);
    }

    public Pair<Double, Double> getXYRaw() {
        double x = SimpleMath.applyThresholdAndSensitivity(
                joystick.getX(),
                Constants.Control.JOYSTICK_X_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
        double y = SimpleMath.applyThresholdAndSensitivity(
                joystick.getY(),
                Constants.Control.JOYSTICK_Y_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);

        return new Pair<>(x, y);
    }

    public Pair<Double, Double> getXYOriented() {
        Pair<Double, Double> xy = getXYRaw();
        return AbstractControl.orientXY(new Pair<>(xy.getFirst(), xy.getSecond()));
    }

    public Double getSpin() {
        // Gets raw twist value
        return SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(joystick.getTwist(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public boolean isHalfSpeedTriggered() {
        return joystick.getRawButton(1);
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.remap(
                joystick.getRawAxis(3),
                1,
                -1,
                Constants.Control.DIRECTIONAL_SPEED_METER_LOW,
                Constants.Control.DIRECTIONAL_SPEED_METER_HIGH);

        if (isHalfSpeedTriggered()) {
            speed /= HALF_SPEED_DIRECTIONAL_DIVIDER;
        }

        return speed;
    }

    public Double getSpinSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.remap(
                joystick.getRawAxis(3),
                1,
                -1,
                Constants.Control.SPIN_SPEED_METER_LOW,
                Constants.Control.SPIN_SPEED_METER_HIGH);

        if (isHalfSpeedTriggered()) {
            speed /= HALF_SPEED_SPIN_DIVIDER;
        }

        return speed;
    }

    @Override
    public boolean isPoseResetTriggered() {
        return joystick.getRawButtonPressed(3) || joystick.getRawButtonPressed(5);
    }

    @Override
    public boolean isLimelightResetTriggered() {
        return joystick.getRawButtonPressed(4) || joystick.getRawButtonPressed(6);
    }

    @Override
    public boolean isKillTriggered() {
        return xboxController.getRawButton(8);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        xboxController.setRumble(type, value);
    }

    @Override
    public boolean isAutoScoreTriggered() {
        return false;
    }

    @Override
    public boolean isElevatorL2Triggered() {
        return xboxController.getXButton();
    }

    @Override
    public boolean isElevatorL3Triggered() {
        return xboxController.getBButton();
    }

    @Override
    public boolean isElevatorL4Triggered() {
        return xboxController.getYButton();
    }

    @Override
    public boolean isCoralShootTriggered() {
        return xboxController.getPOV() == 270;
    }

    @Override
    public boolean isCoralGroundIntakeTriggered() {
        return xboxController.getLeftTriggerAxis() > SHOULDER_TRIGGER_THRESHOLD;
    }

    @Override
    public boolean isCoralGroundIntakeSimpleTriggered() {
        return false;
    }

    @Override
    public boolean isCoralSourceIntakeTriggered() {
        return xboxController.getLeftBumperButton();
    }

    @Override
    public boolean isElevatorAlgaeLowTriggered() {
        return xboxController.getRightTriggerAxis() > SHOULDER_TRIGGER_THRESHOLD && xboxController.getPOV() != 0;
    }

    @Override
    public boolean isElevatorAlgaeHighTriggered() {
        return xboxController.getRightTriggerAxis() > SHOULDER_TRIGGER_THRESHOLD && xboxController.getPOV() == 0;
    }

    @Override
    public ReefLevelSwitchValue getReefLevelSwitchValue() {
        return isAutoAlignNearTriggered() ? ReefLevelSwitchValue.L2 : ReefLevelSwitchValue.NONE;
    }

    @Override
    public boolean isManualOverrideTriggered() {
        return xboxController.getPOV() == 0;
    }

    @Override
    public boolean isGroundAlgaeTriggered() {
        return xboxController.getRightBumperButton();
    }

    @Override
    public boolean isReefAlgaeSimpleTriggered() {
        return false;
    }

    @Override
    public boolean isScoreAlgaeTriggered() {
        return xboxController.getPOV() == 90;
    }

    @Override
    public boolean isCoralIntakeScoreL1Triggered() {
        return xboxController.getPOV() == 180;
    }

    @Override
    public LinearVelocity getManualElevatorVelocity() {
        double leftY = MathUtil.applyDeadband(-xboxController.getLeftY(), MANUAL_AXIS_DEADBAND);
        return MANUAL_ELEVATOR_VELOCITY_MAX.times(leftY * Math.abs(leftY));
    }

    @Override
    public AngularVelocity getManualElevatorArmVelocity() {
        double rightY = MathUtil.applyDeadband(-xboxController.getRightY(), MANUAL_AXIS_DEADBAND);
        return MANUAL_ELEVATOR_ARM_VELOCITY_MAX.times(rightY * Math.abs(rightY));
    }

    @Override
    public boolean isClimbTriggered() {
        return xboxController.getRawButton(7);
    }

    @Override
    public boolean isClimbBurstTriggered() {
        return false;
    }

    @Override
    public boolean isCoralSourceIntakeAutoTriggered() {
        return false;
    }
}
