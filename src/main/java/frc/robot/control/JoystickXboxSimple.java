package frc.robot.control;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

@SuppressWarnings({"java:S109"})
public class JoystickXboxSimple implements AbstractControl {

    private static final double HALF_SPEED_DIRECTIONAL_DIVIDER = 3;
    private static final double HALF_SPEED_SPIN_DIVIDER = 2;

    private static final LinearVelocity MANUAL_ELEVATOR_VELOCITY_MAX =
            Centimeters.of(50).per(Second);
    private static final AngularVelocity MANUAL_ELEVATOR_ARM_VELOCITY_MAX =
            Degrees.of(180).per(Second);

    private static final double AUTO_SOURCE_MAX_DISTANCE = 2.3; // meters
    private static final double AUTO_SOURCE_MAX_ANGLE_DIFF = 80; // degrees

    private Joystick joystick;
    private XboxController xboxController;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    private ReefLevelSwitchValue reefswitch = ReefLevelSwitchValue.L4;

    public JoystickXboxSimple(int joystickPort, int xboxPort) {
        joystick = new Joystick(joystickPort);
        xboxController = new XboxController(xboxPort);

        new Trigger(() -> xboxController.getAButtonPressed())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L1).ignoringDisable(true));
        new Trigger(() -> xboxController.getXButtonPressed())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L2).ignoringDisable(true));
        new Trigger(() -> xboxController.getBButtonPressed())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L3).ignoringDisable(true));
        new Trigger(() -> xboxController.getYButtonPressed())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L4).ignoringDisable(true));
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

    @Override
    public boolean isElevatorRelativeDriveTriggered() {
        return joystick.getRawButton(8)
                || (isAutoScoreTriggered()
                        && getReefLevelSwitchValue() != ReefLevelSwitchValue.L1); // elevator relative when auto score
    }

    @Override
    public boolean isCoralIntakeRelativeDriveTriggered() {
        return joystick.getRawButton(10)
                || (isAutoScoreTriggered()
                        && getReefLevelSwitchValue() == ReefLevelSwitchValue.L1); // coral relative when auto score
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
        return isAutoScoreTriggered(); // half speed auto enabled when scoring
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
        return joystick.getRawButtonPressed(5);
    }

    @Override
    public boolean isLimelightResetTriggered() {
        return joystick.getRawButtonPressed(6);
    }

    @Override
    public boolean isKillTriggered() {
        return joystick.getRawButton(4);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        xboxController.setRumble(type, value);
    }

    @Override
    public boolean isAutoScoreTriggered() {
        return joystick.getRawButton(1);
    }

    @Override
    public boolean isElevatorL2Triggered() {
        return false;
    }

    @Override
    public boolean isElevatorL3Triggered() {
        return false;
    }

    @Override
    public boolean isElevatorL4Triggered() {
        return false;
    }

    @Override
    public boolean isCoralShootTriggered() {
        return false;
    }

    @Override
    public boolean isCoralGroundIntakeTriggered() {
        return false;
    }

    @Override
    public boolean isCoralGroundIntakeSimpleTriggered() {
        return joystick.getRawButton(2);
    }

    @Override
    public boolean isReefAlgaeSimpleTriggered() {
        return joystick.getRawButton(3);
    }

    @Override
    public boolean isCoralSourceIntakeTriggered() {
        return false;
    }

    @Override
    public boolean isElevatorAlgaeLowTriggered() {
        return false;
    }

    @Override
    public boolean isElevatorAlgaeHighTriggered() {
        return false;
    }

    @Override
    public ReefLevelSwitchValue getReefLevelSwitchValue() {
        return reefswitch;
    }

    @Override
    public boolean isManualOverrideTriggered() {
        return false;
    }

    @Override
    public boolean isGroundAlgaeTriggered() {
        return false;
    }

    @Override
    public boolean isScoreAlgaeTriggered() {
        return false;
    }

    @Override
    public boolean isCoralIntakeScoreL1Triggered() {
        return false;
    }

    @Override
    public LinearVelocity getManualElevatorVelocity() {
        double axis = SimpleMath.povToVector(joystick.getPOV()).getY();
        return MANUAL_ELEVATOR_VELOCITY_MAX.times(axis);
    }

    @Override
    public AngularVelocity getManualElevatorArmVelocity() {
        double axis = SimpleMath.povToVector(joystick.getPOV()).getX();
        return MANUAL_ELEVATOR_ARM_VELOCITY_MAX.times(axis);
    }

    @Override
    public boolean isClimbTriggered() {
        return joystick.getRawButton(7);
    }

    @Override
    public boolean isClimbBurstTriggered() {
        return joystick.getRawButton(9);
    }

    @Override
    public boolean isCoralSourceIntakeAutoTriggered() {
        Pose2d robot = RobotContainer.poseSensorFusion.getEstimatedPosition();
        SourcePosition closestSource = IGamePosition.closestTo(robot, SourcePosition.values());

        return closestSource.getPose().getTranslation().getDistance(robot.getTranslation()) < AUTO_SOURCE_MAX_DISTANCE
                && Math.abs(closestSource
                                .getPose()
                                .getRotation()
                                .minus(robot.getRotation())
                                .getMeasure()
                                .abs(Degrees))
                        < AUTO_SOURCE_MAX_ANGLE_DIFF;
    }
}
