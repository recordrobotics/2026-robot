package frc.robot.control;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;

/**
 * @deprecated This is a backup control scheme using an Xbox controller.
 * It is not part of the official control schemes and is not available in the control practice
 */
@Deprecated(forRemoval = false)
public class XboxSimpleBackup implements AbstractControl {

    private static final double SHOULDER_TRIGGER_THRESHOLD = 0.3;

    private static final double AUTO_SOURCE_MAX_DISTANCE = 2.3; // meters
    private static final double AUTO_SOURCE_MAX_ANGLE_DIFF = 80; // degrees

    private static final double SPEED_LEVEL = 2;
    private static final double HALF_SPEED_DIRECTIONAL_DIVIDER = 3;
    private static final double HALF_SPEED_SPIN_DIVIDER = 2;

    private XboxController xboxController;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    private ReefLevelSwitchValue reefswitch = ReefLevelSwitchValue.L4;

    /**
     * @deprecated This is a backup control scheme using an Xbox controller.
     * It is not part of the official control schemes and is not available in the control practice
     */
    @Deprecated(forRemoval = false)
    public XboxSimpleBackup(int xboxPort) {
        xboxController = new XboxController(xboxPort);

        new Trigger(() -> xboxController.getAButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L1).ignoringDisable(true));
        new Trigger(() -> xboxController.getXButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L2).ignoringDisable(true));
        new Trigger(() -> xboxController.getBButton())
                .onTrue(new InstantCommand(() -> reefswitch = ReefLevelSwitchValue.L3).ignoringDisable(true));
        new Trigger(() -> xboxController.getYButton())
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
        return false;
    }

    @Override
    public boolean isElevatorRelativeDriveTriggered() {
        return xboxController.getPOV() == 0 // POV up
                || (isAutoScoreTriggered()
                        && getReefLevelSwitchValue()
                                != ReefLevelSwitchValue.L1); // elevator relative when auto score not L1
    }

    @Override
    public boolean isCoralIntakeRelativeDriveTriggered() {
        return xboxController.getPOV() == 270 // POV left
                || (isAutoScoreTriggered()
                        && getReefLevelSwitchValue() == ReefLevelSwitchValue.L1); // coral relative when auto score L1
    }

    @Override
    public boolean isClimbRelativeDriveTriggered() {
        return xboxController.getPOV() == 180; // POV down
    }

    public Pair<Double, Double> getXYRaw() {
        double x = SimpleMath.applyThresholdAndSensitivity(
                xboxController.getLeftX(),
                Constants.Control.JOYSTICK_X_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
        double y = SimpleMath.applyThresholdAndSensitivity(
                xboxController.getLeftY(),
                Constants.Control.JOYSTICK_Y_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);

        return new Pair<>(x, y);
    }

    public Pair<Double, Double> getXYOriented() {
        Pair<Double, Double> xy = getXYRaw();
        return AbstractControl.orientXY(new Pair<>(xy.getFirst(), xy.getSecond()));
    }

    public double getSpin() {
        // Gets raw twist value
        return SimpleMath.applyThresholdAndSensitivity(
                -SimpleMath.remap(xboxController.getRightX(), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public boolean isHalfSpeedTriggered() {
        return isAutoScoreTriggered(); // half speed auto enabled when scoring
    }

    public Double getDirectionalSpeedLevel() {
        double speed = SPEED_LEVEL;

        if (isHalfSpeedTriggered()) {
            speed /= HALF_SPEED_DIRECTIONAL_DIVIDER;
        }

        return speed;
    }

    public Double getSpinSpeedLevel() {
        double speed = SPEED_LEVEL;

        if (isHalfSpeedTriggered()) {
            speed /= HALF_SPEED_SPIN_DIVIDER;
        }

        return speed;
    }

    @Override
    public boolean isPoseResetTriggered() {
        return xboxController.getBackButton();
    }

    @Override
    public boolean isLimelightResetTriggered() {
        return xboxController.getStartButton();
    }

    @Override
    public boolean isKillTriggered() {
        return xboxController.getPOV() == 90;
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        xboxController.setRumble(type, value);
    }

    @Override
    public boolean isAutoScoreTriggered() {
        return xboxController.getRightBumperButton();
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
        return xboxController.getRightTriggerAxis() > SHOULDER_TRIGGER_THRESHOLD;
    }

    @Override
    public boolean isReefAlgaeSimpleTriggered() {
        return xboxController.getLeftBumperButton();
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
        return MetersPerSecond.of(0);
    }

    @Override
    public AngularVelocity getManualElevatorArmVelocity() {
        return RotationsPerSecond.of(0);
    }

    @Override
    public boolean isClimbTriggered() {
        return xboxController.getLeftTriggerAxis() > SHOULDER_TRIGGER_THRESHOLD;
    }

    @Override
    public boolean isClimbBurstTriggered() {
        return false;
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
