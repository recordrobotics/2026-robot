package frc.robot.tests;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants;
import frc.robot.Constants.Game.IGamePosition;
import frc.robot.Constants.Game.SourcePosition;
import frc.robot.RobotContainer;
import frc.robot.control.AbstractControl;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.modifiers.DrivetrainControl;
import java.util.EnumMap;
import java.util.Map;

public class TestControlBridge implements AbstractControl {

    private static final double HALF_SPEED_DIRECTIONAL_DIVIDER = 3;
    private static final double HALF_SPEED_SPIN_DIVIDER = 2;

    private static final LinearVelocity MANUAL_ELEVATOR_VELOCITY_MAX =
            Centimeters.of(50).per(Second);
    private static final AngularVelocity MANUAL_ELEVATOR_ARM_VELOCITY_MAX =
            Degrees.of(180).per(Second);

    private static final double AUTO_SOURCE_MAX_DISTANCE = 2.3; // meters
    private static final double AUTO_SOURCE_MAX_ANGLE_DIFF = 80; // degrees

    private static TestControlBridge instance;

    static {
        DashboardUI.Overview.setTestControl(getInstance());
    }

    private ReefLevelSwitchValue reefswitch = ReefLevelSwitchValue.L4;

    private Transform2d lastVelocity = new Transform2d();
    private Transform2d lastAcceleration = new Transform2d();
    private Transform2d velocity = new Transform2d();
    private Transform2d acceleration = new Transform2d();
    private Transform2d jerk = new Transform2d();

    private EnumMap<Button, Integer> buttonStates = new EnumMap<>(Button.class);
    private EnumMap<Axis, Double> axisStates = new EnumMap<>(Axis.class);
    private EnumMap<RumbleType, Double> rumbleStates = new EnumMap<>(RumbleType.class);

    public static TestControlBridge getInstance() {
        if (instance == null) {
            instance = new TestControlBridge();
        }
        return instance;
    }

    private boolean getButton(Button btn) {
        int state = buttonStates.getOrDefault(btn, 0);
        // state > 0 is being pressed, state < 0 is being held down, state == 0 is released
        return state != 0;
    }

    @Override
    public void update() {
        for (Map.Entry<Button, Integer> entry : buttonStates.entrySet()) {
            int state = entry.getValue();

            if (state > 0) {
                entry.setValue(state - 1); // Decrement the button state
            } else if (state < 0) {
                entry.setValue(-1); // Button is held down
            }
        }

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
                                .div(0.02),
                        velocity.getRotation().minus(lastVelocity.getRotation()))
                .div(0.02);
        jerk = new Transform2d(
                        acceleration
                                .getTranslation()
                                .minus(lastAcceleration.getTranslation())
                                .div(0.02),
                        acceleration.getRotation().minus(lastAcceleration.getRotation()))
                .div(0.02);

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
        return getButton(Button.AUTO_ALIGN);
    }

    @Override
    public boolean isElevatorRelativeDriveTriggered() {
        return getButton(Button.ELEVATOR_RELATIVE_DRIVE)
                || (isAutoScoreTriggered()
                        && getReefLevelSwitchValue() != ReefLevelSwitchValue.L1); // elevator relative when auto score
    }

    @Override
    public boolean isCoralIntakeRelativeDriveTriggered() {
        return getButton(Button.CORAL_INTAKE_RELATIVE_DRIVE)
                || (isAutoScoreTriggered()
                        && getReefLevelSwitchValue() == ReefLevelSwitchValue.L1); // coral relative when auto score
    }

    @Override
    public boolean isClimbRelativeDriveTriggered() {
        return getButton(Button.CLIMB_RELATIVE_DRIVE);
    }

    public Pair<Double, Double> getXYRaw() {
        double x = SimpleMath.applyThresholdAndSensitivity(
                axisStates.getOrDefault(Axis.X, 0.0),
                Constants.Control.JOYSTICK_X_THRESHOLD,
                Constants.Control.JOYSTICK_DIRECTIONAL_SENSITIVITY);
        double y = SimpleMath.applyThresholdAndSensitivity(
                axisStates.getOrDefault(Axis.Y, 0.0),
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
                -SimpleMath.remap(axisStates.getOrDefault(Axis.TWIST, 0.0), -1.0, 1.0, -1.0, 1.0),
                Constants.Control.JOYSTICK_SPIN_THRESHOLD,
                Constants.Control.JOYSTICK_SPIN_SENSITIVITY);
    }

    public boolean isHalfSpeedTriggered() {
        return isAutoScoreTriggered(); // half speed auto enabled when scoring
    }

    public Double getDirectionalSpeedLevel() {
        // Remaps speed meter from -1 -> 1 to 0.5 -> 4, then returns
        double speed = SimpleMath.remap(
                axisStates.getOrDefault(Axis.SPEED_LEVEL, 0.0),
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
                axisStates.getOrDefault(Axis.SPEED_LEVEL, 0.0),
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
        return getButton(Button.POSE_RESET);
    }

    @Override
    public boolean isLimelightResetTriggered() {
        return getButton(Button.LIMELIGHT_RESET);
    }

    @Override
    public boolean isKillTriggered() {
        return getButton(Button.KILL);
    }

    @Override
    public void vibrate(RumbleType type, double value) {
        rumbleStates.put(type, value);
    }

    @Override
    public boolean isAutoScoreTriggered() {
        return getButton(Button.AUTO_SCORE);
    }

    @Override
    public boolean isElevatorL2Triggered() {
        return getButton(Button.L2);
    }

    @Override
    public boolean isElevatorL3Triggered() {
        return getButton(Button.L3);
    }

    @Override
    public boolean isElevatorL4Triggered() {
        return getButton(Button.L4);
    }

    @Override
    public boolean isCoralShootTriggered() {
        return getButton(Button.CORAL_SHOOT);
    }

    @Override
    public boolean isCoralGroundIntakeTriggered() {
        return getButton(Button.GROUND_INTAKE);
    }

    @Override
    public boolean isCoralGroundIntakeSimpleTriggered() {
        return getButton(Button.GROUND_INTAKE_SIMPLE);
    }

    @Override
    public boolean isReefAlgaeSimpleTriggered() {
        return getButton(Button.REEF_ALGAE_SIMPLE);
    }

    @Override
    public boolean isCoralSourceIntakeTriggered() {
        return getButton(Button.SOURCE_INTAKE);
    }

    @Override
    public boolean isElevatorAlgaeLowTriggered() {
        return getButton(Button.ALGAE_LOW);
    }

    @Override
    public boolean isElevatorAlgaeHighTriggered() {
        return getButton(Button.ALGAE_HIGH);
    }

    @Override
    public ReefLevelSwitchValue getReefLevelSwitchValue() {
        return reefswitch;
    }

    @Override
    public boolean isManualOverrideTriggered() {
        return getButton(Button.MANUAL_OVERRIDE);
    }

    @Override
    public boolean isGroundAlgaeTriggered() {
        return getButton(Button.GROUND_ALGAE);
    }

    @Override
    public boolean isScoreAlgaeTriggered() {
        return getButton(Button.SCORE_ALGAE);
    }

    @Override
    public boolean isCoralIntakeScoreL1Triggered() {
        return getButton(Button.INTAKE_SCORE_L1);
    }

    @Override
    public LinearVelocity getManualElevatorVelocity() {
        double axis = SimpleMath.povToVector((int) Math.round(axisStates.getOrDefault(Axis.POV, -1.0)))
                .getY();
        return MANUAL_ELEVATOR_VELOCITY_MAX.times(axis);
    }

    @Override
    public AngularVelocity getManualElevatorArmVelocity() {
        double axis = SimpleMath.povToVector((int) Math.round(axisStates.getOrDefault(Axis.POV, -1.0)))
                .getX();
        return MANUAL_ELEVATOR_ARM_VELOCITY_MAX.times(axis);
    }

    @Override
    public boolean isClimbTriggered() {
        return getButton(Button.CLIMB);
    }

    @Override
    public boolean isClimbBurstTriggered() {
        return getButton(Button.CLIMB_BURST);
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

    public double getRumble(RumbleType type) {
        return rumbleStates.getOrDefault(type, 0.0);
    }

    /**
     * Sets the reef level switch value.
     * @param level the reef level switch value to set
     */
    public void setReefLevel(ReefLevelSwitchValue level) {
        this.reefswitch = level;
    }

    /**
     * Holds a button indefinitely until released {@link #releaseButton(Button)}.
     * @param btn the button to hold
     */
    public void holdButton(Button btn) {
        buttonStates.put(btn, -1);
    }

    /**
     * Releases a button.
     * @param btn the button to release
     */
    public void releaseButton(Button btn) {
        buttonStates.put(btn, 0);
    }

    /**
     * Presses a button for a certain number of cycles.
     * The button will be pressed for the specified number of cycles, after which it will be released.
     * @param btn the button to press
     * @param cycles the number of cycles to press the button for
     */
    public void pressButton(Button btn, int cycles) {
        // add 1 to cycle count because of update order - control periodic (where cycles is decremented) is called
        // before trigger periodic where value is read
        buttonStates.put(btn, cycles + 1);
    }

    /**
     * Sets the value of an axis.
     * @param axis the axis to set
     * @param value the value to set the axis to
     */
    public void setAxis(Axis axis, double value) {
        axisStates.put(axis, value);
    }

    /**
     * Resets the control bridge state.
     * This method resets all button states, axis states, and motion states to their initial values.
     */
    public void reset() {
        lastVelocity = new Transform2d();
        lastAcceleration = new Transform2d();
        velocity = new Transform2d();
        acceleration = new Transform2d();
        jerk = new Transform2d();

        buttonStates.clear();
        axisStates.clear();
        reefswitch = ReefLevelSwitchValue.L4;
    }

    public enum Button {
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        AUTO_ALIGN,
        ELEVATOR_RELATIVE_DRIVE,
        CORAL_INTAKE_RELATIVE_DRIVE,
        CLIMB_RELATIVE_DRIVE,
        POSE_RESET,
        LIMELIGHT_RESET,
        KILL,
        AUTO_SCORE,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        L2,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        L3,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        L4,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        CORAL_SHOOT,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        GROUND_INTAKE,
        GROUND_INTAKE_SIMPLE,
        REEF_ALGAE_SIMPLE,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        SOURCE_INTAKE,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        ALGAE_LOW,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        ALGAE_HIGH,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        MANUAL_OVERRIDE,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        GROUND_ALGAE,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        SCORE_ALGAE,
        /**
         * @deprecated This is an old control scheme trigger and will be removed
         */
        @Deprecated(forRemoval = true)
        INTAKE_SCORE_L1,
        CLIMB,
        CLIMB_BURST
    }

    public enum Axis {
        X,
        Y,
        TWIST,
        SPEED_LEVEL,
        POV
    }
}
