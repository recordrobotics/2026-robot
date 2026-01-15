package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.Game.CoralLevel;
import frc.robot.dashboard.DashboardUI;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.modifiers.DrivetrainControl;

public interface AbstractControl {

    void update();

    // Movement
    DrivetrainControl getDrivetrainControl();

    Transform2d getRawDriverInput();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isAutoAlignTriggered();

    boolean isElevatorRelativeDriveTriggered();

    boolean isCoralIntakeRelativeDriveTriggered();

    boolean isClimbRelativeDriveTriggered();

    // Misc
    boolean isPoseResetTriggered();

    boolean isLimelightResetTriggered();

    boolean isKillTriggered();

    // Elevator
    boolean isAutoScoreTriggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isElevatorL2Triggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isElevatorL3Triggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isElevatorL4Triggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isElevatorAlgaeLowTriggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isElevatorAlgaeHighTriggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isManualOverrideTriggered();

    LinearVelocity getManualElevatorVelocity();

    AngularVelocity getManualElevatorArmVelocity();

    ReefLevelSwitchValue getReefLevelSwitchValue();

    // Intake coral
    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isCoralGroundIntakeTriggered();

    boolean isCoralGroundIntakeSimpleTriggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isCoralSourceIntakeTriggered();

    boolean isCoralSourceIntakeAutoTriggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isCoralIntakeScoreL1Triggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isCoralShootTriggered();

    // Ground Algae
    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isGroundAlgaeTriggered();

    // Score algae
    boolean isReefAlgaeSimpleTriggered();

    /**
     * @deprecated This is an old control scheme trigger and will be removed
     */
    @Deprecated(forRemoval = true)
    boolean isScoreAlgaeTriggered();

    // Climb
    boolean isClimbTriggered();

    boolean isClimbBurstTriggered();

    void vibrate(RumbleType type, double value);

    // Orient XY
    static Pair<Double, Double> orientXY(Pair<Double, Double> input) {
        double inputX = input.getFirst();
        double inputY = input.getSecond();

        return switch (DashboardUI.Overview.getDriverOrientation()) {
            case X_AXIS_TOWARDS_TRIGGER -> {
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) yield new Pair<>(-inputY, -inputX);
                else yield new Pair<>(inputY, inputX);
            }
            case Y_AXIS_TOWARDS_TRIGGER -> {
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) yield new Pair<>(inputX, -inputY);
                else yield new Pair<>(-inputX, inputY);
            }
            case X_AXIS_INVERTED_TOWARDS_TRIGGER -> {
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) yield new Pair<>(inputY, inputX);
                else yield new Pair<>(-inputY, -inputX);
            }
            default -> new Pair<>(0.0, 0.0);
        };
    }

    // Orient Angle
    static Rotation2d orientAngle(Rotation2d angle) {
        return switch (DashboardUI.Overview.getDriverOrientation()) {
            case X_AXIS_TOWARDS_TRIGGER -> {
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue)
                    yield new Rotation2d(angle.getRadians() - Math.PI / 2);
                else yield new Rotation2d(angle.getRadians() + Math.PI / 2);
            }
            case Y_AXIS_TOWARDS_TRIGGER -> {
                if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) yield angle;
                else yield new Rotation2d(angle.getRadians() + Math.PI);
            }
            default -> angle;
        };
    }

    enum ReefLevelSwitchValue {
        NONE,
        L1,
        L2,
        L3,
        L4;

        public CoralLevel toCoralLevel() {
            return switch (this) {
                case L1 -> CoralLevel.L1;
                case L2 -> CoralLevel.L2;
                case L3 -> CoralLevel.L3;
                case L4 -> CoralLevel.L4;
                default -> CoralLevel.L4; // None is L4 to preserve default far-align behavior
            };
        }
    }
}
