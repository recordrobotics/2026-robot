package frc.robot.control;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.utils.DriverStationUtils;
import frc.robot.utils.modifiers.DrivetrainControl;

public interface AbstractControl {

    void update();

    boolean isIntakeUpPressed();

    boolean isIntakePressed();

    boolean isIntakeInvertPressed();

    boolean isReverseIntakePressed();

    boolean isClimbPressed();

    boolean isShooterInvertPressed();

    boolean isUnstuckSpindexerPressed();

    // Movement
    DrivetrainControl getDrivetrainControl();

    Transform2d getRawDriverInput();

    boolean getKidShoot();

    Transform2d getKidRawDriverInput();

    double getKidsSpeedLevel();

    // Misc
    boolean isPoseResetTriggered();

    boolean isKillTriggered();

    boolean isSlowSpeedPressed();

    void vibrate(RumbleType type, double value);

    String toDisplayName();

    // Orient XY
    static Pair<Double, Double> orientXY(Pair<Double, Double> input) {
        double inputX = input.getFirst();
        double inputY = input.getSecond();

        if (DriverStationUtils.getCurrentAlliance() == Alliance.Blue) return new Pair<>(-inputY, -inputX);
        else return new Pair<>(inputY, inputX);
    }
}
