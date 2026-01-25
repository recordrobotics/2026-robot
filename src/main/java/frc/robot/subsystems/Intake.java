package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;

public class Intake extends KillableSubsystem implements PoweredSubsystem {

    private static final double ARM_POSITION_TOLERANCE = 0.15;
    private static final double ARM_VELOCITY_TOLERANCE = 1.05;
    private final IntakeIO io;
    private final MotionMagicExpoVoltage armRequest;
    private double armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_UP_POSITION);

    public Intake(IntakeIO io) {
        this.io = io;
        armRequest = new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Intake.ARM_UP_POSITION));
    }

    @Override
    public void periodic() {
        RobotContainer.model.intake.update(getArmPositionRotations());
    }

    public double getArmPositionRotations() { // average of both motors
        return (io.getArmLeaderPositionRotations() + io.getArmFollowerPositionRotations()) / 2.0;
    }

    public void setArm(double angleRadians) {
        armTargetRotations = Units.radiansToRotations(angleRadians);
        io.setArmLeaderMotionMagic(armRequest.withPosition(armTargetRotations));
    }

    public boolean armAtGoal() {
        return SimpleMath.isWithinTolerance(getArmPositionRotations(), armTargetRotations, ARM_POSITION_TOLERANCE)
                && SimpleMath.isWithinTolerance(getArmVelocityRotationsPerSecond(), 0, ARM_VELOCITY_TOLERANCE);
    }

    public double getArmVelocityRotationsPerSecond() {
        return (io.getArmLeaderVelocityRotationsPerSecond() + io.getArmFollowerVelocityRotationsPerSecond()) / 2.0;
    }

    public double getWheelVelocityRotationsPerSecond() {
        return io.getWheelVelocityRotationsPerSecond();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getArmLeaderCurrentDrawAmps() + io.getArmFollowerCurrentDrawAmps() + io.getWheelCurrentDrawAmps();
    }

    public void resetEncoders() {
        io.setArmPositionRotations(0); // TODO is right position to reset to 0?
    }

    @Override
    public void kill() {
        io.setArmVoltage(0);
        io.setWheelVoltage(0);
    }
}
