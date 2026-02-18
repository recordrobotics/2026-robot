package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;

public final class Intake extends KillableSubsystem implements PoweredSubsystem {
    // Arm Leader is on the left of the intake (right of the robot) and Arm Follower is on the right of the intake (left
    // of the robot)

    private static final double ARM_POSITION_TOLERANCE = 0.15; // TODO
    private static final double ARM_VELOCITY_TOLERANCE = 1.05; // TODO
    private static final double WHEEL_VELOCITY_TOLERANCE_MPS = 1.0; // TODO
    private final IntakeIO io;
    private final MotionMagicExpoVoltage armLeaderRequest;
    private final Follower armFollowerRequest;
    private final MotionMagicVelocityVoltage wheelRequest;
    private double armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
    private double wheelTargetVelocityMps = 0.0;
    private Distance rollerDiameter = Inches.of(1.875);
    private IntakeState targetState = IntakeState.STARTING;

    public enum IntakeState {
        INTAKE,
        EJECT,
        STARTING,
        RETRACTED
    }

    public Intake(IntakeIO io) {
        this.io = io;
        armLeaderRequest =
                new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS));
        armFollowerRequest = io.createArmFollower();
        wheelRequest = new MotionMagicVelocityVoltage(RotationsPerSecond.of(0.0));

        TalonFXConfiguration configLeader = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsLeader = configLeader.Slot0;
        slot0ConfigsLeader.kS = Constants.Intake.ARM_KS;
        slot0ConfigsLeader.kV = Constants.Intake.ARM_KV;
        slot0ConfigsLeader.kA = Constants.Intake.ARM_KA;
        slot0ConfigsLeader.kG = Constants.Intake.ARM_KG;
        slot0ConfigsLeader.kP = Constants.Intake.ARM_KP;
        slot0ConfigsLeader.kI = 0;
        slot0ConfigsLeader.kD = Constants.Intake.ARM_KD;
        slot0ConfigsLeader.GravityType = GravityTypeValue.Arm_Cosine;
        configLeader.Feedback.SensorToMechanismRatio = Constants.Intake.ARM_GEAR_RATIO;
        configLeader.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        io.applyArmLeaderTalonFXConfig(configLeader);

        TalonFXConfiguration configFollower = new TalonFXConfiguration();

        configFollower.Feedback.SensorToMechanismRatio = Constants.Intake.ARM_GEAR_RATIO;
        configFollower.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        io.applyArmFollowerTalonFXConfig(configFollower);

        io.setArmFollowerMotionMagic(armFollowerRequest);

        TalonFXConfiguration configWheel = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsWheel = configWheel.Slot0;
        slot0ConfigsWheel.kS = Constants.Intake.WHEEL_KS;
        slot0ConfigsWheel.kV = Constants.Intake.WHEEL_KV;
        slot0ConfigsWheel.kA = Constants.Intake.WHEEL_KA;
        slot0ConfigsWheel.kG = 0;
        slot0ConfigsWheel.kP = Constants.Intake.WHEEL_KP;
        slot0ConfigsWheel.kI = 0;
        slot0ConfigsWheel.kD = Constants.Intake.WHEEL_KD;
        configWheel.Feedback.SensorToMechanismRatio =
                Constants.Intake.WHEEL_GEAR_RATIO * Math.PI * rollerDiameter.in(Meter);
        configWheel.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        io.applyWheelTalonFXConfig(configWheel);

        setState(targetState);
    }

    @Override
    public void periodicManaged() {
        RobotContainer.model.intakeModel.update(Units.rotationsToRadians(getArmPositionRotations()), 0.0);
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getArmPositionRotations() { // average of both motors
        return (io.getArmLeaderPositionRotations() + io.getArmFollowerPositionRotations()) / 2.0;
    }

    public void setState(IntakeState state) {
        armTargetRotations = switch (state) {
            case INTAKE -> Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS);
            case EJECT -> Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS);
            case STARTING -> Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
            case RETRACTED -> Units.radiansToRotations(Constants.Intake.ARM_RETRACTED_POSITION_RADIANS);
        };
        wheelTargetVelocityMps = switch (state) {
            case INTAKE -> Constants.Intake.WHEEL_INTAKE_VELOCITY_MPS;
            case EJECT -> Constants.Intake.WHEEL_EJECT_VELOCITY_MPS;
            case STARTING, RETRACTED -> 0.0;
        };
        io.setArmLeaderMotionMagic(armLeaderRequest.withPosition(armTargetRotations)); // follower will follow this
        io.setWheelMotionMagic(wheelRequest.withVelocity(wheelTargetVelocityMps));
        targetState = state;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.DEBUG_REAL)
    public IntakeState getTargetState() {
        return targetState;
    }

    private boolean armAtGoal() {
        return SimpleMath.isWithinTolerance(getArmPositionRotations(), armTargetRotations, ARM_POSITION_TOLERANCE)
                && SimpleMath.isWithinTolerance(getArmVelocityRotationsPerSecond(), 0, ARM_VELOCITY_TOLERANCE);
    }

    private boolean wheelAtGoal() {
        return SimpleMath.isWithinTolerance(
                getWheelVelocityMps(), wheelTargetVelocityMps, WHEEL_VELOCITY_TOLERANCE_MPS);
    }

    public boolean atGoal() {
        return armAtGoal() && wheelAtGoal();
    }

    public double getArmVelocityRotationsPerSecond() {
        return (io.getArmLeaderVelocityRotationsPerSecond() + io.getArmFollowerVelocityRotationsPerSecond()) / 2.0;
    }

    public double getWheelVelocityMps() {
        return io.getWheelVelocityMps();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getArmLeaderCurrentDrawAmps() + io.getArmFollowerCurrentDrawAmps() + io.getWheelCurrentDrawAmps();
    }

    public void resetEncoders() {
        io.setArmPositionRotations(Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS));
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    @Override
    public void kill() {
        io.setArmVoltage(0);
        io.setWheelVoltage(0);
    }
}
