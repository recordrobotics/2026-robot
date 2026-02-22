package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.IntakeIO;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class Intake extends KillableSubsystem implements PoweredSubsystem {
    // Arm Leader is on the left of the intake (right of the robot) and Arm Follower is on the right of the intake (left
    // of the robot)

    private static final double ARM_POSITION_TOLERANCE = Units.degreesToRotations(5);
    private static final double ARM_POSITION_TOLERANCE_WHEEL_START = Units.degreesToRotations(6);
    private static final double ARM_VELOCITY_TOLERANCE = Units.degreesToRotations(50);
    private static final double WHEEL_VELOCITY_TOLERANCE_MPS = 1.0; // TODO

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(0.8).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(0.2);
    private static final Time SYSID_TIMEOUT = Seconds.of(0.8);

    private static final Time SYSID_WHEEL_TIMEOUT = Seconds.of(3.0);

    private final IntakeIO io;
    private final SysIdRoutine sysIdRoutineArm;
    private final SysIdRoutine sysIdRoutineWheel;

    private final MotionMagicExpoVoltage armLeaderRequest;
    private final Follower armFollowerRequest;
    private final MotionMagicVelocityVoltage wheelRequest;

    private double armTargetRotations = Units.radiansToRotations(Constants.Intake.ARM_STARTING_POSITION_RADIANS);
    private double wheelTargetVelocityMps = 0.0;
    private double actualWheelTargetVelocityMps = 0.0;
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
        slot0ConfigsLeader.kD = Constants.Intake.ARM_KD;
        slot0ConfigsLeader.GravityType = GravityTypeValue.Arm_Cosine;
        slot0ConfigsLeader.GravityArmPositionOffset =
                Units.radiansToRotations(Constants.Intake.ARM_GRAVITY_POSITION_OFFSET_RADIANS);

        configLeader.MotionMagic.MotionMagicExpo_kV = Constants.Intake.ARM_MMEXPO_KV;
        configLeader.MotionMagic.MotionMagicExpo_kA = Constants.Intake.ARM_MMEXPO_KA;

        configLeader.CurrentLimits.SupplyCurrentLimit = Constants.Intake.ARM_SUPPLY_CURRENT_LIMIT.in(Amps);
        configLeader.CurrentLimits.StatorCurrentLimit = Constants.Intake.ARM_STATOR_CURRENT_LIMIT.in(Amps);
        configLeader.CurrentLimits.SupplyCurrentLimitEnable = true;
        configLeader.CurrentLimits.StatorCurrentLimitEnable = true;
        configLeader.CurrentLimits.SupplyCurrentLowerLimit = Constants.Intake.ARM_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        configLeader.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Intake.ARM_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);

        configLeader.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                Units.radiansToRotations(Constants.Intake.ARM_MAX_POSITION_RADIANS);
        configLeader.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                Units.radiansToRotations(Constants.Intake.ARM_DOWN_POSITION_RADIANS);
        configLeader.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        configLeader.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        configLeader.Feedback.SensorToMechanismRatio = Constants.Intake.ARM_GEAR_RATIO;
        configLeader.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        configLeader.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyArmLeaderTalonFXConfig(configLeader);
        io.applyArmFollowerTalonFXConfig(
                configLeader.withMotorOutput(configLeader.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)));

        io.setArmFollowerMotionMagic(armFollowerRequest);

        TalonFXConfiguration configWheel = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsWheel = configWheel.Slot0;
        slot0ConfigsWheel.kS = Constants.Intake.WHEEL_KS;
        slot0ConfigsWheel.kV = Constants.Intake.WHEEL_KV;
        slot0ConfigsWheel.kA = Constants.Intake.WHEEL_KA;
        slot0ConfigsWheel.kP = Constants.Intake.WHEEL_KP;

        configWheel.MotionMagic.MotionMagicJerk = Constants.Intake.WHEEL_MAX_JERK;
        configWheel.MotionMagic.MotionMagicAcceleration = Constants.Intake.WHEEL_MAX_ACCELERATION;

        configWheel.CurrentLimits.SupplyCurrentLimit = Constants.Intake.WHEEL_SUPPLY_CURRENT_LIMIT.in(Amps);
        configWheel.CurrentLimits.StatorCurrentLimit = Constants.Intake.WHEEL_STATOR_CURRENT_LIMIT.in(Amps);
        configWheel.CurrentLimits.SupplyCurrentLimitEnable = true;
        configWheel.CurrentLimits.StatorCurrentLimitEnable = true;
        configWheel.CurrentLimits.SupplyCurrentLowerLimit = Constants.Intake.WHEEL_SUPPLY_LOWER_CURRENT_LIMIT.in(Amps);
        configWheel.CurrentLimits.SupplyCurrentLowerTime =
                Constants.Intake.WHEEL_SUPPLY_LOWER_CURRENT_LIMIT_TIME.in(Seconds);

        configWheel.Feedback.SensorToMechanismRatio = 1.0 / Constants.Intake.WHEEL_METERS_PER_ROTATION;
        configWheel.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        configWheel.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        io.applyWheelTalonFXConfig(configWheel);

        setState(targetState);

        sysIdRoutineArm = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("Intake/Arm/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(
                        v -> io.setArmVoltage(v.in(Volts) + 1.7 * Math.signum(v.in(Volts)) + 0.3), null, this));

        sysIdRoutineWheel = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        SYSID_WHEEL_TIMEOUT,
                        state -> Logger.recordOutput("Intake/Wheel/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setWheelVoltage(v.in(Volts)), null, this));
    }

    @Override
    public void periodicManaged() {
        RobotContainer.model.intakeModel.update(Units.rotationsToRadians(getArmPositionRotations()));

        updateWheel();
    }

    public void setState(IntakeState state) {
        targetState = state;

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

        if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysIdArm))
            io.setArmLeaderMotionMagic(armLeaderRequest.withPosition(armTargetRotations)); // follower will follow this

        updateWheel();
    }

    @Override
    protected void onForceDisabledChange(boolean isNowForceDisabled) {
        if (isNowForceDisabled) {
            io.setArmVoltage(0.0);
            io.setWheelVoltage(0.0);
        } else {
            io.setArmLeaderMotionMagic(armLeaderRequest.withPosition(armTargetRotations)); // follower will follow this
            io.setWheelMotionMagic(wheelRequest.withVelocity(actualWheelTargetVelocityMps));
        }
    }

    private void updateWheel() {
        boolean armNearGoal = SimpleMath.isWithinTolerance(
                getArmPositionRotations(), armTargetRotations, ARM_POSITION_TOLERANCE_WHEEL_START);

        if (armNearGoal || Math.abs(wheelTargetVelocityMps) < Math.abs(actualWheelTargetVelocityMps)) {
            actualWheelTargetVelocityMps = wheelTargetVelocityMps;

            if (!isForceDisabled() && !(SysIdManager.getProvider() instanceof SysIdWheel)) {
                io.setWheelMotionMagic(wheelRequest.withVelocity(actualWheelTargetVelocityMps));
            }
        }
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

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getArmPositionRotations() { // average of both motors
        return (io.getArmLeaderPositionRotations() + io.getArmFollowerPositionRotations()) / 2.0;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getArmVelocityRotationsPerSecond() {
        return (io.getArmLeaderVelocityRotationsPerSecond() + io.getArmFollowerVelocityRotationsPerSecond()) / 2.0;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getArmVoltage() {
        return (io.getArmLeaderVoltage() + io.getArmFollowerVoltage()) / 2.0;
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getWheelPositionMeters() {
        return io.getWheelPositionMeters();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getWheelVelocityMps() {
        return io.getWheelVelocityMps();
    }

    @AutoLogLevel(level = AutoLogLevel.Level.SYSID)
    public double getWheelVoltage() {
        return io.getWheelVoltage();
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

    public Command sysIdQuasistaticWheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineWheel.quasistatic(direction);
    }

    public Command sysIdDynamicWheel(SysIdRoutine.Direction direction) {
        return sysIdRoutineWheel.dynamic(direction);
    }

    public Command sysIdQuasistaticArm(SysIdRoutine.Direction direction) {
        return sysIdRoutineArm.quasistatic(direction);
    }

    public Command sysIdDynamicArm(SysIdRoutine.Direction direction) {
        return sysIdRoutineArm.dynamic(direction);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() {
        io.close();
    }

    public static class SysIdArm implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.intake.sysIdQuasistaticArm(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.intake.sysIdDynamicArm(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }

    public static class SysIdWheel implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.intake.sysIdQuasistaticWheel(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.intake.sysIdDynamicWheel(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }

        @Override
        public boolean isReversed() {
            return false;
        }
    }
}
