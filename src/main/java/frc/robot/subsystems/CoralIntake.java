// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.CoralIntakeIO;
import frc.robot.subsystems.io.sim.CoralIntakeSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.EncoderResettableSubsystem;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import frc.robot.utils.modifiers.GroundIntakeAssist;
import org.littletonrobotics.junction.Logger;

public final class CoralIntake extends KillableSubsystem implements PoweredSubsystem, EncoderResettableSubsystem {

    private static final Velocity<VoltageUnit> SYSID_RAMP_RATE = Volts.of(4.0).per(Second);
    private static final Voltage SYSID_STEP_VOLTAGE = Volts.of(2.3);
    private static final Time SYSID_TIMEOUT = Seconds.of(1.0);

    private static final double POSITION_TOLERANCE = 0.15;
    private static final double VELOCITY_TOLERANCE = 1.05;

    private final CoralIntakeIO io;

    private final PIDController pid =
            new PIDController(Constants.CoralIntake.WHEEL_KP, 0, Constants.CoralIntake.WHEEL_KD);

    private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(
            Constants.CoralIntake.WHEEL_KS, Constants.CoralIntake.WHEEL_KV, Constants.CoralIntake.WHEEL_KA);

    private CoralIntakeState currentIntakeState = CoralIntakeState.UP;

    private double intakePushAndPullRampStart = 0;

    private final MotionMagicExpoVoltage armRequest;

    private double armPositionCached = 0;
    private double armVelocityCached = 0;
    private double armVoltageCached = 0;
    private double wheelPositionCached = 0;
    private double wheelVelocityCached = 0;
    private double wheelVoltageCached = 0;

    private TrapezoidProfile.State currentSetpoint = new TrapezoidProfile.State();
    private double lastSpeed = 0;

    private final SysIdRoutine sysIdRoutineWheel;
    private final SysIdRoutine sysIdRoutineArm;

    public CoralIntake(CoralIntakeIO io) {
        this.io = io;

        TalonFXConfiguration armConfig = new TalonFXConfiguration();

        // set slot 0 gains
        Slot0Configs slot0ConfigsArm = armConfig.Slot0;
        slot0ConfigsArm.kS = Constants.CoralIntake.ARM_KS;
        slot0ConfigsArm.kV = Constants.CoralIntake.ARM_KV;
        slot0ConfigsArm.kA = Constants.CoralIntake.ARM_KA;
        slot0ConfigsArm.kG = Constants.CoralIntake.ARM_KG;
        slot0ConfigsArm.kP = Constants.CoralIntake.ARM_KP;
        slot0ConfigsArm.kI = 0;
        slot0ConfigsArm.kD = Constants.CoralIntake.ARM_KD;
        slot0ConfigsArm.GravityType = GravityTypeValue.Arm_Cosine;
        armConfig.Feedback.SensorToMechanismRatio = Constants.CoralIntake.ARM_GEAR_RATIO;

        // set Motion Magic settings
        MotionMagicConfigs motionMagicConfigsArm = armConfig.MotionMagic;
        motionMagicConfigsArm.MotionMagicCruiseVelocity = Constants.CoralIntake.MAX_ARM_VELOCITY;
        motionMagicConfigsArm.MotionMagicAcceleration = Constants.CoralIntake.MAX_ARM_ACCELERATION;
        motionMagicConfigsArm.MotionMagicJerk = Constants.CoralIntake.MAX_ARM_JERK;
        motionMagicConfigsArm.MotionMagicExpo_kV = Constants.CoralIntake.ARM_MMEXPO_KV;
        motionMagicConfigsArm.MotionMagicExpo_kA = Constants.CoralIntake.ARM_MMEXPO_KA;

        io.applyArmTalonFXConfig(armConfig
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.CoralIntake.ARM_SUPPLY_CURRENT_LIMIT)
                        .withStatorCurrentLimit(Constants.CoralIntake.ARM_STATOR_CURRENT_LIMIT)
                        .withSupplyCurrentLimitEnable(true)
                        .withStatorCurrentLimitEnable(true)));

        armPositionCached = io.getArmPosition();

        armRequest = new MotionMagicExpoVoltage(Units.radiansToRotations(Constants.CoralIntake.ARM_START_POS));
        set(CoralIntakeState.UP);

        sysIdRoutineWheel = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, // default 1 volt/second ramp rate
                        null, // default 7 volt step voltage
                        null,
                        state -> Logger.recordOutput("CoralIntake/Wheel/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setWheelVoltage(v.in(Volts)), null, this));

        sysIdRoutineArm = new SysIdRoutine(
                new SysIdRoutine.Config(
                        SYSID_RAMP_RATE,
                        SYSID_STEP_VOLTAGE,
                        SYSID_TIMEOUT,
                        state -> Logger.recordOutput("CoralIntake/Arm/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setArmVoltage(v.in(Volts)), null, this));

        SmartDashboard.putNumber("CoralIntakeArm", Constants.CoralIntake.ARM_START_POS);

        GroundIntakeAssist.getDefault().setEnabled(true);
    }

    public CoralIntakeSim getSimIO() throws IllegalStateException {
        if (io instanceof CoralIntakeSim simIO) {
            return simIO;
        } else {
            throw new IllegalStateException("CoralIntakeIO is not a simulation");
        }
    }

    public enum CoralIntakeState {
        UP,
        SOURCE,
        PUSH_READY,
        PUSH_OUT,
        GROUND,
        L1_SCORE,
        L1_DOWN;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getWheelVelocity() {
        return wheelVelocityCached
                / SimpleMath.SECONDS_PER_MINUTE
                / Constants.CoralIntake.WHEEL_GEAR_RATIO; /* RPM -> RPS */
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getWheelPosition() {
        return wheelPositionCached / Constants.CoralIntake.WHEEL_GEAR_RATIO;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getWheelSetTo() {
        return wheelVoltageCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmAngle() {
        return armPositionCached * SimpleMath.PI2;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmVelocity() {
        return armVelocityCached * SimpleMath.PI2;
    }

    /** Used for sysid as units have to be in rotations in the logs */
    @AutoLogLevel(level = Level.SYSID)
    public double getArmAngleRotations() {
        return armPositionCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmVelocityRotations() {
        return armVelocityCached;
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getArmSetTo() {
        return armVoltageCached;
    }

    @AutoLogLevel(level = Level.DEBUG_REAL)
    public CoralIntakeState getState() {
        return currentIntakeState;
    }

    /** Set the current shooter speed on both wheels to speed */
    public void setWheel(double speed) {
        pid.setSetpoint(speed);
    }

    public void setArm(double angleRadians) {
        currentSetpoint.position = angleRadians;
        if (!(SysIdManager.getProvider() instanceof SysIdArm)) {
            io.setArmMotionMagic(armRequest.withPosition(Units.radiansToRotations(angleRadians)));
        }
    }

    public boolean armAtGoal() {
        return SimpleMath.isWithinTolerance(getArmAngle(), currentSetpoint.position, POSITION_TOLERANCE)
                && SimpleMath.isWithinTolerance(getArmVelocity(), 0, VELOCITY_TOLERANCE);
    }

    public void set(CoralIntakeState state) {
        currentIntakeState = state;
        switch (state) {
            case SOURCE -> {
                setWheel(Constants.CoralIntake.SOURCE_SPEED);
                setArm(Constants.CoralIntake.ARM_INTAKE);
            }
            case PUSH_READY -> {
                setWheel(Constants.CoralIntake.INTAKE_SPEED);
                setArm(Constants.CoralIntake.ARM_PUSH);
            }
            case PUSH_OUT -> {
                intakePushAndPullRampStart = Timer.getTimestamp();
                setWheel(Constants.CoralIntake.PUSH_OUT_SPEED);
                setArm(Constants.CoralIntake.ARM_PUSH);
            }
            case GROUND -> {
                setWheel(Constants.CoralIntake.INTAKE_SPEED);
                setArm(Constants.CoralIntake.ARM_DOWN);
            }
            case L1_SCORE -> {
                setWheel(Constants.CoralIntake.L1_SCORE_SPEED);
                setArm(Constants.CoralIntake.ARM_SCORE_L1);
            }
            case L1_DOWN -> {
                setWheel(0);
                setArm(Constants.CoralIntake.ARM_SCORE_L1);
            }
            case UP -> {
                setWheel(0);
                setArm(Constants.CoralIntake.ARM_UP);
            }
            default -> { // should never happen
                io.setArmVoltage(0);
                setWheel(0);
            }
        }
    }

    @Override
    public void periodicManaged() {
        wheelVelocityCached = io.getWheelVelocity();
        armPositionCached = io.getArmPosition();
        armVelocityCached = io.getArmVelocity();
        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.SYSID)) {
            wheelPositionCached = io.getWheelPosition();
            wheelVoltageCached = io.getWheelVoltage();
            armVoltageCached = io.getArmVoltage();
        }

        if (currentIntakeState == CoralIntakeState.PUSH_OUT) {
            // push and pull ramp
            double rampT = MathUtil.clamp(
                    (Timer.getTimestamp() - intakePushAndPullRampStart) / Constants.CoralIntake.PUSH_OUT_RAMP_TIME,
                    0,
                    1);
            double rampedSpeed = MathUtil.interpolate(
                    Constants.CoralIntake.PUSH_OUT_SPEED, Constants.CoralIntake.PULL_THROUGH_SPEED, rampT);
            setWheel(rampedSpeed);
        }

        double pidOutput = pid.calculate(getWheelVelocity());
        double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());

        if (!(SysIdManager.getProvider() instanceof SysIdWheel)) {
            io.setWheelVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
        }

        lastSpeed = pid.getSetpoint();

        // Update mechanism
        RobotContainer.model.coralIntake.update(getArmAngle());
        RobotContainer.model.coralIntake.updateSetpoint(currentSetpoint.position);
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

    @Override
    public void kill() {
        setWheel(0);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getWheelCurrentDrawAmps() + io.getArmCurrentDrawAmps();
    }

    @Override
    public void resetEncoders() {
        io.setArmPosition(Units.radiansToRotations(Constants.CoralIntake.ARM_START_POS));
        armPositionCached = Units.radiansToRotations(Constants.CoralIntake.ARM_START_POS);
    }

    public static class SysIdArm implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.coralIntake.sysIdQuasistaticArm(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.coralIntake.sysIdDynamicArm(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }

    public static class SysIdWheel implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.coralIntake.sysIdQuasistaticWheel(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.coralIntake.sysIdDynamicWheel(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
