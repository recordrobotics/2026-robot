// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.io.ElevatorHeadIO;
import frc.robot.subsystems.io.sim.ElevatorHeadSim;
import frc.robot.utils.AutoLogLevel;
import frc.robot.utils.AutoLogLevel.Level;
import frc.robot.utils.KillableSubsystem;
import frc.robot.utils.PoweredSubsystem;
import frc.robot.utils.SimpleMath;
import frc.robot.utils.SysIdManager;
import frc.robot.utils.SysIdManager.SysIdProvider;
import org.littletonrobotics.junction.Logger;

public final class ElevatorHead extends KillableSubsystem implements PoweredSubsystem {

    private static final String GP_NAME_NONE = "";
    private static final String GP_NAME_ALGAE = "algae";
    private static final String GP_NAME_CORAL = "coral";

    private static final double CORAL_READY_MAX_VELOCITY = 0.1; // m/s
    private static final double WAITING_FOR_ALGAE_MIN_VELOCITY = 1.0; // TODO: tune
    private static final double ALGAE_ACQUIRED_MAX_VELOCITY = 0.5; // TODO: tune

    private static final double UNSTUCKIFY_CORAL_VELOCITY = -3.0;

    private final ElevatorHeadIO io;

    private boolean debouncedValue = false;
    private boolean debouncedCertainValue = false;
    private Debouncer debouncer = new Debouncer(Constants.ElevatorHead.DEBOUNCE_TIME, Debouncer.DebounceType.kFalling);
    private Debouncer debouncerCertain =
            new Debouncer(Constants.ElevatorHead.DEBOUNCE_TIME_CERTAIN, Debouncer.DebounceType.kFalling);

    private final PIDController pid = new PIDController(Constants.ElevatorHead.KP, 0, Constants.ElevatorHead.KD);
    private final ProfiledPIDController positionPid = new ProfiledPIDController(
            Constants.ElevatorHead.KP_POSITION,
            0,
            Constants.ElevatorHead.KD_POSITION,
            new TrapezoidProfile.Constraints(
                    Constants.ElevatorHead.POSITION_MODE_MAX_VELOCITY,
                    Constants.ElevatorHead.POSITION_MODE_MAX_ACCELERATION));
    private final SimpleMotorFeedforward feedForward =
            new SimpleMotorFeedforward(Constants.ElevatorHead.KS, Constants.ElevatorHead.KV, Constants.ElevatorHead.KA);

    private CoralShooterStates currentState = CoralShooterStates.OFF;

    private double positionCached = 0;
    private double velocityCached = 0;
    private double voltageCached = 0;

    private boolean hasAlgae = false;
    private boolean waitingForAlgae = false;
    private boolean waitingForIntakeSpeed = false;

    private double lastSpeed = 0;

    private final SysIdRoutine sysIdRoutine;

    private boolean lastUnstuckifyCoral = false;
    private double lastUnstuckifyCoralPidSetpoint = 0;

    public ElevatorHead(ElevatorHeadIO io) {
        this.io = io;

        set(CoralShooterStates.OFF); // initialize as off

        io.setPosition(0);
        positionCached = 0;
        positionPid.reset(0);

        positionPid.setTolerance(
                Constants.ElevatorHead.AT_GOAL_POSITION_TOLERANCE, Constants.ElevatorHead.AT_GOAL_VELOCITY_TOLERANCE);

        sysIdRoutine = new SysIdRoutine(
                // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
                new SysIdRoutine.Config(
                        null,
                        null,
                        null,
                        state -> Logger.recordOutput("ElevatorHead/SysIdTestState", state.toString())),
                new SysIdRoutine.Mechanism(v -> io.setVoltage(v.in(Volts)), null, this));

        SmartDashboard.putBoolean("ElevatorHead/UnstuckifyCoral", false);
    }

    public ElevatorHeadSim getSimIO() throws IllegalStateException {
        if (io instanceof ElevatorHeadSim simIO) {
            return simIO;
        } else {
            throw new IllegalStateException("ElevatorHeadIO is not a simulation");
        }
    }

    public enum AlgaeGrabberStates {
        OUT_GROUND,
        OUT_REEF,
        SHOOT_BARGE,
        INTAKE_GROUND,
        INTAKE_REEF,
        HOLD_REEF,
        HOLD_GROUND,
        OFF;
    }

    public enum CoralShooterStates {
        OUT_FORWARD,
        OUT_BACKWARD,
        INTAKE,
        POSITION,
        OFF;
    }

    public enum GamePiece {
        NONE(0, GP_NAME_NONE),
        ALGAE(0, GP_NAME_ALGAE),
        CORAL(0, GP_NAME_CORAL),
        CORAL_CERTAIN(1, GP_NAME_CORAL),
        CORAL_POSITIONED(2, GP_NAME_CORAL);

        private final int tier;
        private final String gpName;

        private GamePiece(int tier, String gpName) {
            this.tier = tier;
            this.gpName = gpName;
        }

        public boolean atLeast(GamePiece other) {
            return this.gpName.equals(other.gpName) && this.tier >= other.tier;
        }
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getVelocity() {
        return velocityCached
                / SimpleMath.SECONDS_PER_MINUTE
                / Constants.ElevatorHead.GEAR_RATIO
                * Math.PI
                * Constants.ElevatorHead.WHEEL_DIAMETER.in(Meters); /* RPM -> RPS -> METERS */
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getPosition() {
        return positionCached
                / Constants.ElevatorHead.GEAR_RATIO
                * Math.PI
                * Constants.ElevatorHead.WHEEL_DIAMETER.in(Meters); /* Rotations -> Meters */
    }

    @AutoLogLevel(level = Level.SYSID)
    public double getVoltage() {
        return voltageCached;
    }

    public void moveBy(Distance distance) {
        moveBy(distance.in(Meters));
    }

    public void moveBy(double meters) {
        set(0);
        currentState = CoralShooterStates.POSITION;
        positionPid.reset(getPosition());
        positionPid.setGoal(getPosition() + meters);
    }

    /** Set shooter speed */
    public void set(double speed) {
        pid.setSetpoint(speed);
    }

    /** Set the shooter speed to the preset ShooterStates state */
    public void set(CoralShooterStates state) {
        waitingForAlgae = false;
        waitingForIntakeSpeed = false;
        hasAlgae = false;
        switch (state) {
            case OUT_FORWARD:
                currentState = CoralShooterStates.OUT_FORWARD;
                set(Constants.ElevatorHead.OUT_SPEED_FORWARD);
                break;
            case OUT_BACKWARD:
                currentState = CoralShooterStates.OUT_BACKWARD;
                set(Constants.ElevatorHead.OUT_SPEED_BACKWARD);
                break;
            case INTAKE:
                currentState = CoralShooterStates.INTAKE;
                set(Constants.ElevatorHead.INTAKE_SPEED);
                break;
            case POSITION:
                currentState = CoralShooterStates.POSITION;
                set(0);
                positionPid.setGoal(getPosition());
                break;
            case OFF: // Off
            default: // should never happen
                currentState = CoralShooterStates.OFF;
                set(0);
                break;
        }
    }

    public void set(AlgaeGrabberStates state) {
        currentState = CoralShooterStates.OFF;
        switch (state) {
            case OUT_GROUND:
                set(Constants.ElevatorHead.OUT_GROUND_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = false;
                hasAlgae = false;
                break;
            case OUT_REEF:
                set(Constants.ElevatorHead.OUT_REEF_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = false;
                hasAlgae = false;
                break;
            case SHOOT_BARGE:
                set(Constants.ElevatorHead.SHOOT_BARGE_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = false;
                hasAlgae = false;
                break;
            case INTAKE_GROUND:
                set(Constants.ElevatorHead.INTAKE_GROUND_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = true;
                hasAlgae = false;
                break;
            case INTAKE_REEF:
                set(Constants.ElevatorHead.INTAKE_REEF_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = true;
                hasAlgae = false;
                break;
            case HOLD_REEF:
                set(Constants.ElevatorHead.HOLD_REEF_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = false;
                hasAlgae = true;
                break;
            case HOLD_GROUND:
                set(Constants.ElevatorHead.HOLD_GROUND_SPEED);
                waitingForAlgae = false;
                waitingForIntakeSpeed = false;
                hasAlgae = true;
                break;
            case OFF: // Off
            default: // should never happen
                waitingForIntakeSpeed = false;
                waitingForAlgae = false;
                set(0);
                break;
        }
    }

    @AutoLogLevel(level = Level.REAL)
    public GamePiece getGamePiece() {
        if (debouncedCertainValue && positionAtGoal()) {
            return GamePiece.CORAL_POSITIONED;
        } else if (debouncedCertainValue) {
            return GamePiece.CORAL_CERTAIN;
        } else if (debouncedValue) {
            return GamePiece.CORAL;
        } else if (hasAlgae) {
            return GamePiece.ALGAE;
        } else {
            return GamePiece.NONE;
        }
    }

    @AutoLogLevel(level = Level.REAL)
    public CoralShooterStates getCurrentCoralShooterState() {
        return currentState;
    }

    private boolean positionAtGoal() {
        return positionPid.atGoal();
    }

    /**
     * @return true if the coral shooter is ready to shoot coral (at low velocity or has coral and at goal position)
     * @deprecated use getGamePiece().atLeast(GamePiece.CORAL_POSITIONED) instead
     */
    @Deprecated(forRemoval = true)
    public boolean coralReady() {
        return Math.abs(getVelocity()) < CORAL_READY_MAX_VELOCITY
                || (getGamePiece().atLeast(GamePiece.CORAL) && positionAtGoal());
    }

    @Override
    public void periodicManaged() {

        positionCached = io.getPosition();
        velocityCached = io.getVelocity();
        if (Constants.RobotState.AUTO_LOG_LEVEL.isAtOrLowerThan(Level.SYSID)) {
            voltageCached = io.getVoltage();
        }

        boolean unstuckifyCoral = SmartDashboard.getBoolean("ElevatorHead/UnstuckifyCoral", false);

        if (!unstuckifyCoral && currentState == CoralShooterStates.POSITION) {
            double pidOutput = positionPid.calculate(getPosition());

            Logger.recordOutput("ElevatorHead/PositionSetpoint", positionPid.getSetpoint().position);

            double feedforwardOutput =
                    feedForward.calculateWithVelocities(lastSpeed, positionPid.getSetpoint().velocity);

            if (!(SysIdManager.getProvider() instanceof SysId)) {
                io.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
            }

            lastSpeed = positionPid.getSetpoint().velocity;
        } else {

            if (unstuckifyCoral && !lastUnstuckifyCoral) {
                lastUnstuckifyCoral = true;
                lastUnstuckifyCoralPidSetpoint = pid.getSetpoint();
                pid.setSetpoint(UNSTUCKIFY_CORAL_VELOCITY);
            } else if (!unstuckifyCoral && lastUnstuckifyCoral) {
                lastUnstuckifyCoral = false;
                pid.setSetpoint(lastUnstuckifyCoralPidSetpoint);
            }

            double pidOutput = pid.calculate(getVelocity());
            double feedforwardOutput = feedForward.calculateWithVelocities(lastSpeed, pid.getSetpoint());

            if (!(SysIdManager.getProvider() instanceof SysId)) {
                io.setVoltage(pidOutput + feedforwardOutput); // Feed forward runs on voltage control
            }

            lastSpeed = pid.getSetpoint();

            if (waitingForIntakeSpeed && Math.abs(getVelocity()) > WAITING_FOR_ALGAE_MIN_VELOCITY) {
                waitingForIntakeSpeed = false;
                waitingForAlgae = true;
            }

            if (waitingForAlgae && Math.abs(getVelocity()) < ALGAE_ACQUIRED_MAX_VELOCITY) {
                hasAlgae = true;
                waitingForAlgae = false;
            }
        }

        debouncedValue = !debouncer.calculate(io.isCoralDetectorTriggered());
        debouncedCertainValue = !debouncerCertain.calculate(io.isCoralDetectorTriggered());
    }

    @Override
    public void simulationPeriodicManaged() {
        io.simulationPeriodic();
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    @Override
    public void kill() {
        set(CoralShooterStates.OFF);
        set(AlgaeGrabberStates.OFF);
        io.setVoltage(0);
    }

    /** frees up all hardware allocations */
    @Override
    public void close() throws Exception {
        io.close();
    }

    @Override
    public double getCurrentDrawAmps() {
        return io.getCurrentDrawAmps();
    }

    public static class SysId implements SysIdProvider {
        @Override
        public Command sysIdQuasistatic(Direction direction) {
            return RobotContainer.elevatorHead.sysIdQuasistatic(direction);
        }

        @Override
        public Command sysIdDynamic(Direction direction) {
            return RobotContainer.elevatorHead.sysIdDynamic(direction);
        }

        @Override
        public boolean isEnabled() {
            return true;
        }
    }
}
