package frc.robot.utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TalonFXMotorGroup implements AutoCloseable {

    public record MotorConfig(int deviceId, String name, InvertedValue inverted) {}

    private static final class MotorData {
        private final MotorConfig config;
        private final TalonFX device;
        private final Follower followerRequest;

        private boolean lastConnected = false;

        private MotorData(MotorConfig config, TalonFX device, Follower followerRequest) {
            this.config = config;
            this.device = device;
            this.followerRequest = followerRequest;
        }
    }

    private final String groupName;
    private final MotorData[] motors;
    private int leaderIndex = -1;
    private ControlRequest lastControlRequest;
    private boolean lostAllMotors = false;

    private final Alert errorAlert = new Alert("", AlertType.kError);
    private final Alert lostAllMotorPositionsAlert = new Alert("", AlertType.kWarning);

    public TalonFXMotorGroup(String groupName, MotorConfig... motors) {
        this.groupName = groupName;

        lostAllMotorPositionsAlert.setText(groupName + " lost all motor positions!");

        if (motors.length == 0) {
            throw new IllegalArgumentException("Must have at least one motor in the group");
        }

        this.motors = new MotorData[motors.length];
        for (int i = 0; i < motors.length; i++) {
            this.motors[i] = new MotorData(
                    motors[i], new TalonFX(motors[i].deviceId()), new Follower(-1, MotorAlignmentValue.Aligned));
        }

        periodic();
    }

    public void applyConfig(TalonFXConfiguration config) {
        for (MotorData motor : motors) {
            motor.device
                    .getConfigurator()
                    .apply(config.withMotorOutput(config.MotorOutput.withInverted(motor.config.inverted)));
        }
    }

    public void setControl(ControlRequest request) {
        lastControlRequest = request;
        if (leaderIndex != -1) {
            motors[leaderIndex].device.setControl(request);
        }
    }

    public void updateFollower(MotorData motor) {
        motor.device.setControl(motor.followerRequest
                .withLeaderID(motors[leaderIndex].config.deviceId)
                .withMotorAlignment(
                        motors[leaderIndex].config.inverted == motor.config.inverted
                                ? MotorAlignmentValue.Aligned
                                : MotorAlignmentValue.Opposed));
    }

    public void checkLeaderDisconnect() {
        if (!motors[leaderIndex].device.isConnected()) {
            motors[leaderIndex].lastConnected = false;

            leaderIndex = findLeader();

            for (int i = 0; i < motors.length; i++) {
                if (i != leaderIndex) {
                    updateFollower(motors[i]);
                }
            }

            setControl(lastControlRequest);
        }
    }

    public void checkFollowerReconnect() {
        for (int i = 0; i < motors.length; i++) {
            if (i != leaderIndex && !motors[i].lastConnected && motors[i].device.isConnected()) {
                motors[i].device.setPosition(getAveragePosition());
                motors[i].lastConnected = true;
                updateFollower(motors[i]);
            }
        }
    }

    public final int findLeader() {
        for (int i = 0; i < motors.length; i++) {
            if (motors[i].device.isConnected()) {
                return i;
            }
        }
        return -1;
    }

    public double getAveragePosition() {
        return SimpleMath.average(getPositions());
    }

    public double[] getPositions() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.device.isConnected())
                .mapToDouble(m -> m.device.getPosition().getValueAsDouble())
                .toArray();
    }

    public double[] getVelocities() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.device.isConnected())
                .mapToDouble(m -> m.device.getVelocity().getValueAsDouble())
                .toArray();
    }

    public Current[] getCurrents() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.device.isConnected())
                .map(m -> m.device.getSupplyCurrent().getValue())
                .toArray(Current[]::new);
    }

    public double[] getVoltages() {
        return Arrays.stream(motors)
                .filter(m -> (m.lastConnected || lostAllMotors) && m.device.isConnected())
                .mapToDouble(m -> m.device.getMotorVoltage().getValueAsDouble())
                .toArray();
    }

    public void setPosition(double position) {
        if (leaderIndex != -1) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].device.setPosition(position);
            }
            lostAllMotors = false;
        }
    }

    public void updateAlert() {
        if (leaderIndex == -1) {
            errorAlert.setText(groupName + " no leader!");
            errorAlert.set(true);
        } else {
            List<String> disconnectedMotors = new ArrayList<>();
            for (int i = 0; i < motors.length; i++) {
                if (!motors[i].device.isConnected()) {
                    disconnectedMotors.add(motors[i].config.name);
                }
            }

            if (!disconnectedMotors.isEmpty()) {
                errorAlert.setText(groupName + " " + String.join(",", disconnectedMotors) + " disconnected!");
                errorAlert.set(true);
            } else {
                errorAlert.set(false);
            }
        }

        lostAllMotorPositionsAlert.set(lostAllMotors);
    }

    public boolean hasLostPosition() {
        return lostAllMotors;
    }

    public final void periodic() {
        if (leaderIndex == -1) {
            leaderIndex = findLeader();
            if (leaderIndex != -1) {
                setControl(lastControlRequest);
            }
        }

        updateAlert();

        if (leaderIndex != -1) {
            checkLeaderDisconnect();
            checkFollowerReconnect();
        } else {
            lostAllMotors = true;
        }

        for (int i = 0; i < motors.length; i++) {
            motors[i].lastConnected = motors[i].device.isConnected();
        }
    }

    public TalonFXSimState getSimState(int index) {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            throw new UnsupportedOperationException("Cannot get sim state of real motor");
        }
        return motors[index].device.getSimState();
    }

    public TalonFXSimState[] getSimStates() {
        if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REAL) {
            throw new UnsupportedOperationException("Cannot get sim state of real motor");
        }
        return Arrays.stream(motors).map(m -> m.device.getSimState()).toArray(TalonFXSimState[]::new);
    }

    public TalonFX getMotor(int index) {
        return motors[index].device;
    }

    @Override
    public void close() {
        for (int i = 0; i < motors.length; i++) {
            motors[i].device.close();
        }
    }
}
