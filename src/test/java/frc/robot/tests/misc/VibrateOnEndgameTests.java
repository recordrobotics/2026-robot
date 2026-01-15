package frc.robot.tests.misc;

import static edu.wpi.first.wpilibj.simulation.DriverStationSim.getMatchTime;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.notifyNewData;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAllianceStationId;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setAutonomous;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setEnabled;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setFmsAttached;
import static edu.wpi.first.wpilibj.simulation.DriverStationSim.setMatchTime;
import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static utils.TestRobot.*;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.VibrateXbox;
import java.lang.reflect.Field;
import java.util.Set;
import org.junit.jupiter.api.Test;
import utils.BooleanSupplierEx;

class VibrateOnEndgameTests {

    BooleanSupplierEx stopOnVibrate() {
        return stopOnCommandInit(cmd -> {
            if (cmd instanceof ParallelRaceGroup pr) {
                try {
                    Field field = ParallelRaceGroup.class.getDeclaredField("m_commands");
                    field.setAccessible(true);
                    @SuppressWarnings("unchecked")
                    Set<Command> commands = (Set<Command>) field.get(pr);
                    return commands.stream().anyMatch(VibrateXbox.class::isInstance);
                } catch (NoSuchFieldException | SecurityException | IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
            }

            return false;
        });
    }

    boolean periodic() {
        setMatchTime(getMatchTime() - 0.02);
        notifyNewData();
        return true;
    }

    @Test
    void testVibrateWithFMS() {
        testUntil(
                stopOnVibrate(),
                this::periodic,
                robot -> {
                    setAllianceStationId(AllianceStationID.Blue1);
                    setFmsAttached(true);
                    setMatchTime(23);
                    setEnabled(true);
                    notifyNewData();
                },
                () -> assertTrue(
                        controlBridge().getRumble(RumbleType.kBothRumble) > 0, "Controller should be vibrating"));
    }

    @Test
    void testNoVibrateInAutoWithFMS() {
        testUntil(
                stopOnVibrate().or(() -> getMatchTime() < 13),
                this::periodic,
                robot -> {
                    setAllianceStationId(AllianceStationID.Blue1);
                    setFmsAttached(true);
                    setMatchTime(15);
                    setAutonomous(true);
                    setEnabled(true);
                    notifyNewData();
                },
                () -> assertEquals(
                        0, controlBridge().getRumble(RumbleType.kBothRumble), "Controller should NOT be vibrating"));
    }

    @Test
    void testVibrateInPracticeWithoutFMS() {
        testUntil(
                stopOnVibrate(),
                this::periodic,
                robot -> {
                    setAllianceStationId(AllianceStationID.Blue1);
                    setMatchTime(23);
                    setEnabled(true);
                    notifyNewData();
                },
                () -> assertTrue(
                        controlBridge().getRumble(RumbleType.kBothRumble) > 0, "Controller should be vibrating"));
    }

    @Test
    void testNoVibrateInTeleopWithoutFMS() {
        testUntil(
                stopOnVibrate().or(() -> getMatchTime() > 4),
                () -> {
                    setMatchTime(getMatchTime() + 0.02);
                    notifyNewData();
                    return true;
                },
                robot -> {
                    setAllianceStationId(AllianceStationID.Blue1);
                    setMatchTime(0);
                    setEnabled(true);
                    notifyNewData();
                },
                () -> assertEquals(
                        0, controlBridge().getRumble(RumbleType.kBothRumble), "Controller should NOT be vibrating"));
    }
}
