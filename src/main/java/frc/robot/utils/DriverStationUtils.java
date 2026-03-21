package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.OptionalDouble;

public final class DriverStationUtils {

    private static boolean inPracticeMode = false;

    private DriverStationUtils() {}

    public static Alliance getCurrentAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get();
        }
        return Alliance.Blue;
    }

    @SuppressWarnings("java:S1244") // matchTime is exactly -1
    public static void teleopInit() {
        inPracticeMode = false;

        // Practice mode detection is only without FMS
        if (DriverStation.isFMSAttached()) return;

        final double teleopInitMatchTime = DriverStation.getMatchTime();
        CommandScheduler.getInstance().schedule(Commands.waitUntil(() -> {
            double matchTime = DriverStation.getMatchTime();

            if (matchTime == -1) {
                // Match time not valid, can't be in practice mode
                return true;
            }

            if (Math.abs(teleopInitMatchTime - matchTime) < 1e-4) return false; // Wait for update

            // After update, make sure the match time goes down instead of up
            if (matchTime < teleopInitMatchTime) {
                inPracticeMode = true;
            }

            return true;
        }));
    }

    public static boolean isInPracticeMode() {
        return inPracticeMode;
    }

    public record MatchTimeData(boolean hubActive, double timeLeftInShift) {}

    public static MatchTimeData isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return new MatchTimeData(false, 0);
        }

        double matchTime = DriverStation.getMatchTime();

        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return new MatchTimeData(true, matchTime);
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return new MatchTimeData(false, 25);
        }

        // We're teleop enabled, compute.
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its likely early in teleop.
        if (gameData.isEmpty()) {
            return new MatchTimeData(true, 0);
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return new MatchTimeData(true, 0);
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active =
                switch (alliance.get()) {
                    case Red -> !redInactiveFirst;
                    case Blue -> redInactiveFirst;
                };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return new MatchTimeData(true, matchTime - 130);
        } else if (matchTime > 105) {
            // Shift 1
            return new MatchTimeData(shift1Active, matchTime - 105);
        } else if (matchTime > 80) {
            // Shift 2
            return new MatchTimeData(!shift1Active, matchTime - 80);
        } else if (matchTime > 55) {
            // Shift 3
            return new MatchTimeData(shift1Active, matchTime - 55);
        } else if (matchTime > 30) {
            // Shift 4
            return new MatchTimeData(!shift1Active, matchTime - 30);
        } else {
            // End game, hub always active.
            return new MatchTimeData(true, matchTime);
        }
    }

    /**
     * Returns the time remaining in the TELEOP period (in rounded INTEGER seconds)
     * <hr>
     * <p>
     * If the current period is not teleop, returns empty
     * </p>
     * <p>
     * If NOT connected to FMS (driver practice):
     * <ul>
     *     <li>If in practice mode returns time remaining as usual</li>
     *     <li>If NOT in practice mode returns empty
     *            (no defined remaining match time since
     *            could be enabled for any amount of time)</li>
     * </ul>
     * </p>
     * @return the remaining match time in the TELEOP period
     * (in rounded INTEGER seconds) or empty if undefined
     */
    public static OptionalDouble getTeleopMatchTime() {
        if (!DriverStation.isTeleop()) return OptionalDouble.empty(); // Not in teleop

        if (!DriverStation.isFMSAttached() && !isInPracticeMode())
            return OptionalDouble.empty(); // If no fms, empty if not in practice mode

        // FMS is attached or in practice mode
        double matchTime = DriverStation.getMatchTime();
        return OptionalDouble.of(matchTime);
    }
}
