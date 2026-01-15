package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
        Commands.waitUntil(() -> {
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
                })
                .schedule();
    }

    public static boolean isInPracticeMode() {
        return inPracticeMode;
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
