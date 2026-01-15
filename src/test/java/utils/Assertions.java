package utils;

import static org.junit.jupiter.api.AssertionFailureBuilder.*;

import frc.robot.utils.maplesim.Arena2025ReefscapeWithAlgae;
import frc.robot.utils.maplesim.ReefscapeReefAlgaeSide;
import frc.robot.utils.maplesim.ReefscapeReefAlgaeSimulation;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefBranch;

public class Assertions {

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of corals, where each coral
     * is represented by its branch ID (e.g., "BA4", "RC3") followed by an optional
     * count (e.g., "BA40" - blue A4 has 0 coral, "RB12" - red B1 has 2 coral).
     * Full reefstring example: "BG4,BH4,RA12" - blue G4 has 1 coral, blue H4 has 1 coral, red A1 has 2 coral.
     * @implNote Only asserts that the reef contains the specified corals, but does not check if all other branches are empty.
     * @param reefString the expected reef string
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefCoralAtLeast(String reefString) {
        assertReefCoral(reefString, false);
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of corals, where each coral
     * is represented by its branch ID (e.g., "BA4", "RC3") followed by an optional
     * count (e.g., "BA40" - blue A4 has 0 coral, "RB12" - red B1 has 2 coral).
     * Full reefstring example: "BG4,BH4,RA12" - blue G4 has 1 coral, blue H4 has 1 coral, red A1 has 2 coral.
     * @implNote Also asserts that all other branches not mentioned in the reefString are empty.
     * @param reefString the expected reef string
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefCoralExactly(String reefString) {
        assertReefCoral(reefString, true);
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of corals, where each coral
     * is represented by its branch ID (e.g., "BA4", "RC3") followed by an optional
     * count (e.g., "BA40" - blue A4 has 0 coral, "RB12" - red B1 has 2 coral).
     * Full reefstring example: "BG4,BH4,RA12" - blue G4 has 1 coral, blue H4 has 1 coral, red A1 has 2 coral.
     * @param reefString the expected reef string
     * @param assertRestAreEmpty if true, also asserts that all other branches not mentioned in the reefString are empty
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefCoral(String reefString, boolean assertRestAreEmpty) {
        ArrayList<String> corals = new ArrayList<>(Arrays.asList(reefString.split(",")));
        ArrayList<String> coralIds = new ArrayList<>(
                corals.stream().map(coral -> coral.trim().substring(0, 3)).toList());

        // also assert rest of the branches are empty
        if (assertRestAreEmpty) {
            for (Map.Entry<String, ReefscapeReefBranch> entry :
                    ((Arena2025Reefscape) SimulatedArena.getInstance()).blueReefSimulation.getBranchesSet()) {
                if (!coralIds.contains("B" + entry.getKey())) {
                    corals.add("B" + entry.getKey() + "0");
                }
            }

            for (Map.Entry<String, ReefscapeReefBranch> entry :
                    ((Arena2025Reefscape) SimulatedArena.getInstance()).redReefSimulation.getBranchesSet()) {
                if (!coralIds.contains("R" + entry.getKey())) {
                    corals.add("R" + entry.getKey() + "0");
                }
            }
        }

        StringBuilder sb = new StringBuilder();
        List<String> actualReefString = new ArrayList<>();

        for (String coral : corals) {
            coral = coral.trim();

            ReefscapeReefBranch branch;

            if (coral.charAt(0) == 'B') {
                branch = ((Arena2025Reefscape) SimulatedArena.getInstance())
                        .blueReefSimulation.getBranch(coral.substring(1, 3));
            } else if (coral.charAt(0) == 'R') {
                branch = ((Arena2025Reefscape) SimulatedArena.getInstance())
                        .redReefSimulation.getBranch(coral.substring(1, 3));
            } else {
                throw new IllegalArgumentException("Invalid reef alliance: " + coral);
            }

            if (branch == null) {
                throw new IllegalArgumentException("Invalid reef coral: " + coral);
            }

            int expectedCount;
            if (coral.length() >= 4) {
                expectedCount = Integer.parseInt(coral.substring(3));
            } else {
                expectedCount = 1; // Default to 1 if no count is specified
            }

            int actualCount = branch.getGamePieceCount();

            if (actualCount != expectedCount) {
                String branchId = coral.substring(0, 3);
                actualReefString.add(branchId + actualCount);
                sb.append("Expected branch " + branchId + " to have " + expectedCount + " coral, but it has "
                        + actualCount + ".\n");
            }
        }

        if (!sb.isEmpty()) {
            assertionFailure()
                    .message(sb.toString())
                    .expected(reefString)
                    .actual(String.join(",", actualReefString))
                    .buildAndThrow();
        }
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of algae positions where algae should be MISSING, where each algae position is represented by alliance color [B, R] its 2 branches (e.g., "BAB", "REF")
     * Full reefstring example: "BAB,REF,RGH" - Blue AB does NOT have algae, Red EF does NOT have algae, Red GH does NOT have algae.
     * @implNote Only asserts that the reef does NOT contain the specified algae, but does NOT check if all other positions are full.
     * @param reefString the expected reef string
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid algae positions
     */
    public static void assertReefAlgaeAtLeastMissing(String reefString) {
        assertReefAlgaeMissing(reefString, false);
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of algae positions where algae should be MISSING, where each algae position is represented by alliance color [B, R] its 2 branches (e.g., "BAB", "REF")
     * Full reefstring example: "BAB,REF,RGH" - Blue AB does NOT have algae, Red EF does NOT have algae, Red GH does NOT have algae.
     * @implNote Also asserts that all other algae positions not mentioned in the reefString have algae.
     * @param reefString the expected reef string
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefAlgaeExactlyMissing(String reefString) {
        assertReefAlgaeMissing(reefString, true);
    }

    /**
     * Asserts that the reef matches the expected string format.
     * The expected format is a comma-separated list of algae positions where algae should be MISSING, where each algae position is represented by alliance color [B, R] its 2 branches (e.g., "BAB", "REF")
     * Full reefstring example: "BAB,REF,RGH" - Blue AB does NOT have algae, Red EF does NOT have algae, Red GH does NOT have algae.
     * @param reefString the expected reef string
     * @param assertRestAreFull if true, also asserts that all other algae positions not mentioned in the reefString have algae.
     * @throws IllegalArgumentException if the reefString is not in the expected format or contains invalid coral IDs
     */
    public static void assertReefAlgaeMissing(String reefString, boolean assertRestAreFull) {
        ArrayList<String> expectedMissingAlgaePositions = new ArrayList<>(Arrays.asList(reefString.split(",")));

        StringBuilder sb = new StringBuilder();
        List<String> actualReefString = new ArrayList<>();

        for (Map.Entry<String, ReefscapeReefAlgaeSimulation> allianceAlgaeSim : Arrays.asList(
                Map.entry("B", ((Arena2025ReefscapeWithAlgae) SimulatedArena.getInstance()).blueAlgaeSimulation),
                Map.entry("R", ((Arena2025ReefscapeWithAlgae) SimulatedArena.getInstance()).redAlgaeSimulation))) {
            for (Map.Entry<String, ReefscapeReefAlgaeSide> side :
                    allianceAlgaeSim.getValue().getSidesSet()) {
                String algaePositionID = allianceAlgaeSim.getKey() + side.getKey();
                boolean expectedToBeMissing = expectedMissingAlgaePositions.contains(algaePositionID);
                boolean actualIsThere = side.getValue().isFull();
                if (expectedToBeMissing && actualIsThere) {
                    sb.append("Expected algae position " + algaePositionID + " to be missing, but it is present.\n");
                } else if (!expectedToBeMissing && !actualIsThere && assertRestAreFull) {
                    sb.append("Expected algae position " + algaePositionID + " to be present, but it is missing.\n");
                }
                expectedMissingAlgaePositions.remove(algaePositionID);
            }
        }

        if (!expectedMissingAlgaePositions.isEmpty()) {
            sb.append("Invalid algae positions: " + String.join(",", expectedMissingAlgaePositions) + "\n");
        }

        if (!sb.isEmpty()) {
            assertionFailure()
                    .message(sb.toString())
                    .expected(reefString)
                    .actual(String.join(",", actualReefString))
                    .buildAndThrow();
        }
    }
}
