package build.utils.pathgenerator;

import build.utils.FileUtils;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import edu.wpi.first.math.geometry.Pose2d;
import java.io.File;
import java.io.FileNotFoundException;

public final class PathHelper {

    private static final String NEXT_CONTROL = "nextControl";
    private static final String PREV_CONTROL = "prevControl";
    private static final String LINKED_NAME = "linkedName";
    private static final String ANCHOR = "anchor";
    private static final String IDEAL_STARTING_STATE = "idealStartingState";
    private static final String GOAL_END_STATE = "goalEndState";
    private static final String ROTATION = "rotation";
    private static final String X = "x";
    private static final String Y = "y";

    private static int counter = 0;

    private PathHelper() {}

    public static int getCounter() {
        return counter;
    }

    public static void editLinkedWaypoint(String name, Pose2d pose) throws FileNotFoundException {
        for (File path : new File(FileUtils.getLaunchDirectory(), "src/main/deploy/pathplanner/paths").listFiles()) {
            JsonNode obj = FileUtils.readJson(path);
            processWaypoints(obj, name, pose);
            FileUtils.writeJson(path, obj, new PathPlannerPrettyPrinter());
        }
    }

    private static void processWaypoints(JsonNode obj, String name, Pose2d pose) {
        for (JsonNode waypoint : obj.get("waypoints")) {
            if (isLinkedWaypoint(waypoint, name)) {
                counter++;
                updateWaypoint(obj, waypoint, pose);
            }
        }
    }

    private static boolean isLinkedWaypoint(JsonNode waypoint, String name) {
        return waypoint.get(LINKED_NAME) != null
                && name.equals(waypoint.get(LINKED_NAME).asText());
    }

    private static void updateWaypoint(JsonNode obj, JsonNode waypoint, Pose2d pose) {
        updateNextControl(obj, waypoint, pose);
        updatePrevControl(obj, waypoint, pose);
        updateAnchor(waypoint, pose);
    }

    private static void updateNextControl(JsonNode obj, JsonNode waypoint, Pose2d pose) {
        if (waypoint.has(NEXT_CONTROL) && waypoint.get(NEXT_CONTROL).isObject()) {
            if (isFirstWaypoint(waypoint)) {
                ((ObjectNode) obj.get(IDEAL_STARTING_STATE))
                        .put(ROTATION, pose.getRotation().getDegrees());
            }
            updateControlHandle(waypoint, pose, NEXT_CONTROL);
        }
    }

    private static void updatePrevControl(JsonNode obj, JsonNode waypoint, Pose2d pose) {
        if (waypoint.has(PREV_CONTROL) && waypoint.get(PREV_CONTROL).isObject()) {
            if (isLastWaypoint(waypoint)) {
                ((ObjectNode) obj.get(GOAL_END_STATE))
                        .put(ROTATION, pose.getRotation().getDegrees());
            }
            updateControlHandle(waypoint, pose, PREV_CONTROL);
        }
    }

    private static void updateControlHandle(JsonNode waypoint, Pose2d pose, String controlType) {
        double controlHandleOffsetX = waypoint.get(controlType).get(X).asDouble()
                - waypoint.get(ANCHOR).get(X).asDouble();
        double controlHandleOffsetY = waypoint.get(controlType).get(Y).asDouble()
                - waypoint.get(ANCHOR).get(Y).asDouble();

        ((ObjectNode) waypoint.get(controlType)).put(X, pose.getX() + controlHandleOffsetX);
        ((ObjectNode) waypoint.get(controlType)).put(Y, pose.getY() + controlHandleOffsetY);
    }

    private static void updateAnchor(JsonNode waypoint, Pose2d pose) {
        ((ObjectNode) waypoint.get(ANCHOR)).put(X, pose.getX());
        ((ObjectNode) waypoint.get(ANCHOR)).put(Y, pose.getY());
    }

    private static boolean isFirstWaypoint(JsonNode waypoint) {
        return !waypoint.has(PREV_CONTROL) || waypoint.get(PREV_CONTROL).isNull();
    }

    private static boolean isLastWaypoint(JsonNode waypoint) {
        return !waypoint.has(NEXT_CONTROL) || waypoint.get(NEXT_CONTROL).isNull();
    }
}
