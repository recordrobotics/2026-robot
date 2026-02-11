package frc.robot.subsystems;

import com.google.common.collect.ImmutableSortedMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Constants;
import frc.robot.utils.ConsoleLogger;
import frc.robot.utils.ManagedSubsystemBase;
import frc.robot.utils.camera.Cameras;
import frc.robot.utils.camera.GenericCamera;
import frc.robot.utils.camera.PhysicalCamera;
import frc.robot.utils.camera.objectdetection.ObjectDetectionCamera;
import frc.robot.utils.camera.objectdetection.ObjectDetectionClass;
import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import org.littletonrobotics.junction.Logger;

public class FieldStateTracker extends ManagedSubsystemBase {

    /**
     * The cameras used for vision measurements
     */
    private final Set<ObjectDetectionCamera> cameras = Set.of(
            /*
             * Intake Left
             */
            Cameras.createPhotonVisionObjectDetectionCamera(
                    Constants.Vision.INTAKE_LEFT_NAME,
                    PhysicalCamera.ARDUCAM_160_FOV,
                    Constants.Vision.ROBOT_TO_CAMERA_INTAKE_LEFT,
                    ImmutableSortedMap.<Integer, ObjectDetectionClass>naturalOrder()
                            .put(1, ObjectDetectionClass.BUMPER)
                            .put(2, ObjectDetectionClass.FUEL)
                            .build()),
            /*
             * Intake Right
             */
            Cameras.createPhotonVisionObjectDetectionCamera(
                    Constants.Vision.INTAKE_RIGHT_NAME,
                    PhysicalCamera.ARDUCAM_160_FOV,
                    Constants.Vision.ROBOT_TO_CAMERA_INTAKE_RIGHT,
                    ImmutableSortedMap.<Integer, ObjectDetectionClass>naturalOrder()
                            .put(1, ObjectDetectionClass.BUMPER)
                            .put(2, ObjectDetectionClass.FUEL)
                            .build()));

    /**
     * The deferred pose estimations to be added at the end of the calculation phase
     */
    private ConcurrentLinkedQueue<DeferredObjectDetection> deferredObjectDetections = new ConcurrentLinkedQueue<>();

    /**
     * Executor service for asynchronous calculations
     */
    private ExecutorService executor = Executors.newSingleThreadExecutor();
    /**
     * Future for the current calculation task
     */
    private Future<?> calculationFuture = null;

    private Set<Pose2d> fieldObjects = new HashSet<>();

    public void addObjectDetectionUpdate(
            Pose2d pose, ObjectDetectionClass detectionClass, double confidence, double timestamp) {
        deferredObjectDetections.add(new DeferredObjectDetection(pose, detectionClass, confidence, timestamp));
    }

    public record DeferredObjectDetection(
            Pose2d pose, ObjectDetectionClass detectionClass, double confidence, double timestampSeconds) {}

    /**
     * Starts the asynchronous calculation phase
     */
    public void startCalculation() {
        deferredObjectDetections.clear();
        calculationFuture = executor.submit(this::performCalculation);
    }

    /**
     * Waits for the end of the calculation and applies all updates
     */
    public void endCalculation() {
        if (calculationFuture != null) {
            try {
                calculationFuture.get();
            } catch (ExecutionException e) {
                ConsoleLogger.logError(e);
            } catch (InterruptedException e) {
                ConsoleLogger.logError(e);
                Thread.currentThread().interrupt();
            }
        }

        fieldObjects.clear();
        while (!deferredObjectDetections.isEmpty()) {
            DeferredObjectDetection detection = deferredObjectDetections.poll();
            if (detection != null) {
                fieldObjects.add(detection.pose());
            }
        }

        logValues();
    }

    /**
     * Logs values to the logger
     */
    private void logValues() {
        Logger.recordOutput(
                "FieldStateTracker/FieldObjects",
                fieldObjects.stream().map(Pose3d::new).toArray(Pose3d[]::new));

        cameras.stream().forEach(GenericCamera::logValues);
    }

    @Override
    public void periodicManaged() {
        /* logic is asynchronous */
    }

    /**
     * Performs the asynchronous calculation for the pose sensor fusion
     */
    public void performCalculation() {
        cameras.stream().forEach(GenericCamera::periodic);
        cameras.stream().forEach(camera -> camera.addObjectDetectionUpdates(this));

        /* TODO: perform logic */
    }
}
