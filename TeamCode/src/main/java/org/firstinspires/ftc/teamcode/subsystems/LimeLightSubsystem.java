package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.MovementUtil.getFollowPoseFromLimelight;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedList;
import java.util.List;

/**
 * Wraps the Limelight camera in a subsystem for easier use
 */
public class LimeLightSubsystem extends SubsystemBase {

    // Name of the hardware device
    public static final String NAME = "limelight";

    // The length of time to keep LLResults in the queue before removing them.
    private static final long QUEUE_DEFAULT_TIMEOUT = 500; // X milliseconds

    // How many degrees back is your limelight rotated from perfectly vertical?
    private  static final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 0.0;

    // Distance from the center of the Limelight lens to the floor
    private static final double LIMELIGHT_FLOOR_HEIGHT_INCHES = 4.25;

    // Used to calculate distance from an APRIL tag
    private static final double APRIL_TAG_CENTER_HEIGHT_INCHES = 6.125;

    // Used to calculate distance from a sample target laying flat on the floor
    private static final double SAMPLE_TARGET_CENTER_HEIGHT_INCHES = .75;

    private final Telemetry telemetryA;
    private final Limelight3A limelight;
    private LLResult lastResult;

    // A queue of the last few LLResults in the last QUEUE_DEFAULT_TIMEOUT window
    private final Deque<LLResult> resultsQueue = new LinkedList<>();

    /**
     * Initialize the limelight camera and default to pipeline 0
     * @param hardwareMap hardware map
     * @param telemetry telemetry object to write data to the hub
     */
    public LimeLightSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, NAME);
        limelight.start();
        limelight.pipelineSwitch(Pipeline.APRIL_TAG.id);
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    /**
     * Gets the last target seen of the selected pipeline
     * @return last LLResult or null if none
     */
    public LLResult getLastResult() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
           this.lastResult = result;
           this.addResultToQueue(result);
        }

        return lastResult;
    }

    /**
     * Gets direct access to the Limelight3A object
     * @return Limelight3A object
     */
    public Limelight3A getLimelight(){
        return limelight;
    }

    /**
     * Switches the Limelight pipeline
     * @param pipeline Enum of the pipeline to switch to
     */
    public void setPipeline(@NonNull Pipeline pipeline) {
        limelight.pipelineSwitch(pipeline.id);
        this.lastResult = null;
        resultsQueue.clear();
    }

    /**
     * Returns the LLResult queue. for doing averaging or prediction
     * @return Deque of LLResults
     */
    public Deque<LLResult> getResults() {
        return resultsQueue;
    }

    /**
     * Adds a LLResult to the queue
     * @param result LLResult to add
     */
    private void addResultToQueue(LLResult result) {

        resultsQueue.addLast(result);
        // Remove elements older than QUEU_DEFAULT_TIMEOUT
        while (!resultsQueue.isEmpty()
                && (resultsQueue.peekFirst() != null)
                && (resultsQueue.peekFirst().getStaleness() > QUEUE_DEFAULT_TIMEOUT)) {
            resultsQueue.removeFirst();
        }
    }

    /**
     * Calculates the distance to the target in inches based on the current pipeline
     * @return distance in inches
     */
    public double getResultDistanceInches() {
        LLResult result = getLastResult();
        double targetHeight = 0;
        double distance = -1;
        if (result != null && result.isValid() && result.getStaleness() < 100) {
            Pipeline pipeline = Pipeline.values()[result.getPipelineIndex()];
            switch (pipeline) {
                case APRIL_TAG:
                    targetHeight = APRIL_TAG_CENTER_HEIGHT_INCHES;
                    break;
                case BLUE_SPEC:
                case RED_SPEC:
                case YELLOW_SPEC:
                    targetHeight = SAMPLE_TARGET_CENTER_HEIGHT_INCHES;
                    break;
            }

            double targetOffsetAngle = result.getTy();
            double angleToObjectDegrees = LIMELIGHT_MOUNT_ANGLE_DEGREES + targetOffsetAngle;
            double angleToGoalRadians = Math.toRadians(angleToObjectDegrees); //  (3.14159 / 180.0);
            distance = (targetHeight - LIMELIGHT_FLOOR_HEIGHT_INCHES) / Math.tan(angleToGoalRadians);
        }

        return distance;
    }

    /**
     * Switches the pipeline to the next one in the enum
     */
    public void togglePipeline(){
        Pipeline cur = Pipeline.values()[limelight.getStatus().getPipelineIndex()];
        switch (cur){
            case APRIL_TAG:
                setPipeline(Pipeline.BLUE_SPEC);
                break;
            case BLUE_SPEC:
                setPipeline(Pipeline.RED_SPEC);
                break;
            case RED_SPEC:
                setPipeline(Pipeline.YELLOW_SPEC);
                break;
            case YELLOW_SPEC:
                setPipeline(Pipeline.APRIL_TAG);
                break;
        }
    }

    /**
     * Gets called periodically by the scheduler. Currently this is only
     * providing spew back to the hub and could be disable for match play
     */
    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();

        Pipeline pipeline = Pipeline.values()[limelight.getStatus().getPipelineIndex()];
        telemetryA.addData("Pipeline", pipeline.toString());

        if (result != null && result.isValid() && result.getStaleness() < 50) {
            telemetryA.addData("CAM Latency", result.getStaleness());
            telemetryA.addData("CAM distance", getResultDistanceInches());
            telemetryA.addData("Cam tx", result.getTx());
            telemetryA.addData("Cam ty", result.getTy());
            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                Pose convertedPose = getFollowPoseFromLimelight(botPose);
                telemetryA.addData("Bot Pose Fixed X:", convertedPose.getX());
                telemetryA.addData("Bot Pose Fixed Y:", convertedPose.getY());
                telemetryA.addData("Bot Heading Fixed: ", Math.toDegrees(convertedPose.getHeading()));

                //telemetryA.addData("Bot Pose RAW X", curPos.x);
                //telemetryA.addData("Bot Pose RAW Y", curPos.y);
                //telemetryA.addData("Bot Heading Raw:   ", angles.getYaw(AngleUnit.DEGREES));

            }
        }
    }

    /**
     * Calculates the average of the last few LLResults in the queue
     * @return Pose3D of the average
     */
    public Pose3D getAveragePose3D() {
        List<Pose3D> poses = new ArrayList<>();
        List<Double> xValues = new ArrayList<>();
        List<Double> yValues = new ArrayList<>();
        List<Double> yawValues = new ArrayList<>(); // List to store yaw values

        // Extract Pose3D, calculate x, y, and yaw averages
        for (LLResult result : resultsQueue) {
            if (result.isValid() && result.getStaleness() < QUEUE_DEFAULT_TIMEOUT) {
                Pose3D pose3D = result.getBotpose();
                if (pose3D != null) {
                    poses.add(pose3D);
                    xValues.add(pose3D.getPosition().x);
                    yValues.add(pose3D.getPosition().y);

                    // Get yaw and add to list
                   yawValues.add(pose3D.getOrientation().getYaw()); // Assuming firstAngle is yaw
                }
            }
        }

        if (poses.isEmpty()) {
            return null; // Handle empty case
        }

        // Calculate average and standard deviation for x, y, and yaw
        double xAvg = calculateAverage(xValues);
        double yAvg = calculateAverage(yValues);
        double yawAvg = calculateAverage(yawValues); // Average yaw
        double xStdDev = calculateStandardDeviation(xValues, xAvg);
        double yStdDev = calculateStandardDeviation(yValues, yAvg);
        double yawStdDev = calculateStandardDeviation(yawValues, yawAvg); // Standard deviation for yaw

        // Filter outliers, including yaw in the condition
        List<Pose3D> filteredPoses = new LinkedList<>();
        for (int i = 0; i < poses.size(); i++) {
            Pose3D pose = poses.get(i);
            if (Math.abs(pose.getPosition().x - xAvg) <= 2 * xStdDev &&
                    Math.abs(pose.getPosition().y - yAvg) <= 2 * yStdDev &&
                    Math.abs(yawValues.get(i) - yawAvg) <= 2 * yawStdDev) { // Check yaw outlier
                filteredPoses.add(pose);
            }
        }

        // Calculate average of filtered poses, including yaw
        double filteredXAvg = calculateAveragePose3D(filteredPoses, "x");
        double filteredYAvg = calculateAveragePose3D(filteredPoses, "y");
        double filteredYawAvg = calculateAverageYaw(filteredPoses); // Average filtered yaw

        // Use the last Pose3d as a template to create a new one with the average values
        Pose3D lastPose3d = resultsQueue.peekLast() != null ? resultsQueue.peekLast().getBotpose() : null;
        if (lastPose3d == null) {
            return null;
        }

        return new Pose3D(
                new Position(
                        lastPose3d.getPosition().unit,
                        filteredXAvg,
                        filteredYAvg,
                        lastPose3d.getPosition().z,
                        lastPose3d.getPosition().acquisitionTime),
                new YawPitchRollAngles(
                        AngleUnit.DEGREES,
                        filteredYawAvg,
                        lastPose3d.getOrientation().getPitch(),
                        lastPose3d.getOrientation().getRoll(),
                        lastPose3d.getOrientation().getAcquisitionTime()));
    }

    /**
     * Calculates the average of a list of values
     * @param values list of values
     * @return average
     */
    private static double calculateAverage(List<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Double value : values) {
            sum += value;
        }
        return sum / values.size();
    }

    /**
     * Calculates the standard deviation of a list of values
     * @param values list of values
     * @param average average of the values
     * @return standard deviation
     */
    private static double calculateStandardDeviation(List<Double> values, double average) {
        if (values.isEmpty()) {
            return 0.0;
        }
        double sumOfSquaredDeviations = 0.0;
        for (Double value : values) {
            sumOfSquaredDeviations += Math.pow(value - average, 2);
        }
        return Math.sqrt(sumOfSquaredDeviations / (values.size() - 1));
    }

    /**
     * Calculates the average of a list of Pose3D objects
     * @param poses list of Pose3D objects
     * @param coordinate x or y
     * @return average
     */
    private static double calculateAveragePose3D(List<Pose3D> poses, String coordinate) {
        if (poses.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Pose3D pose : poses) {
            if (coordinate.equals("x")) {
                sum += pose.getPosition().x;
            } else if (coordinate.equals("y")) {
                sum += pose.getPosition().y;
            }
        }
        return sum / poses.size();
    }

    /**
     * Calculates the average yaw of a list of Pose3D objects
     * @param poses list of Pose3D objects
     * @return average yaw
     */
    private static double calculateAverageYaw(List<Pose3D> poses) {
        if (poses.isEmpty()) {
            return 0.0;
        }
        double sum = 0.0;
        for (Pose3D pose : poses) {
            sum += pose.getOrientation().getYaw();
        }
        return sum / poses.size();
    }
}
