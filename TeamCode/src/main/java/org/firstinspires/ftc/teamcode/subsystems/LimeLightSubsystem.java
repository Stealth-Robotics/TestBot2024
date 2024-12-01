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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.Deque;
import java.util.LinkedList;

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

}
