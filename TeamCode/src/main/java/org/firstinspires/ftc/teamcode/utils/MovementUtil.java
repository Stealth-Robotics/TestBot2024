package org.firstinspires.ftc.teamcode.utils;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class MovementUtil {


    // Limelight uses meters starting at the center of the field
    public static double FIELD_POSE_X_MIN_METERS = -1.8288;
    public static double FIELD_POSE_X_MAX_METERS = 1.8288;
    public static double FIELD_POSE_Y_MIN_METERS = -1.8288;
    public static double FIELD_POSE_Y_MAX_METERS = 1.8288;

    // These need tuning they are for pointing at objects or AprilTags
    public static final double ROTATION_CAMERA_X_THRESHOLD = 2;
    public static final double ALIGNMENT_CAMERA_X_THRESHOLD = 4;
    public static final double MIN_ROTATION_POWER = 0.2;
    public static final double MIN_TRANSLATION_POWER = .4;
    public static final double POWER_ROTATION_KP = 1.0;
    public static final double POWER_TRANSLATION_KP = 10;
    public static final double ALIGNMENT_KP  = .5;

    private static final double LL_ANGLE_DELTA = 90;


    /**
     * Converts a Pose3D that Limelight uses to a Pose that the Follower uses
     */
    @NonNull
    public static Pose getFollowPoseFromLimelight(Pose3D result)
    {
        Pose followPose = new Pose();

        // Camera turning CC from 0 goes to -180
        // Camera turning CW from 0 goes to 180
        double llYaw = result.getOrientation().getYaw(AngleUnit.DEGREES);
        double imuAngle = (llYaw < 0) ? (360 + llYaw + LL_ANGLE_DELTA) : (llYaw + LL_ANGLE_DELTA);

        // Ensure within 0-360 range
        imuAngle %= 360;

        followPose.setHeading(Math.toRadians(imuAngle));

        // limelight starting field position is in the middle of the flied
        // and is in meters. Pedro follower is blue alliance corner and
        // is in inches
        // Pedro.X, Y is 0 to 144 inches
        // Limelight.X, Y is -1.8288 to +1.8288

        // first convert to inches
        Position position = result.getPosition().toUnit(DistanceUnit.INCH);

        // now translate to follower coordinates
        // Note X and Y are swapped in the limelight coordinates
        followPose.setY(position.x + (FollowerConstants.FIELD_SIZE_X_INCHES / 2));
        followPose.setX(-position.y + (FollowerConstants.FIELD_SIZE_Y_INCHES / 2));
        return followPose;
    }

    // The below methods are just some test code playing with pointing
    // and moving based on object detection needs work

    public static Pose getHeadingToObject(Pose curPose, double cameraX) {
        // Define the threshold for the camera's center
        // Define the maximum radian change
        double maxRadianChange = 2;
        double currentHeading = curPose.getHeading();

        // Check if the cameraX is outside the threshold
        if (Math.abs(cameraX) > ROTATION_CAMERA_X_THRESHOLD) {
            // Calculate the adjustment needed based on the distance from the center
            double adjustment = cameraX * .1; // Scale adjustment
            if (Math.abs(adjustment) > maxRadianChange) {
                adjustment = Math.signum(adjustment) * maxRadianChange;
            }

            // Update the heading
            currentHeading -= adjustment; // Subtract adjustment for left turn (negative cameraX)

            // Ensure the heading is within the range [0, 2 * Math.PI]
            if (currentHeading < 0) {
                currentHeading += 2 * Math.PI;
            } else if (currentHeading >= 2 * Math.PI) {
                currentHeading -= 2 * Math.PI;
            }

            currentHeading %= (2 * Math.PI);
            return new Pose(curPose.getX(), curPose.getY(), currentHeading);
        }

        return curPose;
    }

    public static Pose getAlignmentPoseToObject(Pose curPos, double cameraX) {
        // Define the threshold for the camera's center

        // Define the maximum lateral shift allowed
        double maxLateralShift = 50; // Adjust as needed

        // Check if the cameraX is outside the threshold
        if (Math.abs(cameraX) > ALIGNMENT_CAMERA_X_THRESHOLD) {
            // Calculate the lateral shift needed
            double lateralShift = cameraX * ALIGNMENT_KP; // Adjust scaling factor as needed

            // Limit the lateral shift to the maximum allowed
            lateralShift = Math.min(Math.max(lateralShift, -maxLateralShift), maxLateralShift);

            // Calculate the shift in X and Y based on the robot's heading
            double shiftX = lateralShift * Math.cos(curPos.getHeading() + Math.PI / 2); // Shift perpendicular to heading
            double shiftY = lateralShift * Math.sin(curPos.getHeading() + Math.PI / 2);

            // Calculate the new X and Y positions
            double newX = curPos.getX() - shiftX;
            double newY = curPos.getY() - shiftY;

            // Update the pose
            return new Pose(newX, newY, curPos.getHeading());
        }

        // If the cameraX is within the threshold, return the current pose
        return curPos;
    }

    public static double calculateMotorPower(double cameraX, double minMotorPower, double motorKp) {
        // Define the camera's X-axis range
        double cameraXRange = 31;

        // Define the motor power range
        double maxMotorPower = 1.0;

        // Calculate the scaled motor power
        double scaledPower = minMotorPower + (maxMotorPower - minMotorPower) * (Math.abs(cameraX) / cameraXRange) * motorKp;

        // Limit the scaled power to the motor power range
        scaledPower = Math.min(Math.max(scaledPower, minMotorPower), maxMotorPower);

        return scaledPower;
    }

    public static Pose3D averagePose3DWithOutlierRemoval(List<Pose3D> poses) {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumHeading = 0.0;
        int count = 0;
        long latestAcquisitionTime = 0;
        // TODO: filter out old entries etc.
        //long currentTime = System.currentTimeMillis();
        // Outlier detection
        double medianX = median(poses, 0);
        double medianY = median(poses, 1);
        double medianHeading = median(poses, 2);

        double madX = mad(poses, medianX, 0);
        double madY = mad(poses, medianY, 1);
        double madHeading = mad(poses, medianHeading, 2);

        double outlierThresholdAngle = 20.0; // Adjust as needed
        double outlierThresholdPose = 1;

        // Filtering outliers and finding latest acquisition time
        List<Pose3D> filteredPoses = new ArrayList<>();
        for (Pose3D pose3D : poses) {
            Position pose = pose3D.getPosition();

            if (Math.abs(pose.x - medianX) <= outlierThresholdPose * madX &&
                    Math.abs(pose.y - medianY) <= outlierThresholdPose * madY &&
                    Math.abs(getHeading(pose3D) - medianHeading) <= outlierThresholdAngle * madHeading
                    && isValidPose(pose)) {

                // Update latest acquisition time if current pose is valid and more recent
                if (pose.acquisitionTime > latestAcquisitionTime){
                    latestAcquisitionTime = pose.acquisitionTime;
                }

                filteredPoses.add(pose3D);
            }
        }

        // Calculation
        for (Pose3D pose : filteredPoses) {
            sumX += pose.getPosition().x;
            sumY += pose.getPosition().y;
            sumHeading += getHeading(pose);
            count++;
        }

        double avgX = sumX / count;
        double avgY = sumY / count;
        double avgHeading = sumHeading / count;

        // Return with latest acquisition time
        return new Pose3D(new Position(DistanceUnit.METER, avgX, avgY, 0, latestAcquisitionTime),
                new YawPitchRollAngles(AngleUnit.DEGREES, avgHeading, 0, 0, latestAcquisitionTime));
    }

    // Helper function to check if a pose is valid
    private static boolean isValidPose(Position pose) {
        return (pose.x >= FIELD_POSE_X_MIN_METERS && pose.x <=FIELD_POSE_X_MAX_METERS
                && pose.y >= FIELD_POSE_Y_MIN_METERS && pose.y <= FIELD_POSE_Y_MAX_METERS);
    }

    // Helper functions for median and MAD calculation
    private static double median(List<Pose3D> poses, int type) {
        List<Double> values = new ArrayList<>();
        for (Pose3D pose : poses) {
            if (type == 0) {
                values.add(pose.getPosition().x);
            } else if (type == 1) {
                values.add(pose.getPosition().y);
            } else {
                values.add(getHeading(pose));
            }
        }
        Collections.sort(values);
        int size = values.size();
        if (size % 2 == 0) {
            return (values.get(size / 2 - 1) + values.get(size / 2)) / 2;
        } else {
            return values.get(size / 2);
        }
    }

    private static double mad(List<Pose3D> poses, double median, int type) {
        List<Double> deviations = new ArrayList<>();
        for (Pose3D pose : poses) {
            if (type == 0) {
                deviations.add(Math.abs(pose.getPosition().x - median));
            } else if (type == 1) {
                deviations.add(Math.abs(pose.getPosition().y - median));
            } else {
                deviations.add(Math.abs(getHeading(pose) - median));
            }
        }
        Collections.sort(deviations);
        int size = deviations.size();
        if (size % 2 == 0) {
            return (deviations.get(size / 2 - 1) + deviations.get(size / 2)) / 2;
        } else {
            return deviations.get(size / 2);
        }
    }

    private static double getHeading(Pose3D pose) {
        return pose.getOrientation().getYaw();
    }
}
