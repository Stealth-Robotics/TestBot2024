package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.stealthrobotics.library.StealthSubsystem;

/**
 * This is a wrapper around the Pedro follower object and provides
 * Movement control of the robot
 */
public class FollowerSubsystem extends StealthSubsystem {
    private final Follower follower;
    private final Telemetry telemetryA;

    /**
     * Used for rotational movement when providing a heading to the robot.
     */
    private final PIDFController headingTrackPidf
            = new PIDFController(0.02, 0.0, 0.0, 0.0);

    // Tolerance for heading track
    private static final double TOLERANCE = 1.0; // Degrees from center

    // Minimum power needed to rotate the robot
    private static final double MIN_ROTATION_POWER = 0.07;

    // Maximum power to use to rotate the robot in place.
    private static final double MAX_ROTATION_POWER = 0.8;

    public FollowerSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        follower = new Follower(hardwareMap);
        follower.initialize();
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareMap.get(DcMotorEx.class, leftFrontMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotorEx.class, leftRearMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotorEx.class, rightRearMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardwareMap.get(DcMotorEx.class, rightFrontMotorName).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        headingTrackPidf.setTolerance(TOLERANCE);
        follower.startTeleopDrive();

    }

    /**
     * Gives direct access to the follower object
     * @return follower object
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Prints follower information to the dashboard. Disable spew for any
     * real automation or driving
     */
    @Override
    public void periodic() {
        this.getFollower().update();
        Pose curPos = this.getFollower().getPose();
        telemetryA.addData("IMU X", curPos.getX());
        telemetryA.addData("IMU Y", curPos.getY());
        telemetryA.addData("IMU Heading:", Math.toDegrees(curPos.getHeading()));
    }


    /**
     * Rotates the robot to face a certain heading delta from current heading
     * @param headingDelta heading change required + or - values
     * @param tolerance tolerance in degrees from desired heading
     */
    public void updateFollowerHeading(double headingDelta, double tolerance) {
        follower.updatePose();
        Pose curPos = follower.getPose();
        double curHeading = Math.toDegrees(curPos.getHeading());
        double adjustedHeading = normalizeHeading(curHeading + headingDelta);

        // Calculate the absolute difference, considering the circular nature of headings
        double difference = Math.abs(adjustedHeading - curHeading);
        if (difference > 180) {
            difference = 360 - difference;
        }

        // Check if the difference is within the tolerance
        if (difference > tolerance) {

            if (follower.isBusy())
            {
                telemetryA.addLine("Follower is busy");
                follower.breakFollowing();
            }

            Pose newPose = new Pose(curPos.getX(), curPos.getY(), Math.toRadians(adjustedHeading));
            PathBuilder builder = new PathBuilder();
            PathChain path = builder.addPath(
                            new BezierLine(
                                    new Point(curPos.getX(), curPos.getY(), Point.CARTESIAN),
                                    new Point(curPos.getX(), curPos.getY(), Point.CARTESIAN)))
                    .setConstantHeadingInterpolation(newPose.getHeading())
                    .build();

            follower.followPath(path);
            follower.update();
        }
    }

    /**
     * Initiates a heading track command to face a certain heading delta from current heading.
     * Note - need to be called until true is returned or the robot will continue to rotate.
     * @param headingDelta heading change required + or - values
     */
    public Boolean doHeadingTrack (double headingDelta) {

        headingTrackPidf.setSetPoint(0);
        double output = headingTrackPidf.calculate(headingDelta);
        telemetryA.addData("PIDF Power", output);

        // Check if the difference is within the tolerance
        if (!headingTrackPidf.atSetPoint()) {
            double scaledOutput = Math.max(Math.abs(output), MIN_ROTATION_POWER) * Math.signum(output);
            scaledOutput = MathFunctions.clamp(scaledOutput, -MAX_ROTATION_POWER, MAX_ROTATION_POWER);
            telemetryA.addData("Scaled Power", scaledOutput);

            follower.setTeleOpMovementVectors(0, 0, scaledOutput);
            follower.update();
            return false;
        }

        telemetryA.addLine("At Set Point");
        follower.setTeleOpMovementVectors(0, 0, 0);
        follower.update();

        return true;
    }

    public Command followPathCommand(Path path){
        return followPathCommand(path, false);
    }
    
    public Command followPathCommand(PathChain path){
        return followPathCommand(path, false);
    }
    
    public Command followPathCommand(Path path, boolean holdPoint){
        return this.runOnce(()-> follower.followPath(path,holdPoint))
                .andThen(new WaitUntilCommand(()-> !follower.isBusy()));
    }
    public Command followPathCommand(PathChain path, boolean holdPoint){
        return this.runOnce(()-> follower.followPath(path,holdPoint))
                .andThen(new WaitUntilCommand(()-> !follower.isBusy()));
    }
    public void setPose(Pose pose){
        follower.setPose(pose);
    }

    private static double normalizeHeading(double heading) {
        heading = (heading) % 360;
        return heading < 0 ? heading + 360 : heading;
    }
}
