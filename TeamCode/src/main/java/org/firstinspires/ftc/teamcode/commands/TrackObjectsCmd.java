package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.utils.MovementUtil.POWER_TRANSLATION_KP;
import static org.firstinspires.ftc.teamcode.utils.MovementUtil.calculateMotorPower;
import static org.firstinspires.ftc.teamcode.utils.MovementUtil.getAlignmentPoseToObject;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;

/*
* This example is for tracking objects using the Limelight.
* Still a work in progress
*/
public class TrackObjectsCmd extends CommandBase {
    private final FollowerSubsystem followerSubsystem;
    private final LimeLightSubsystem limelightSubsystem;
    private final Telemetry telemetryA;
    private boolean doExecute = true;
    private boolean strafeTrack = false;

    private static final double MIN_TRANSLATION_POWER = 0.07;
    private static final double MAX_TRANSLATION_POWER = 0.8;


    public TrackObjectsCmd(FollowerSubsystem followerSubsystem, LimeLightSubsystem limelightSubsystem, Telemetry telemetryA) {
        this.followerSubsystem = followerSubsystem;
        this.limelightSubsystem = limelightSubsystem;
        this.telemetryA = telemetryA;
    }

    public void toggleStrafeTrack() {
        strafeTrack = !strafeTrack;
    }

    public void toggleDoExecute() {
        doExecute = !doExecute;
    }

    public void setLimelightPipeline(Pipeline pipeline) {
        limelightSubsystem.setPipeline(pipeline);
    }

    public void StrafeTrack(Pose curPos, double cameraX) {

        Pose newPose = getAlignmentPoseToObject(curPos, cameraX);
        if (newPose != curPos) {
            Follower follower = followerSubsystem.getFollower();
            double scaledPower = calculateMotorPower(cameraX, MIN_TRANSLATION_POWER, POWER_TRANSLATION_KP);
            follower.setMaxPower(scaledPower);
            follower.holdPoint(newPose);
            follower.update();
        }
    }

    @Override
    public void execute() {
        if (!doExecute) {
            return;
        }

        Follower follower = followerSubsystem.getFollower();
        follower.updatePose();
        Pose curPos = follower.getPose();
        LLResult result = limelightSubsystem.getLastResult();
        if (result != null && result.isValid() && result.getStaleness() < 50) {
            if (strafeTrack) {
                StrafeTrack(curPos, result.getTx());
            } else {
               followerSubsystem.updateFollowerHeading(-result.getTx(), .5);
            }

            //telemetryA.addData("Cam X", result.getTx());
            //telemetryA.addData("Cam Y", result.getTy());
        } else {
            follower.breakFollowing();
            //follower.setMaxPower(1);
        }

        //telemetryA.addData("Pipeline", limelight.getStatus().getPipelineIndex());
    }
}
