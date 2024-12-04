package org.firstinspires.ftc.teamcode.common;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.utils.MovementUtil;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public abstract class AutoBase extends StealthOpMode {

    protected Telemetry telemetryA;
    protected FollowerSubsystem followerSubsystem;
    protected LimeLightSubsystem limelightSubsystem;

    protected LifterSubsystem lss;
    protected ExtenderSubsystem ess;

    protected Pose lastPose;

    public Pose avgPose;

    private static final long AVERAGE_WAIT_TIME = 100;

    private long timer = 0;

    protected void Init() {
        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        followerSubsystem = new FollowerSubsystem(hardwareMap, telemetry);
        limelightSubsystem = new LimeLightSubsystem(hardwareMap, telemetry);
        lss = new LifterSubsystem(hardwareMap, telemetry);
        ess = new ExtenderSubsystem(hardwareMap, telemetry);
        register(followerSubsystem, limelightSubsystem, lss, ess);
    }

    @Override
    public void whileWaitingToStart() {
        LLResult result = limelightSubsystem.getLastResult();
        if (result != null && result.isValid()) {
            Pose3D pose3d = result.getBotpose();
            Pose fixedPose = MovementUtil.getFollowPoseFromLimelight(pose3d);
            this.lastPose = fixedPose;
            telemetryA.addData("CAM X", fixedPose.getX());
            telemetryA.addData("CAM Y", fixedPose.getY());
            telemetryA.addData("CAM Heading", Math.toDegrees(fixedPose.getHeading()));
            if (MathUtils.clamp(fixedPose.getX(), 0, 144) < 72) {
                Alliance.set(Alliance.BLUE);
            }
        }

        // Get average of position over time
        if (System.currentTimeMillis() > timer + AVERAGE_WAIT_TIME) {
            timer = System.currentTimeMillis();
            Pose3D avgPose3d = limelightSubsystem.getAveragePose3D();
            if (avgPose3d != null) {
                this.avgPose = MovementUtil.getFollowPoseFromLimelight(avgPose3d);
                telemetryA.addData("CAM AVG X", avgPose.getX());
                telemetryA.addData("CAM AVG Y", avgPose.getY());
                telemetryA.addData("CAM Heading AVG", Math.toDegrees(avgPose.getHeading()));

            }

        }
    }

    public LifterSubsystem getLifter() {
        return lss;
    }

    public ExtenderSubsystem getExtender() {
        return ess;
    }

    public FollowerSubsystem getFollower() {
        return followerSubsystem;
    }

    public LimeLightSubsystem getLimelight() {
        return limelightSubsystem;
    }
}
