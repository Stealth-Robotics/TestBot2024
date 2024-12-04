package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Paths.PushBlocksPath;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.utils.MovementUtil;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Config
@Autonomous(name = "Auto Push Blue", group = "examples")
public class AutoPushBlue extends StealthOpMode {


    private Telemetry telemetryA;
    private FollowerSubsystem followerSubsystem;

    private LimeLightSubsystem limelightSubsystem;

    PushBlocksPath path;

    /**
     * Override this to setup your hardware, commands, button bindings, etc.
     */
    @Override
    public void initialize() {
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("Example of pushing blue samples");
        this.followerSubsystem = new FollowerSubsystem(hardwareMap, telemetry);
        this.limelightSubsystem = new LimeLightSubsystem(hardwareMap, telemetry);
        LifterSubsystem lss = new LifterSubsystem(hardwareMap, telemetry);
        register(lss);
        this.path = new PushBlocksPath(lss);
        register(this.followerSubsystem, this.limelightSubsystem);
    }

    @Override
    public Command getAutoCommand() {
        return new InstantCommand(()->
        {
            Follower follower = this.followerSubsystem.getFollower();
            follower.setStartingPose(PushBlocksPath.startPos);
            follower.update();
            follower.followPath(path.pathChain, true);
            follower.update();
        });
    }

    @Override
    public void whileWaitingToStart() {
        LLResult result = limelightSubsystem.getLastResult();
        if (result != null && result.isValid()) {
            Pose3D pose3d = result.getBotpose();
            Pose updatedPose = MovementUtil.getFollowPoseFromLimelight(pose3d);
            path.updatePose(MovementUtil.getFollowPoseFromLimelight(pose3d));
            telemetryA.addData("Field Fixed X", updatedPose.getX());
            telemetryA.addData("Field Fixed Y", updatedPose.getY());
            telemetryA.addData("Heading Fixed", Math.toDegrees(updatedPose.getHeading()));
            telemetryA.update();
        }
    }
}
