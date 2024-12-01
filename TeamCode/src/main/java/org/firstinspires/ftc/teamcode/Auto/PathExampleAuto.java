package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Paths.OutAndBackPath;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.firstinspires.ftc.teamcode.utils.MovementUtil;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "Out and Back", group = "examples")
public class PathExampleAuto extends StealthOpMode {

    private Telemetry telemetryA;
    private FollowerSubsystem followerSubsystem;

    private LimeLightSubsystem limelightSubsystem;

    private OutAndBackPath outAndBackPath;


    /**
     * Override this to setup your hardware, commands, button bindings, etc.
     */
    @Override
    public void initialize() {
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.followerSubsystem = new FollowerSubsystem(hardwareMap, telemetry);
        this.limelightSubsystem = new LimeLightSubsystem(hardwareMap, telemetry);
        LifterSubsystem lss = new LifterSubsystem(hardwareMap, telemetry);
        this.outAndBackPath = new OutAndBackPath(lss);
        register(this.followerSubsystem, this.limelightSubsystem);
        telemetryA.addLine("this follows a square path of 4x4");
    }

    @Override
    public Command getAutoCommand() {
        return new InstantCommand(()->
        {
            Follower follower = this.followerSubsystem.getFollower();
            follower.setStartingPose(outAndBackPath.StartPose);
            follower.update();
            follower.followPath(outAndBackPath.pathChain);
            follower.update();
        });
    }

    @Override
    public void whileWaitingToStart() {
        LLResult result = limelightSubsystem.getLastResult();
        if (result != null && result.isValid()){
            Pose3D pose3d = result.getBotpose();
            outAndBackPath.StartPose = MovementUtil.getFollowPoseFromLimelight(pose3d);
            telemetryA.addData("CAM X", outAndBackPath.StartPose.getX());
            telemetryA.addData("CAM Y", outAndBackPath.StartPose.getY());
            telemetryA.addData("Heading", Math.toDegrees(outAndBackPath.StartPose.getHeading()));
            telemetryA.update();
        }


        //CommandScheduler.getInstance().run();
    }
}
