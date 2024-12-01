package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Paths.PushBlocksPath;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
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
        this.path = new PushBlocksPath();
        register(this.followerSubsystem, this.limelightSubsystem);

    }

    @Override
    public Command getAutoCommand() {
        return new InstantCommand(()->
        {
            Follower follower = this.followerSubsystem.getFollower();
            follower.setStartingPose(PushBlocksPath.StartPos);
            follower.update();
            follower.followPath(path.pathChain, true);
            follower.update();
        });
    }

    @Override
    public void whileWaitingToStart() {
        // TODO: Add code that checks the limelight for pose and
        // update the follower accordingly.
        //CommandScheduler.getInstance().run();
    }
}
