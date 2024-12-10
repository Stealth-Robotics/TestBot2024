package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Paths.PathSquare;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

@Autonomous(name = "Auto Square", group = "examples")
public class AutoSquare extends OpMode {

    private Follower follower;

    private PathChain pathChain;

    private Telemetry telemetryA;

    private boolean end;

    @Override
    public void init() {
        this.follower = new Follower(hardwareMap);
        pathChain =PathSquare.getPathChain();
        follower.followPath(pathChain);
        follower.setPose(PathSquare.StartPose);
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("this follows a square path of 4x4");
        telemetryA.update();
    }

    @Override
    public void loop() {
        this.follower.update();
        if (this.follower.atParametricEnd()) {
            this.follower.followPath(pathChain);
        }

    }
}
