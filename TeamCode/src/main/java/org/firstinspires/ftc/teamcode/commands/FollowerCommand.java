package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

import java.util.function.DoubleSupplier;

public class FollowerCommand extends CommandBase {

    private final FollowerSubsystem subsystem;
    private final Telemetry telemetryA;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier zx;


    // setting to false means field centric
    private boolean toggleRobotCentric = false;

    private static final double maxPower = 1.0;
    private static final double slowPower = .5;
    private boolean slowMode = false;

    public FollowerCommand(FollowerSubsystem subsystem, Telemetry telemetry, DoubleSupplier x, DoubleSupplier y, DoubleSupplier zx){

        this.subsystem = subsystem;
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.x = x;
        this.y = y;
        this.zx = zx;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){
        toggleRobotCentric = false;
        slowMode = false;
    }

    @Override
    public void execute(){
        Follower follower = subsystem.getFollower();
        follower.setTeleOpMovementVectors(x.getAsDouble(), y.getAsDouble(), zx.getAsDouble(), toggleRobotCentric);
        follower.update();
        telemetryA.addData("x", x.getAsDouble());
        telemetryA.addData("y", y.getAsDouble());
        telemetryA.addData("zx", zx.getAsDouble());
    }
    
    public void toggleRobotCentric(){
        toggleRobotCentric = !toggleRobotCentric;
        telemetryA.addData("toggleRobotCentric", toggleRobotCentric);
    }
    
    public void toggleSlowMode(){
        slowMode = !slowMode;
        Follower follower = subsystem.getFollower();
        if(slowMode){
            follower.setMaxPower(slowPower);
        }
        else {
            follower.setMaxPower(maxPower);
        }

        telemetryA.addData("toggleSlowMode", slowMode);
    }

    public void resetImu()
    {
        subsystem.getFollower().resetIMU();
    }
}
