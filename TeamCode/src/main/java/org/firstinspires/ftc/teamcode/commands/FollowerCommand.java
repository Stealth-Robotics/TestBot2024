package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

import java.util.function.DoubleSupplier;

/**
 * This sends commands to the follower to move the robot.
 */
public class FollowerCommand extends CommandBase {

    // Drive Subsystem
    private final FollowerSubsystem subsystem;
    private final Telemetry telemetryA;

    // Stick input fof X
    private final DoubleSupplier x;
    // Stick input for Y
    private final DoubleSupplier y;

    // Rotational input for Z
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
        // this must be added to the command group or the subsystem will fail to initialize
        addRequirements(subsystem);
    }

    /**
     * called the first time you press the play button on the hub
     */
    @Override
    public void initialize(){
        toggleRobotCentric = false;
        slowMode = false;
    }

    /**
     * executes is called in rapidly by the scheduler.
     */
    @Override
    public void execute(){
        Follower follower = subsystem.getFollower();
        follower.setTeleOpMovementVectors(x.getAsDouble(), y.getAsDouble(), zx.getAsDouble(), toggleRobotCentric);
        follower.update();
    }

    /**
     * Swaps between fei;d centric and robot centric.
     */
    public void toggleRobotCentric(){
        toggleRobotCentric = !toggleRobotCentric;
        telemetryA.addData("toggleRobotCentric", toggleRobotCentric);
    }

    /**
     * Swaps between slow and fast mode.
     */
    public void toggleSlowMode(){
        slowMode = !slowMode;
        Follower follower = subsystem.getFollower();
        if(slowMode){
            follower.setMaxPower(slowPower);
        }
        else {
            follower.setMaxPower(maxPower);
        }
    }

    /**
     * This will reset the IMU to 0x, 0y, 0 heading
     * When driving field centric and need to adjust the robot orientation
     */
    public void resetImu()
    {
        subsystem.getFollower().resetIMU();
    }
}
