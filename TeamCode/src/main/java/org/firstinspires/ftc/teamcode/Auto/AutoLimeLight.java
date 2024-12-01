package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.TrackObjectsCmd;
import org.firstinspires.ftc.teamcode.common.Pipeline;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

/**
 * This show how the robot can track a target using the Limelight.
 */
@Autonomous(name = "Auto Limelight", group = "examples")
public class AutoLimeLight extends StealthOpMode {


    // Example starting pose for the robot. Pedro follower uses
    //
    private final Pose startingPose  = new Pose(8, 48, 0);

    /**
     * Override this to setup your hardware, commands, button bindings, etc.
     */
    @Override
    public void initialize() {
        // Allows for multiple writers to the telemetry window
        Telemetry telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // main driving and positioning subsystem
        LimeLightSubsystem llss = new LimeLightSubsystem(hardwareMap, telemetry);
        FollowerSubsystem fss = new FollowerSubsystem(hardwareMap, telemetry);

        // registering means the periodic method will be called on each subsystem
        register(llss, fss);
        TrackObjectsCmd trackObjects = new TrackObjectsCmd(fss, llss, telemetry);
        fss.getFollower().setStartingPose(startingPose);

        GamepadEx driver = new GamepadEx(gamepad1);
        telemetryA.addLine("Use A, B, X, Y to change the limelight pipeline. Left stick button to stop.");
        telemetryA.addLine("Left bumper to toggle strafe track. Note that requires proper robot field alignment.");
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand(trackObjects::toggleDoExecute));
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(()-> trackObjects.setLimelightPipeline(Pipeline.APRIL_TAG)));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(()-> trackObjects.setLimelightPipeline(Pipeline.RED_SPEC)));
        driver.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand(()-> trackObjects.setLimelightPipeline(Pipeline.BLUE_SPEC)));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(()-> trackObjects.setLimelightPipeline(Pipeline.YELLOW_SPEC)));
        driver.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(trackObjects::toggleStrafeTrack));
        telemetryA.update();
        this.schedule(trackObjects);
    }
    
    @Override
    public void whileWaitingToStart() {

        //CommandScheduler.getInstance().run();
    }
}
