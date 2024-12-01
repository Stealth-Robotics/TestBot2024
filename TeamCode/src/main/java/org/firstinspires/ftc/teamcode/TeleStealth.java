package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ExtenderDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.FollowerCommand;
import org.firstinspires.ftc.teamcode.commands.LifterDefaultCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.subsystems.ExtenderSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimeLightSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

// Example of using the Stealth library for TeleOp driving
// this example also uses pedroPathing to drive the robot
@TeleOp(name = "A Stealth TeleOp", group = "example")

public class TeleStealth extends StealthOpMode {

    // This is the starting position of the robot in inches. Blue home facing red bucket
    // is 0, 0, 0 + 1/2 the robot width and Length
    private static final Pose startPose = new Pose(8.5, 48, 0);

    private GamepadEx driver;
    //private GamepadEx operator;

    private FollowerSubsystem fss;
    private LimeLightSubsystem ls;


    @Override
    public void whileWaitingToStart(){
        // Enabling this would mean the robot is live even before play is pushed
        //CommandScheduler.getInstance().run();
        // block to
    }

    @Override
    public void initialize(){
        driver = new GamepadEx(gamepad1);
        //operator = new GamepadEx(gamepad2);

        // You need to created two classes for each mechanism.
        // The Subsystem sets up the hardware and the command runs the code on the hardware
        fss = new FollowerSubsystem(hardwareMap, telemetry);
        ls = new LimeLightSubsystem(hardwareMap, telemetry);
        LifterSubsystem lifter = new LifterSubsystem(hardwareMap, telemetry);
        ExtenderSubsystem extender = new ExtenderSubsystem(hardwareMap, telemetry);
        register(fss, ls, lifter, extender);

        // this is setting for telemetry to be sent to the driver station
        fss.getFollower().setStartingPose(startPose);
        // registers for the periodic to be called (telemetry)
        // Follower controls robot motion
        FollowerCommand cmd = new FollowerCommand(
                fss,
                telemetry,
                () -> driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX());
        fss.setDefaultCommand(cmd);
        driver.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(new InstantCommand(cmd::toggleSlowMode));
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new InstantCommand(cmd::toggleRobotCentric));
        driver.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new InstantCommand(cmd::resetImu));

        LifterDefaultCommand liftCmd = new LifterDefaultCommand(
                lifter,
                telemetry,
                ()-> driver.getRightY());

        lifter.setDefaultCommand(liftCmd);
        ExtenderDefaultCommand extenderCmd = new ExtenderDefaultCommand(
                extender,
                telemetry,
                ()-> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER),
                ()-> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

        extender.setDefaultCommand(extenderCmd);

        // register for the periodic to be called

        driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(()->lifter.setPosition(.5)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(()->lifter.setPosition(.99)));
        driver.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(()->lifter.setPosition(.001)));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new InstantCommand(()->ls.togglePipeline()));


    }

}
