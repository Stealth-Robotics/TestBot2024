package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Paths.HangSamplesPath;
import org.firstinspires.ftc.teamcode.common.StealthAutoMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;


@Autonomous(name = "Blue pathing example", group = "examples")
public class PathExampleAuto extends StealthAutoMode {


   private HangSamplesPath path;

    /**
     * Override this to setup your hardware, commands, button bindings, etc.
     */
    @Override
    public void initialize() {

        super.initialize();
        telemetryA.addLine("Example path of using the follower");
        path = new HangSamplesPath();
        commandGroup.addCommands(initPath(), runPath());
    }

    @Override
    public Command getAutoCommand() {
        return commandGroup;
    }

    protected Command initPath() {
        return new InstantCommand(()->
        {
            Follower follower = followerSubsystem.getFollower();
            if (avgPose != null) {
                path.setStartPose(avgPose);
            } else if (lastPose != null) {
                path.setStartPose(lastPose);
            }
            follower.setStartingPose(path.getStartPose());
            follower.update();
        });

    }

    private Command runPath() {
        double pickupH = .2;
        double hangH = .5;
        return new SequentialCommandGroup(
                lss.startSetPositionCommand(hangH),
                followerSubsystem.followPathCommand(path.getNextSegment(),true),
                lss.startSetPositionCommand(pickupH),
                //setBothArms(.99),
                followerSubsystem.followPathCommand(path.getNextSegment()),
                new WaitCommand(100),
                lss.startSetPositionCommand(hangH),
                followerSubsystem.followPathCommand(path.getNextSegment(),true),
                lss.startSetPositionCommand(.001),
                followerSubsystem.followPathCommand(path.getNextSegment()),
                lss.startSetPositionCommand(pickupH),
                followerSubsystem.followPathCommand(path.getNextSegment()),
                lss.startSetPositionCommand(hangH),
                followerSubsystem.followPathCommand(path.getNextSegment(), true),
                lss.startSetPositionCommand(pickupH),
                followerSubsystem.followPathCommand(path.getNextSegment()),
                lss.startSetPositionCommand(hangH),
                followerSubsystem.followPathCommand(path.getNextSegment(), true),
                lss.startSetPositionCommand(.001),
                followerSubsystem.followPathCommand(path.getNextSegment())
        );
    }

    private InstantCommand setBothArms(double pose) {
        return new InstantCommand(() ->
        {
            getExtender().setPosition(pose);
            getLifter().setPosition(pose);
        });
    }

}
