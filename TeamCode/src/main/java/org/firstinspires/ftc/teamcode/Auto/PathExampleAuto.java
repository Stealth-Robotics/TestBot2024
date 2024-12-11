package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Paths.OutAndBackPath;
import org.firstinspires.ftc.teamcode.common.StealthAutoMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;

@Disabled
@Autonomous(name = "Blue pathing example", group = "examples")
public class PathExampleAuto extends StealthAutoMode {


   private OutAndBackPath path;

    /**
     * Override this to setup your hardware, commands, button bindings, etc.
     */
    @Override
    public void initialize() {

        super.initialize();
        telemetryA.addLine("Example path of using the follower");
        path = new OutAndBackPath();
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
        Follower follower = followerSubsystem.getFollower();

        return new SequentialCommandGroup(
                new InstantCommand(() ->follower.followPath(path.getNextSegment())),
                new WaitCommand(1000),
                setBothArms(.99),
                new WaitCommand(500),
                new InstantCommand(() ->follower.followPath(path.getNextSegment())),
                new WaitCommand(1000),
                setBothArms(.01)
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
