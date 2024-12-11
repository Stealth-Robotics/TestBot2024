package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.LifterSubsystem;

import java.util.function.DoubleSupplier;

public class LifterDefaultCommand extends CommandBase {

    private final LifterSubsystem lifter;
    private final MultipleTelemetry telemetryA;
    private final DoubleSupplier axis;
    private boolean manualControl = false;
    private static final double axisDeadZone = 0.05;

    public LifterDefaultCommand(LifterSubsystem lifter, Telemetry telemetry, DoubleSupplier axis) {
        addRequirements(lifter);
        this.lifter = lifter;
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.axis = axis;
    }

    @Override
    public void execute() {
        double power = axis.getAsDouble();
        if(power > axisDeadZone || power < -axisDeadZone) {
            lifter.stopRunTo();
            if (!lifter.getMotor1().getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                lifter.getMotor1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            manualControl = true;
            lifter.setPower(power);

        }else if (manualControl) {
            // holds the lift here
            lifter.setPower(-0.1);
            manualControl = false;
        }
    }

}
