package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ExtenderSubsystem;

import java.util.function.DoubleSupplier;

public class ExtenderDefaultCommand extends CommandBase {

    private final ExtenderSubsystem extender;
    private final MultipleTelemetry telemetryA;
    private final DoubleSupplier leftTrigger;
    private final DoubleSupplier rightTrigger;
    private boolean manualControl = false;
    private static final double axisDeadZone = 0.05;


    public ExtenderDefaultCommand(ExtenderSubsystem extender, Telemetry telemetry, DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
        addRequirements(extender);
        this.extender = extender;
        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.leftTrigger = leftTrigger;
        this.rightTrigger = rightTrigger;
    }

    @Override
    public void execute() {
        double leftPower = leftTrigger.getAsDouble();
        double rightPower = rightTrigger.getAsDouble();
        double power = rightPower - leftPower;

        if(power > axisDeadZone || power < -axisDeadZone) {
            extender.stopRunTo();
            if (!extender.getMotor1().getMode().equals(DcMotor.RunMode.RUN_WITHOUT_ENCODER)) {
                extender.getMotor1().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            manualControl = true;
            extender.setPower(power); //Manual control
            telemetryA.addData("Manual Extend", power);
        }else if (manualControl) {

            extender.setPower(0);
            manualControl = false;
        }
    }

}
