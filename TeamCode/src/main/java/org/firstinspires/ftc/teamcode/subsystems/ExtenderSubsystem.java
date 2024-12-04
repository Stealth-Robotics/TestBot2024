package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Example usage of a single DC motor with PIDF control.
 */
@Config
public class ExtenderSubsystem extends SubsystemBase {
    private static final String MOTOR_NAME_1 = "armextender";
    private final Telemetry telemetryA;

    // Adjust these values for your arm. These will need to change
    // based on arm weight and total range of the arm
    public static double kP = 0.006;
    public static double kI = 0.00;
    public static double kD = 0.0;
    public static double kF = 0.00;

    // This should be the maximum encoder extension of the arm(s)
    private static final double MAX_HEIGHT = 417;

    // Acceptable position error to be considered at target location
    public static final double TOLERANCE = 10.0;
    private Boolean motorRunTo = false;
    private final double maxSpeed = 1;
    private final DcMotorEx extendMotor;

    // PIDF to control arm movement keeps the arm from overshooting etc.
    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    public ExtenderSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        extendMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        pidf.setTolerance(TOLERANCE);
        // Use MotorGroup for multiple motors
        resetEncoder();
        extendMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        extendMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Called automatically when the subsystem is registered.
     */
    @Override
    public void periodic() {
        if (motorRunTo) {
            double power = pidf.calculate(extendMotor.getCurrentPosition());
            extendMotor.setPower(power * maxSpeed);

            if (pidf.atSetPoint()) {
                extendMotor.setPower(-.1);
                motorRunTo = false;
            }
        }

        telemetryA.addData("Extend Target:", extendMotor.getTargetPosition());
        telemetryA.addData("Extend Mode:", extendMotor.getMode());
        telemetryA.addData("Extend Position:", extendMotor.getCurrentPosition());
        telemetryA.addData("Extender Power:", extendMotor.getPower());
        telemetryA.addData("IsBusy:", extendMotor.isBusy());
    }

    /**
     * Sets the power of the extend motor
     * @param power power to set
     */
    public void setPower(double power) {
        extendMotor.setPower(power);
    }

    /**
     * Sets the position of the extend motor in % of max range
     * 1 = full up .5 = half up. should be used with limit switches
     * @param position position in % of max range
     */
    public void setPosition(double position) {
        pidf.setSetPoint(position * MAX_HEIGHT);
        motorRunTo = true;
    }

    /**
     * Blocking function to set the position of the extend motor in % of max range
     * Should only be used if this object isn't registered (periodic isn't running)
     * @param position 0 to 1 in % of max range
     */
    public void doPosition(double position) {
        pidf.setSetPoint(position * MAX_HEIGHT);
        motorRunTo = true;
        while (motorRunTo) {
            double power = pidf.calculate(extendMotor.getCurrentPosition());
            extendMotor.setPower(power * maxSpeed);

            if (pidf.atSetPoint()) {
                extendMotor.setPower(-.1);
                motorRunTo = false;
            }
        }
    }

    /**
     * Stops the run to position
     */
    public void stopRunTo() {
        motorRunTo = false;
    }

    /**
     * Returns the current position of the extend motor
     * @return current position
     */
    public int getPosition() {
        return extendMotor.getCurrentPosition();
    }

    /**
     * Resets the encoder of the extend motor
     */
    public void resetEncoder() {
        extendMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Returns the extend motor
     * @return extend motor
     */
    public DcMotorEx getMotor1() {
        return extendMotor;
    }


}
