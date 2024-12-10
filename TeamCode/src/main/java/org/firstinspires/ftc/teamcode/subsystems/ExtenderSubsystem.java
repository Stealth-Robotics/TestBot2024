package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.stealthrobotics.library.StealthSubsystem;

/**
 * Example usage of a single DC motor with PIDF control.
 */
@Config
public class ExtenderSubsystem extends StealthSubsystem {
    private static final String MOTOR_NAME_1 = "armextender";
    private final Telemetry telemetryA;

    // Adjust these values for your arm. These will need to change
    // based on arm weight and total range of the arm
    public static double kP = 0.007;
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
                extendMotor.setPower(0);
                motorRunTo = false;
            }
        }

//        telemetryA.addData("Extend Target:", extendMotor.getTargetPosition());
//        telemetryA.addData("Extend Mode:", extendMotor.getMode());
        telemetryA.addData("Extend Position:", extendMotor.getCurrentPosition());
//        telemetryA.addData("Extender Power:", extendMotor.getPower());
        telemetryA.addData("Extender IsBusy:", extendMotor.isBusy());
        telemetryA.addData("Extender RunTo:", motorRunTo);
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
     * Starts the motor moving but does not wait for it to reach the desired position
     * @param position % of the arm range
     * @return
     */
    public Command startSetPositionCommand(double position) {
        return this.runOnce(()-> setPosition(position));
    }

    /**
     * After calling setPosition this can be called to wait for the position to be reached
     * @param timeout waiting time before giving up on the desired position
     * @return command to wait for the position to be reached
     */
    public Command endSetPositionCommand(long timeout) {
        long endTime = System.currentTimeMillis() + timeout;
        return new WaitUntilCommand(()-> !motorRunTo || System.currentTimeMillis() >= endTime);
    }

    /**
     * Sets the position of the extend motor in encoder ticks
     * @param position position in encoder ticks
     */
    public Command setPositionCommand(double position) {
        return this.runOnce(()-> setPosition(position))
                .andThen(new WaitUntilCommand(()-> !motorRunTo));
    }

    /**
     * Sets the position of the extend motor in encoder ticks with a timeout
     * @param position % of the arm range
     * @param timeout number of ms to wait for the position to be reached
     * @return command to wait for the position to be reached
     */
    public Command setPositionCommand(double position, long timeout) {
        long endTime = System.currentTimeMillis() + timeout;
        return this.runOnce(()-> setPosition(position))
                .andThen(new WaitUntilCommand(()-> !motorRunTo || System.currentTimeMillis() >= endTime));
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
