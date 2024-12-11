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
public class LifterSubsystem extends StealthSubsystem {
    private final DcMotorEx liftMotor;
    private static final String MOTOR_NAME_1 = "elevator";
    private final Telemetry telemetryA;

    public static double kP = 0.006;
    public static double kI = 0.00;
    public static double kD = 0.0;
    public static double kF = 0.00;

    public Boolean motorRunTo = false;

    public static final double TOLERANCE = 10.0;
    private final double maxSpeed = 1;

    private static final double MAX_HEIGHT = 4367;

    private final PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    public LifterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        liftMotor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        pidf.setTolerance(TOLERANCE);
        // Use MotorGroup for multiple motors
        resetEncoder();
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Called by the scheduler if registered and running
     */
    @Override
    public void periodic() {
        if (motorRunTo) {
            double power = pidf.calculate(liftMotor.getCurrentPosition());
            liftMotor.setPower(-power * maxSpeed);

            if (pidf.atSetPoint()) {
                liftMotor.setPower(-.1);
                motorRunTo = false;
            }
        }

        telemetryA.addData("Lift Target:", liftMotor.getTargetPosition());
        telemetryA.addData("Lift Position:", liftMotor.getCurrentPosition());
        telemetryA.addData("motorRunTo:", motorRunTo);
    }

    /**
     * Starts the motor moving but does not wait for it to reach the desired position
     * @param position % of the arm range
     * @return A Command that can be run
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
        return new WaitUntilCommand(()-> (!motorRunTo || System.currentTimeMillis() >= endTime));
    }

    /**
     * Sets the power of the lift motor
     * @param power power to set
     */
    public void setPower(double power) {
        liftMotor.setPower(power);
    }

    /**
     * Sets the position of the lift motor in % of max range
     * 1 = full up .5 = half up. should be used with limit switches
     * @param position position in % of max range
     */
    public void setPosition(double position) {
        pidf.setSetPoint(position * MAX_HEIGHT);
        motorRunTo = true;
    }

    /**
     * Blocking function to set the position of the lift motor in % of max range
     * @param position 0 to 1 in % of max range
     */
    public void doPosition(double position) {
        pidf.setSetPoint(position * MAX_HEIGHT);
        motorRunTo = true;
        while (motorRunTo) {
            double power = pidf.calculate(liftMotor.getCurrentPosition());
            liftMotor.setPower(-power * maxSpeed);

            if (pidf.atSetPoint()) {
                liftMotor.setPower(-.1);
                motorRunTo = false;
            }
        }
    }

    /**
     * Stops the motor(s) from autonomously moving
     */
    public void stopRunTo()
    {
        motorRunTo = false;
    }

    /**
     * Returns the current position of the lift motor
     * @return current position
     */
    public int getPosition() {
        return liftMotor.getCurrentPosition();
    }

    /**
     * Resets the encoder of the lift motor to 0
     */
    public void resetEncoder() {
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Command to set the position of the lift motor in encoder ticks
     * @param position Position in % of range of the lifter
     * @return A Command that can be run
     */
    public Command setPositionCommand(double position) {
        return this.runOnce(()-> setPosition(position))
                .andThen(new WaitUntilCommand(()-> !motorRunTo));
    }

    /**
     * Sets the Lifter position with timeout
     * @param position % of max extension
     * @param timeout Max number of ms to wait for the position to be reached
     * @return Command to wait for the position to be reached
     */
    public Command setPositionCommand(double position, long timeout) {
        long endTime = System.currentTimeMillis() + timeout;
        return this.runOnce(()-> setPosition(position))
                .andThen(new WaitUntilCommand(()-> (!motorRunTo || System.currentTimeMillis() > endTime)));
    }

    /**
     * Returns the lift motor
     * @return raw DcMotorEx object
     */
    public DcMotorEx getMotor1(){
        return liftMotor;
    }
}
