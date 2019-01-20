package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class DanielBot_Hardware
{
    // Constant that converts the pivot arm position to degrees (1120*10/360)
    static final double PIVOTARM_CONSTANT = 280.0 / 9.0;

    /* Public OpMode members. */
    public DcMotor frontLeftDrive     = null;
    public DcMotor backLeftDrive      = null;
    public DcMotor frontRightDrive    = null;
    public DcMotor backRightDrive     = null;
    public DcMotor extendoArm5000     = null;
    public DcMotor pivotArm1 = null;
    public DcMotor pivotArm2 = null;
    public DcMotor collectOtron       = null;
    public Servo   extensionLock      = null;
    public Servo   pivotLock          = null;

    /* Local OpMode members. */
    HardwareMap    hwMap              = null;

    /* Constructor */
    public DanielBot_Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and initialize motors
        frontLeftDrive = hwMap.dcMotor.get("front_left_drive");
        backLeftDrive = hwMap.dcMotor.get("back_left_drive");
        frontRightDrive = hwMap.dcMotor.get("front_right_drive");
        backRightDrive = hwMap.dcMotor.get("back_right_drive");

        extendoArm5000 = hwMap.dcMotor.get("extendoArm_5000");
        pivotArm1 = hwMap.dcMotor.get("pivotArm1");
        pivotArm2 = hwMap.dcMotor.get("pivotArm2");
        collectOtron = hwMap.dcMotor.get("collectOtron");


        // Set direction for all motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        extendoArm5000.setDirection(DcMotor.Direction.FORWARD);
        pivotArm1.setDirection(DcMotor.Direction.FORWARD);
        pivotArm2.setDirection(DcMotor.Direction.FORWARD);
        collectOtron.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        extendoArm5000.setPower(0);
        pivotArm1.setPower(0);
        pivotArm2.setPower(0);
        collectOtron.setPower(0);


        // Set all motors to use brake mode
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoArm5000.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectOtron.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set almost all motors to run with encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendoArm5000.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivotArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // This has no encoder
        collectOtron.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize all installed servos
        extensionLock = hwMap.servo.get("extension_lock");

        extensionLock.setPosition(.9);
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * DrivePowerAll sets all of the drive train motors to the specified power level.
     * @param power Power level to set all motors to
     */
    void DrivePowerAll (double power)
    {
        driveSetPower(power, power, power, power);
    }

    /**
     * driveSetPower sets all of the drive train motors to the specified power levels.
     * @param flPower Power level to set front left motor to
     * @param frPower Power level to set front right motor to
     * @param blPower Power level to set back left motor to
     * @param brPower Power level to set back right motor to
     */
    void driveSetPower (double flPower, double frPower, double blPower, double brPower)
    {
        frontLeftDrive.setPower(flPower);
        frontRightDrive.setPower(frPower);
        backLeftDrive.setPower(blPower);
        backRightDrive.setPower(brPower);
    }

    /**
     * driveSetMode sets all of the drive train motors to the specified mode.
     * @param runmode RunMode to set motors to
     */
    void driveSetMode(DcMotor.RunMode runmode)
    {
        frontLeftDrive.setMode(runmode);
        frontRightDrive.setMode(runmode);
        backLeftDrive.setMode(runmode);
        backRightDrive.setMode(runmode);
    }

    /**
     * driveSetMode sets all of the drive train motors to the specified positions.
     * @param flPosition Position to set front left motor to run to
     * @param frPosition Position to set front right motor to run to
     * @param blPosition Position to set back left motor to run to
     * @param brPosition Position to set back right motor to run to
     */
    void driveSetTargetPosition(int flPosition, int frPosition, int blPosition, int brPosition)
    {
        frontLeftDrive.setTargetPosition(flPosition);
        frontRightDrive.setTargetPosition(frPosition);
        backLeftDrive.setTargetPosition(blPosition);
        backRightDrive.setTargetPosition(brPosition);
    }

    boolean driveAllAreBusy()
    {
        return frontLeftDrive.isBusy() && frontRightDrive.isBusy() && backLeftDrive.isBusy()
                && backRightDrive.isBusy();
    }
}

