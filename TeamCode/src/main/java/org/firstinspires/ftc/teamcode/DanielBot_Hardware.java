package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 * This class is used to define all the specific hardware for a single robot.
 */

public class DanielBot_Hardware
{
    /* Public OpMode members. */
    public DcMotor frontLeftDrive     = null;
    public DcMotor backLeftDrive      = null;
    public DcMotor frontRightDrive    = null;
    public DcMotor backRightDrive     = null;
    public DcMotor extendoArm5000     = null;
    public DcMotor liftinator1        = null;
    public DcMotor liftinator2        = null;
    public DcMotor collectOtron       = null;

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
        liftinator1 = hwMap.dcMotor.get("liftinator1");
        liftinator2 = hwMap.dcMotor.get("liftinator2");
        collectOtron = hwMap.dcMotor.get("collectOtron");


        // Set direction for all motors
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        extendoArm5000.setDirection(DcMotor.Direction.FORWARD);
        liftinator1.setDirection(DcMotor.Direction.FORWARD);
        liftinator2.setDirection(DcMotor.Direction.FORWARD);
        collectOtron.setDirection(DcMotor.Direction.FORWARD);


        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

        extendoArm5000.setPower(0);
        liftinator1.setPower(0);
        liftinator2.setPower(0);
        collectOtron.setPower(0);


        // Set all motors to use brake mode
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendoArm5000.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftinator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftinator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        collectOtron.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to run with encoders
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendoArm5000.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftinator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftinator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectOtron.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}

