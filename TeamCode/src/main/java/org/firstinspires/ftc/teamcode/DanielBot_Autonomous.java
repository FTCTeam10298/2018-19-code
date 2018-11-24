/* Copyright (c) 2018 Brain Stormz. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of Brain Stormz, nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Mat;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

import java.lang.Math;
import java.util.List;

@Autonomous(name="DanielBot Autonomous", group ="DanielBot")
public class DanielBot_Autonomous extends LinearOpMode implements FtcMenu.MenuButtons {
    public enum StartPosition {
        SILVER,
        GOLD
    }

    public enum RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }

    public enum Lift {
        YES,
        NO
    }

    public enum Sampling {
        ZERO,
        ONE,
        TWO
    }

    public enum Gold {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum Crater {
        NEAR,
        FAR
    }

    public enum Depot {
        YES,
        NO
    }


    // Menu option variables
    RunMode runmode = RunMode.RUNMODE_AUTO;
    int delay = 0;
    StartPosition startposition = StartPosition.SILVER;
    Lift hanging = Lift.YES;
    Crater crater = Crater.NEAR;
    Gold gold = Gold.CENTER;
    Depot depot = Depot.YES;
    Sampling sampling = Sampling.ZERO;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    DanielBot_Hardware robot = new DanielBot_Hardware();

    static final double COUNTS_PER_MOTOR_REV = 1120;    // Rev HD Hex v2.1 Motor encoder
    static final double DRIVE_GEAR_REDUCTION = .825;       // This is < 1.0 if geared for torque
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    static final double PIVOTARM_CONSTANT = 280 / 9; //constant that converts liftonator to degrees (1120*10/360)
    static final double EXTENDOARM_CONSTANT = 6720 / (13 * Math.PI); //constant that converts ExtendoArm to inches 1120/(2.16666667 * 3.14159265)

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Aez7J07/////AAABmS8UUfYgtke9gtnv8lpOtUwWoahEf1DgQVHSXzYz9B1RjnN9agKHZOutrQmzOMd57S7KpZKOt6yvLF4rGqzWR4Re/EhtUIe1+MD9DYlEkHHX3bio1F0kblG0BgzIAxmM+u+2L10gHO4pyuUnPohg7/T5mY912NuZGocuhq65i20+xV1b3bjStwuZaKY14lXvEklvO8ZcFvR26928fxmJIVWtRqashdFZ5nxm1w/sqJ9eWJQv3mLZt7AVrclS5NwPxzSwlX6+8IK8dWIOOJuZx0mKVlQMNoAEXioCuIuuFwAMBtm+NMkjXTxtfiShYXnfD7e1pAUsMgksQBGl1cCqcpLr8dD1msx9nOlI6fsfMY9e";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap);
        //robot.pivotLock.setPosition(1);
        robot.extensionLock.setPosition(0);
        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);

        // Initialize Vuforia ----------------------------------------------------------------------
        initVuforia();

        // Run though the menu ---------------------------------------------------------------------
        doMenus();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            dashboard.displayPrintf(1,"Sorry! This device is not compatible with TFOD");
        }

        dashboard.displayPrintf(0, "Status: Ready to start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

        // Pause the program for the selected delay period
        sleep(delay);

        // Init
        if (DoTask("Init", runmode)) {
            TensorFlow();
            if (hanging == Lift.YES) {
                //robot.pivotLock.setPosition(.5);
                //PivotArmUnlock();
                PivotArmSetRotation(.5, -10);
                sleep(100);
                PivotArmSetRotation(.5, 90);
                robot.extensionLock.setPosition(.5);
                sleep(100);
                ExtendoArm5000_UNLATCH();
                ExtendoArm5000_ACTIVATE(.5, 20);
                sleep(500);
                PivotArmSetRotation(1, -80);
                sleep(500);
                ExtendoArm5000_ACTIVATE(.3, -20);
            } else {
                DriveRobotPosition(1, -3);
                sleep(500);
                DriveRobotTurn(.35, 180);
            }
        }

        if (DoTask("Mineral Sampling", runmode)) {
            DriveRobotPosition(.5, 3);
            if (sampling == Sampling.ONE || sampling == Sampling.TWO)
                DriveSample();
            else
                DriveRobotPosition(1, 20);
        }

        if (DoTask("Drive my Car", runmode)) {
            if (depot == Depot.NO && (sampling == Sampling.ONE || sampling == Sampling.ZERO)
                    && crater == Crater.NEAR) {
                DriveRobotPosition(1, 16);
            }
            else if (depot == Depot.YES) {
                if (startposition == StartPosition.GOLD && crater == Crater.NEAR)
                    DriveRobotTurn(1, 90);
                else if (startposition == StartPosition.SILVER ||
                        (startposition == StartPosition.GOLD && crater == Crater.FAR))
                    DriveRobotTurn(1, -90);
                sleep(500);
                DriveRobotPosition(1, 30);
                sleep(500);
                if (startposition == StartPosition.GOLD && crater == Crater.FAR)
                    DriveRobotTurn(1, 135);
                else if ((startposition == StartPosition.GOLD && crater == Crater.NEAR) ||
                        (startposition == StartPosition.SILVER))
                    DriveRobotTurn(1, -135);
                sleep(500);
                if (startposition == StartPosition.SILVER ||
                        startposition == StartPosition.GOLD && crater == Crater.NEAR) {
                    DriveSidewaysTime(2, -1); //right
                    DriveRobothug(1, 30, false);
                } else if (startposition == StartPosition.GOLD && crater == Crater.FAR) {
                    DriveSidewaysTime(2, 1); //left
                    DriveRobothug(1, 30, true);
                }
                robot.collectOtron.setPower(-1);
                sleep(1000);
                robot.collectOtron.setPower(0);
                if (sampling == Sampling.ONE)
                    DriveRobotPosition(1, -60);
                else {//im here FIXME rest of Drive my Car (Beep Beep, Beep Beep, Yeah)
                    DriveRobothug(1, -30, false);
                    DriveSidewaysTime(2, 1);
                    DriveRobotTurn(1, 135);
                    sleep(500);
                    DriveRobotPosition(1, 30);
                    sleep(500);
                    DriveRobotTurn(1, -90);
                    DriveSample();
                    DriveRobotPosition(1, 26);
                }
            }

        }
        if (DoTask("Park", runmode)) {

        }

    }


    /*
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    boolean DoTask (String taskname, RunMode debug)
    {
        dashboard.displayPrintf(0, taskname);
        if (debug == RunMode.RUNMODE_DEBUG)
        {
            dashboard.displayPrintf(1, "Press A to run, B to skip");
            while (opModeIsActive()) {
                if (gamepad1.a) {
                    dashboard.displayPrintf(1, "Run");
                    return true;
                }
                if (gamepad1.b) {
                    dashboard.displayPrintf(1, "Skip");
                    sleep(1000);
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * DriveRobotTime drives the robot the set number of inches at the given power level.
     * @param ms How long to drive
     * @param power Power level to set motors to, negative will drive the robot backwards
     */
    void DriveRobotTime(int ms, double power)
    {
        DrivePowerAll(power);
        sleep(ms);
        DrivePowerAll(0);
    }

    /**
     * DriveRobotPosition drives the robot the set number of inches at the given power level.
     * @param inches How far to drive, can be negative
     * @param power Power level to set motors to
     */
    void DriveRobotPosition(double power, double inches)
    {
        double position = inches*COUNTS_PER_INCH;

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DrivePowerAll(power);

        robot.frontLeftDrive.setTargetPosition((int)position);
        robot.frontRightDrive.setTargetPosition((int)position);
        robot.backLeftDrive.setTargetPosition((int)position);
        robot.backRightDrive.setTargetPosition((int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy()
                    && robot.backRightDrive.isBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        DrivePowerAll(0);
    }


    void DriveRobotTurn (double power, double degree)
    {
//        double position = degree*DRIVE_GEAR_REDUCTION*15.2; // FIXME: Magic number
        double position = degree*COUNTS_PER_INCH*0.20672; // FIXME: Magic number

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(-power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(-power);

        robot.frontLeftDrive.setTargetPosition((int)position);
        robot.frontRightDrive.setTargetPosition(-(int)position);
        robot.backLeftDrive.setTargetPosition((int)position);
        robot.backRightDrive.setTargetPosition(-(int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy()
                    && robot.backRightDrive.isBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        DrivePowerAll(0);
    }

    void DriveRobotSideways (double power, double inches)
    {
        double position = inches*COUNTS_PER_INCH * 2;

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (power > 0) // Drive right
        {
            robot.frontLeftDrive.setTargetPosition((int)-position);
            robot.backLeftDrive.setTargetPosition((int)position);
            robot.backRightDrive.setTargetPosition((int)-position);
            robot.frontRightDrive.setTargetPosition((int)position);
        }
        else // Drive left
        {
            robot.frontRightDrive.setTargetPosition((int)position);
            robot.backRightDrive.setTargetPosition((int)-position);
            robot.backLeftDrive.setTargetPosition((int)position);
            robot.frontLeftDrive.setTargetPosition((int)-position);
        }
        robot.frontRightDrive.setPower(power);
        robot.backRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.frontLeftDrive.setPower(power);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy()
                    && robot.backRightDrive.isBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        DrivePowerAll(0);
    }

    /**
     * DrivePowerAll sets all of the drive train motors to the specified power level.
     * @param power Power level to set motors to
     */
    void DrivePowerAll (double power)
    {
        robot.frontLeftDrive.setPower(power);
        robot.frontRightDrive.setPower(power);
        robot.backLeftDrive.setPower(power);
        robot.backRightDrive.setPower(power);
    }

    /**
     * DriveSidewaysTime makes the robot drive sideways for the specified time and power.
     * @param time How long to drive in seconds
     * @param power The power to use while driving,
     *              positive values go right and negative values go left
     */
    void DriveSidewaysTime (int time, double power)
    {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (power > 0) // Drive left
        {
            robot.frontLeftDrive.setPower(-power);
            robot.backLeftDrive.setPower(power);
            robot.backRightDrive.setPower(-power);
            robot.frontRightDrive.setPower(power);
        }
        else // Drive right
        {
            robot.frontRightDrive.setPower(power);
            robot.backRightDrive.setPower(-power);
            robot.backLeftDrive.setPower(power);
            robot.frontLeftDrive.setPower(-power);
        }
        // Continue driving for the specified amount of time, then stop
        sleep(time*1000);
        DrivePowerAll(0);
    }

    /**
     * DriveRobotHug is used to make the robot drive hugging a wall.
     * The robot will move mostly straight and slightly to the side,
     * so it will stay against the wall.
     * @param power Power to use while driving
     * @param inches How many inches to drive
     * @param hugLeft Whether to hug left or right
     */
    void DriveRobothug (double power, int inches, boolean hugLeft)
    {
        double position = inches*COUNTS_PER_INCH;

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((!hugLeft && inches > 0) || (hugLeft && inches < 0)) {
            robot.frontLeftDrive.setPower(power*.9);
            robot.frontRightDrive.setPower(power);
            robot.backRightDrive.setPower(power*.9);
            robot.backLeftDrive.setPower(power);
        }
        else if ((!hugLeft && inches < 0) || (hugLeft && inches > 0))
        {
            robot.frontLeftDrive.setPower(power);
            robot.frontRightDrive.setPower(power*.9);
            robot.backRightDrive.setPower(power);
            robot.backLeftDrive.setPower(power*.9);
        }

        robot.frontLeftDrive.setTargetPosition((int)position);
        robot.frontRightDrive.setTargetPosition((int)position);
        robot.backRightDrive.setTargetPosition((int)position);
        robot.backLeftDrive.setTargetPosition((int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy()
                    && robot.backRightDrive.isBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        DrivePowerAll(0);

    }

    void DriveSample () {
        if (gold != Gold.CENTER) {
            if (gold == Gold.LEFT)
                DriveRobotTurn(.7, -35);
            else if (gold == Gold.RIGHT)
                DriveRobotTurn(.7, 35);
            DriveRobotPosition(1, 24);
            if (sampling == Sampling.TWO) {
                sleep(500);
                DriveRobotPosition(1, -24);
                if (gold == Gold.LEFT)
                    DriveRobotTurn(.7, 35);
                else if (gold == Gold.RIGHT)
                    DriveRobotTurn(.7, -35);
            }
        } else {
            DriveRobotPosition(1, 20);
            sleep(500);
            if (!(depot == Depot.NO && (sampling == Sampling.ONE || sampling == Sampling.ZERO)
                    && crater == Crater.NEAR))
                DriveRobotPosition(1, -20);
        }
    }

    /**
     * PivotArmSetRotation
     * Positive swings back
     * @param power Power level
     * @param degrees Degrees of rotation
     */
    void PivotArmSetRotation(double power, double degrees)
    {
        int position = (int)(degrees* PIVOTARM_CONSTANT);
        robot.pivotArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pivotArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pivotArm1.setPower(power);
        robot.pivotArm2.setPower(power);
        robot.pivotArm1.setTargetPosition(position);
        robot.pivotArm2.setTargetPosition(position);
        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.pivotArm1.isBusy() && robot.pivotArm2.isBusy()) {
                dashboard.displayPrintf(10, "The ENEMY gates are down!");
            }
            sleep(10);
        }
        robot.pivotArm1.setPower(0);
        robot.pivotArm2.setPower(0);
    }

    /**
     * PivotArmUnlock
     */
    void PivotArmUnlock()
    {
        robot.pivotArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivotArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.pivotArm1.setPower(1);
        robot.pivotArm2.setPower(1);
        sleep(150);
        robot.pivotArm1.setPower(-1);
        robot.pivotArm2.setPower(-1);
        sleep(150);
        robot.pivotArm1.setPower(1);
        robot.pivotArm2.setPower(1);
        sleep(150);
        robot.pivotArm1.setPower(-1);
        robot.pivotArm2.setPower(-1);
        sleep(150);
        robot.pivotArm1.setPower(1);
        robot.pivotArm2.setPower(1);
        sleep(150);
        robot.pivotArm1.setPower(-1);
        robot.pivotArm2.setPower(-1);
        sleep(150);
        robot.pivotArm1.setPower(0);
        robot.pivotArm2.setPower(0);
    }

    /**
     * ExtendoArm_ACTIVATE ACTIVATES the ExtendoArm
     * Positive extends
     * @param power
     * @param inches
     */
    void ExtendoArm5000_ACTIVATE (double power, double inches)
    {
        int position = (int)(-inches*EXTENDOARM_CONSTANT);
        robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extendoArm5000.setPower(power);
        robot.extendoArm5000.setTargetPosition(position);
        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.extendoArm5000.isBusy()) {
                dashboard.displayPrintf(10, "Don't fire at the ENEMY until you see the whites of their eyes!");
            }
            sleep(10);
        }
        robot.extendoArm5000.setPower(0);
    }

    /**
     * ExtendoArm_UNLATCH unlatches the ExtendoArm
     */
    void ExtendoArm5000_UNLATCH()
    {
        robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extendoArm5000.setPower(1);
        sleep(150);
        robot.extendoArm5000.setPower(-1);
        sleep(150);
        robot.extendoArm5000.setPower(1);
        sleep(150);
        robot.extendoArm5000.setPower(-1);
        sleep(150);
        robot.extendoArm5000.setPower(1);
        sleep(150);
        robot.extendoArm5000.setPower(-1);
        sleep(150);
        robot.extendoArm5000.setPower(0);
    }

    /**
     * Determines mineral with TensorFlow
     */
    void TensorFlow () {
        // Sample minerals with TensorFlow
        if (opModeIsActive() && DoTask("Sample minerals with TensorFlow", runmode)) {
            // Activate TensorFlow Object Detection.
            if (tfod != null) {
                tfod.activate();
            }
            int i = 0;
            ElapsedTime holdTimer = new ElapsedTime();
            while (opModeIsActive() && (holdTimer.time() < 2)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        dashboard.displayPrintf(1,"%d objects detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 1) {
                            double left = Math.random()/2;
                            double right = Math.random()/2;
                            double center = Math.random()/2;
                            int vote = 0;
                            final int WIDTH = updatedRecognitions.get(0).getImageWidth();
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL))
                                    vote++;
                                else
                                    vote--;
                                if ((int) recognition.getLeft() < WIDTH/3)
                                    left+=vote;
                                else if ((int) recognition.getLeft() < 2*WIDTH/3)
                                    center+=vote;
                                else
                                    right+=vote;
                                vote=0;
                            }
                            if (left > center && left > right) {
                                gold=Gold.LEFT;
                                if (left > .5)
                                    break;
                                dashboard.displayPrintf(2, "Left");
                            }
                            else if (center > right) {
                                gold=Gold.CENTER;
                                if (center > .5)
                                    break;
                                dashboard.displayPrintf(2, "Center");
                            }
                            else {
                                gold=Gold.RIGHT;
                                if (right > .5)
                                    break;
                                dashboard.displayPrintf(2, "Right");
                            }
                        }
                    }
                }
                i++;
                dashboard.displayPrintf(2, "TensorFlow iteration count: %d", i);
            }
            // Shutdown TensorFlow
            if (tfod != null) {
                tfod.shutdown();
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * Samples a random mineral
     */
    void randomMineral () {
        double random = Math.random();
        if (random < (1 / 3))
            gold = Gold.LEFT;
        else if (random < (2 / 3))
            gold = Gold.CENTER;
        else
            gold = Gold.RIGHT;
    }


    // MENU ----------------------------------------------------------------------------------------
    @Override
    public boolean isMenuUpButton() { return gamepad1.dpad_up; }

    @Override
    public boolean isMenuDownButton() { return gamepad1.dpad_down; }

    @Override
    public boolean isMenuEnterButton() { return gamepad1.dpad_right; }

    @Override
    public boolean isMenuBackButton() { return gamepad1.dpad_left; }

    private void doMenus() {
        FtcChoiceMenu<RunMode> modeMenu = new FtcChoiceMenu<>("Run Mode", null, this);
        FtcValueMenu delayMenu = new FtcValueMenu("Delay:", modeMenu, this, 0, 20000, 1000, 0, "%.0f msec");
        FtcChoiceMenu<StartPosition> startPositionMenu = new FtcChoiceMenu<>("Start Position:", delayMenu, this);
        FtcChoiceMenu<Lift> liftMenu = new FtcChoiceMenu<>("Hanging:", startPositionMenu, this);
        FtcChoiceMenu<Sampling> samplingMenu = new FtcChoiceMenu<>("Number of Samples:", liftMenu, this);
        FtcChoiceMenu<Depot> depotMenu = new FtcChoiceMenu<>("Go for Depot:", samplingMenu, this);
        FtcChoiceMenu<Crater> craterMenu = new FtcChoiceMenu<>("Crater:", depotMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, delayMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, delayMenu);

        delayMenu.setChildMenu(startPositionMenu);

        startPositionMenu.addChoice("Silver Side", StartPosition.SILVER, true, liftMenu);
        startPositionMenu.addChoice("Gold Side", StartPosition.GOLD, false, liftMenu);

        liftMenu.addChoice("Yes", Lift.YES, true, samplingMenu);
        liftMenu.addChoice("No", Lift.NO, false, samplingMenu);

        samplingMenu.addChoice("0", Sampling.ZERO, false, depotMenu);
        samplingMenu.addChoice("1", Sampling.ONE, true, depotMenu);
        samplingMenu.addChoice("2", Sampling.TWO, false, depotMenu);

        depotMenu.addChoice("Yes", Depot.YES, true, craterMenu);
        depotMenu.addChoice("No", Depot.NO, false, craterMenu);

        craterMenu.addChoice("Near", Crater.NEAR, true);
        craterMenu.addChoice("Far", Crater.FAR, false);

        FtcMenu.walkMenuTree(modeMenu, this);
        runmode = modeMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentValue();
        hanging = liftMenu.getCurrentChoiceObject();
        startposition = startPositionMenu.getCurrentChoiceObject();
        depot = depotMenu.getCurrentChoiceObject();
        sampling = samplingMenu.getCurrentChoiceObject();
        crater = craterMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
