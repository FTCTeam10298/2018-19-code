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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

import static java.lang.Math.abs;

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
        RIGHT,
        UNKNOWN
    }

    public enum Crater {
        NEAR,
        FAR,
        NONE
    }

    public enum Depot {
        YES,
        NO
    }

    public enum ExtraScore {
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
    ExtraScore attemptExtraScore = ExtraScore.YES;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    DanielBot_Hardware robot = new DanielBot_Hardware();

    static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    static final double GEARBOX_RATIO         = 20.0;      // 40 for 40:1, 20 for 20:1
    static final double DRIVE_GEAR_REDUCTION  = 24.0/15.0; // This is > 1.0 if geared for torque
    static final double WHEEL_DIAMETER_INCHES = 4.0;       // For figuring circumference
    static final double DRIVETRAIN_ERROR      = 1.04;      // Error determined from testing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR;
    static final double COUNTS_PER_DEGREE     = COUNTS_PER_INCH*0.20672; // Found by testing

    static final double PIVOTARM_CONSTANT     = 1440.0 * 10.0 / 360.0; // Constant that converts pivot arm to degrees (1120*10/360 for Rev 40:1)
    static final double EXTENDOARM_CONSTANT   = 1120.0 * 2.0 * 15.0 / 25.0 / (3. * Math.PI); // Constant that converts ExtendoArm to inches 1120 * 2/(3 * 3.14159265)

    ModernRoboticsI2cRangeSensor rangeSensor;

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

        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);

        // Initialize Vuforia ----------------------------------------------------------------------
        initVuforia();

        // Initialize Sensor
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        // Run though the menu ---------------------------------------------------------------------
        doMenus();

        if (startposition == StartPosition.SILVER && crater == Crater.FAR) {
            dashboard.displayPrintf(1, "You picked the wrong option, Stephanie!");
            crater = Crater.NEAR;
        }

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            dashboard.displayPrintf(1,"THE ZTE SPEEEEED IS TOO FAST FOR TFOD");
        }

        dashboard.displayPrintf(0, "Status: Ready to start");

//        // Motor test (Uncomment out to test drive train encoders)
//        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.backLeftDrive.setPower(0.5);
//        robot.backRightDrive.setPower(0.5);
//        robot.frontLeftDrive.setPower(0.5);
//        robot.frontRightDrive.setPower(0.5);
//
//        robot.backRightDrive.setTargetPosition(5000);
//        while (robot.backRightDrive.isBusy()) {
//            sleep(10);
//        }
//
//        robot.backLeftDrive.setTargetPosition(5000);
//        while (robot.backLeftDrive.isBusy()) {
//            sleep(10);
//        }
//
//        robot.frontRightDrive.setTargetPosition(5000);
//        while (robot.frontRightDrive.isBusy()) {
//            sleep(10);
//        }
//
//        robot.frontLeftDrive.setTargetPosition(5000);
//        while (robot.frontLeftDrive.isBusy()) {
//            sleep(10);
//        }
//
//
//        //test spin
//        DriveRobotTurn(.3, 360);
//        sleep(1000);
//        DriveRobotTurn(.3, -360);
//
//        DriveRobotHug(1, 48, false);

        // Activate TensorFlow Object Detection.
        if (tfod != null) {
            tfod.activate();
        }

        // Sample while waiting for start
        while (!opModeIsActive() && !isStopRequested()) { //DoTask("Sample minerals with TensorFlow", runmode)
            Gold sample = getSampleFromTensorFlow();
            if (sample != Gold.UNKNOWN) {
                gold = sample;
            }
        }

        // Shutdown TensorFlow
        if (tfod != null) {
            tfod.shutdown();
        }

        // Wait for the game to start (driver presses PLAY)
        //waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

        // Pause the program for the selected delay period
        sleep(delay);

        // Init
        if (DoTask("Init", runmode)) {
            if (hanging == Lift.YES) {
                ExtendoArm5000_ACTIVATE(1,5, false);
                PivotArmSetRotation(.5, -20);
                PivotArmSetRotation(.5, 95, true, false);
                DriveRobotPosition(.3, 8);
                ExtendoArm5000_ACTIVATE(1,-1.5, true);
                if (startposition == StartPosition.GOLD && depot == Depot.YES) {
                    PivotArmSetRotation(1, -63);
                } else {
                    PivotArmSetRotation(1, -88);
                }
            } else {
                DriveRobotPosition(1, 10);
            }
        }

        if (DoTask("Mineral Sampling", runmode)) {
            if (startposition == StartPosition.GOLD && depot == Depot.YES) {
                DriveRobotPosition(0.75, 13, true);
                ExtendoArm5000_ACTIVATE(1, 21, false);
                PivotArmSetRotation(1, -10, false, true);

                robot.collectOtron.setPower(.7);
                sleep(1000);
                robot.collectOtron.setPower(0);

                PivotArmSetRotation(1, 10, false, true);
                ExtendoArm5000_ACTIVATE(1, -20, false);
                DriveRobotPosition(0.75, -13, true);
                PivotArmSetRotation(1, -25);
            }

            if (sampling == Sampling.ONE || sampling == Sampling.TWO)
                DriveSample();
            else
                DriveRobotPosition(.5, 20);
        }

        if (DoTask("Drive my Car", runmode)) {
            if (startposition == StartPosition.GOLD) {
                if (sampling == Sampling.ONE && attemptExtraScore == ExtraScore.YES) {
                    PivotArmSetRotation(1, 115, false, true);
                    ExtendoArm5000_ACTIVATE(1, 15, false);
                    sleep(500);
                    robot.collectorGate.setPosition(.25);
                    PivotArmSetRotation(1, -5, false, true);
                    sleep(200);
                    PivotArmSetRotation(1, 5, false, true);
                    sleep(200);
                    PivotArmSetRotation(1, -5, false, true);
                    sleep(200);
                    PivotArmSetRotation(1, 5, false, true);
                    sleep(200);
                    robot.collectorGate.setPosition(.65);
                    PivotArmSetRotation(1, -60, false, true);
                    sleep(200);
                    ExtendoArm5000_ACTIVATE(1, -15, true);
                }

                DriveRobotPosition(1, 8, true);

                if (crater == Crater.FAR) {
                    DriveRobotTurn(.5, -70);
                    DriveRobotPosition(.8, 40, true);
                    DriveRobotTurn(1, -45, false);
                    DriveRobotPosition(1, 30, false);
                }
                else if (crater == Crater.NEAR) {
                    DriveRobotTurn(.5, -90);
                    DriveRobotPosition(.8, -50, true);
                    DriveRobotTurn(1, 40, false);
                    DriveRobotPosition(1, -11, false);
                }
            } else if (startposition == StartPosition.SILVER && depot == Depot.YES) {
                if (sampling == Sampling.ONE && attemptExtraScore == ExtraScore.YES) {
                    PivotArmSetRotation(1, 115, false, true);
                    DriveRobotTurn(1, 20, true);
                    ExtendoArm5000_ACTIVATE(1, 19, false);
                    sleep(500);
                    robot.collectorGate.setPosition(.25);
                    PivotArmSetRotation(1, -5, false, true);
                    sleep(200);
                    PivotArmSetRotation(1, 5, false, true);
                    sleep(200);
                    PivotArmSetRotation(1, -5, false, true);
                    sleep(200);
                    PivotArmSetRotation(1, 5, false, true);
                    sleep(200);
                    robot.collectorGate.setPosition(.65);
                    PivotArmSetRotation(1, -60, false, true);
                    sleep(200);
                    ExtendoArm5000_ACTIVATE(1, -19, true);
                    DriveRobotTurn(1, -20, true);
                }
                DriveRobotPosition(.5, 9.5, true);
                DriveRobotTurn(.6, -90, true);
                // Drive to wall then to depot
                DriveRobotPosition(.7, 50, true);
                DriveRobotTurn(.3, -35);
                //DriveSidewaysTime(.5, 1);
                DriveRobotHug(1, 38, false);

                // Deposit team marker
                DriveRobotTurn(1, -5);
                robot.markerDumper.setPosition(0.7);
//                if (sampling == Sampling.ONE) {
//                    robot.collectOtron.setPower(.7);
//                    sleep(1000);
//                    robot.collectOtron.setPower(0);
//                } else {
                    sleep(250);
//                }

                if (sampling == Sampling.ONE && crater == Crater.NEAR) {
                    DriveRobotPosition(1, -50);
                    DriveSidewaysTime(1, -1); // Strafe right
                    DriveRobotPosition(.6, -25);
                }
                else if (sampling == Sampling.TWO) {
                    // Drive back varying amounts depending on sample
                    if (gold == Gold.LEFT)
                        DriveRobotDistanceToObject(1, 14);
                    else if (gold == Gold.CENTER)
                        DriveRobotDistanceToObject(1, 26);
                    else
                        DriveRobotDistanceToObject(1, 36);

                    DriveRobotTurn(1, -90, true);

                    // Extend arm and collect gold mineral
                    robot.collectOtron.setPower(1);
                    if (gold == Gold.LEFT) {
                        PivotArmSetRotation(1, -48, false, true);
                        DriveRobotPosition(.7, 13);
                        ExtendoArm5000_ACTIVATE(1, 12, false);
                        ExtendoArm5000_ACTIVATE(1, -12, true);
                        DriveRobotPosition(.7, -13);
                    } else if (gold == Gold.CENTER) {
                        PivotArmSetRotation(1, -48, false, true);
                        ExtendoArm5000_ACTIVATE(1, 18, false);
                        ExtendoArm5000_ACTIVATE(1, -8, false);
                        ExtendoArm5000_ACTIVATE(1, -10, true);
                    } else {
                        PivotArmSetRotation(1, -52, false, false);
                        ExtendoArm5000_ACTIVATE(1, 5, false);
                        ExtendoArm5000_ACTIVATE(1, -5, false);
                    }

                    PivotArmSetRotation(1, 45, false, true);

                    DriveRobotTurn(1, 90);

                    DriveRobotDistanceToObject(1, 60);

                    DriveSidewaysTime(0.5, -1);

                    DriveRobotPosition(1, -36);
                }
            }
        }
//        if (DoTask("Park", runmode)) {
//
//        }

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
        robot.DrivePowerAll(power);
        sleep(ms);
        robot.DrivePowerAll(0);
    }

    /**
     * DriveRobotDistanceToObject drives the robot to the set number of inches from an object
     * (usually the wall) at the given power level.
     * @param inches How many inches away to the object to go to
     * @param power Power level to set motors to
     */
    void DriveRobotDistanceToObject(double power, double inches)
    {
        double target = (float)rangeSensor.getDistance(DistanceUnit.INCH) - inches; // FIXME: how accurate is sensor?
        dashboard.displayPrintf(10, "Range Sensor: ", rangeSensor.getDistance(DistanceUnit.INCH));
        DriveRobotPosition(abs(power), target); // Use abs() to make sure power is positive
    }

    /**
     * DriveRobotPosition drives the robot the set number of inches at the given power level.
     * @param inches How far to drive, can be negative
     * @param power Power level to set motors to
     */
    void DriveRobotPosition(double power, double inches, boolean smart_accel)
    {
        int state = 0; // 0 = NONE, 1 = ACCEL, 2 = DRIVE, 3 = DECEL
        double position = inches*COUNTS_PER_INCH;

        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (smart_accel)
        {
            robot.DrivePowerAll(abs(power)/2); // Use abs() to make sure power is positive
            state = 1; // ACCEL
        }
        else {
            robot.DrivePowerAll(abs(power)); // Use abs() to make sure power is positive
        }

        robot.driveSetTargetPosition((int)position, (int)position, (int)position, (int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.driveAllAreBusy()) {
                int flDrive = robot.frontLeftDrive.getCurrentPosition();
                int frDrive = robot.frontRightDrive.getCurrentPosition();
                int blDrive = robot.backLeftDrive.getCurrentPosition();
                int brDrive = robot.backRightDrive.getCurrentPosition();
                dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                dashboard.displayPrintf(4, "Front right encoder: %d", frDrive);
                dashboard.displayPrintf(5, "Back left encoder: %d", blDrive);
                dashboard.displayPrintf(6, "Back right encoder %d", brDrive);

                // State magic
                if (state == 1 &&
                        (abs(flDrive) > 2*COUNTS_PER_INCH ||
                         abs(frDrive) > 2*COUNTS_PER_INCH ||
                         abs(blDrive) > 2*COUNTS_PER_INCH ||
                         abs(brDrive) > 2*COUNTS_PER_INCH )) {
                    // We have gone 2 inches, go to full power
                    robot.DrivePowerAll(abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (abs(flDrive) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(frDrive) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(blDrive) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(brDrive) > COUNTS_PER_INCH*(abs(inches)-2) )) {
                    // Cut power by half to DECEL
                    robot.DrivePowerAll(abs(power)/2); // Use abs() to make sure power is positive
                    state = 3; // We are DECELing now
                }
                dashboard.displayPrintf(7, "State: %d (0=NONE,1=ACCEL,2=DRIVING,3=DECEL", state);
            }
            sleep(10);
        }

        robot.DrivePowerAll(0);
        // Clear used section of dashboard 
        dashboard.displayText(3, "");
        dashboard.displayText(4, "");
        dashboard.displayText(5, "");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
    }

    /** For compatibility */
    void DriveRobotPosition(double power, double inches)
    {
        DriveRobotPosition(power, inches, false);
    }

    void DriveRobotTurn (double power, double degree, boolean smart_accel)
    {
        double position = degree*COUNTS_PER_DEGREE;
        //FIXME: left turns overshoot
        if (degree > 0)
            position *= .9;

        int state = 0; // 0 = NONE, 1 = ACCEL, 2 = DRIVE, 3 = DECEL

        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (smart_accel) {
            state = 1;
            robot.driveSetPower(power*0.5, -power*0.5, power*0.5, -power*0.5);
        }
        else
        {
            robot.driveSetPower(power, -power, power, -power);
        }

        robot.driveSetTargetPosition((int)position, -(int)position, (int)position, -(int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.driveAllAreBusy()) {
                int flDrive = robot.frontLeftDrive.getCurrentPosition();
                int frDrive = robot.frontRightDrive.getCurrentPosition();
                int blDrive = robot.backLeftDrive.getCurrentPosition();
                int brDrive = robot.backRightDrive.getCurrentPosition();
                dashboard.displayPrintf(3, "Front left encoder: %d", flDrive);
                dashboard.displayPrintf(4, "Front right encoder: %d", frDrive);
                dashboard.displayPrintf(5, "Back left encoder: %d", blDrive);
                dashboard.displayPrintf(6, "Back right encoder %d", brDrive);

                // State magic
                if (state == 1 &&
                        (abs(flDrive) > COUNTS_PER_DEGREE*10 ||
                         abs(frDrive) > COUNTS_PER_DEGREE*10 ||
                         abs(blDrive) > COUNTS_PER_DEGREE*10 ||
                         abs(brDrive) > COUNTS_PER_DEGREE*10 )) {
                    // We have rotated 10 degrees, go to full power
                    robot.DrivePowerAll(abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (abs(flDrive) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(frDrive) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(blDrive) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(brDrive) > COUNTS_PER_DEGREE*(abs(degree)-10) )) {
                    // We are within 10 degrees of our destination, cut power by half to DECEL
                    robot.DrivePowerAll(abs(power)/2); // Use abs() to make sure power is positive
                    state = 3; // We are DECELing now
                }
                dashboard.displayPrintf(7, "State: %d (0=NONE,1=ACCEL,2=DRIVING,3=DECEL", state);
            }
            sleep(10);
        }

        robot.DrivePowerAll(0);
        // Clear used section of dashboard 
        dashboard.displayText(3, "");
        dashboard.displayText(4, "");
        dashboard.displayText(5, "");
        dashboard.displayText(6, "");
        dashboard.displayText(7, "");
    }

    /** For compatibility */
    void DriveRobotTurn (double power, double degree)
    {
        DriveRobotTurn(power, degree, false);
    }

    /**
     * DriveSidewaysTime makes the robot drive sideways for the specified time and power.
     * @param time How long to drive in seconds
     * @param power The power to use while driving,
     *              positive values go right and negative values go left
     */
    void DriveSidewaysTime (double time, double power)
    {
        robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.driveSetPower(-power, power, power, -power);

        // Continue driving for the specified amount of time, then stop
        double ms = time*1000;
        sleep((int)ms);
        robot.DrivePowerAll(0);
    }

    /**
     * DriveRobotHug is used to make the robot drive hugging a wall.
     * The robot will move mostly straight and slightly to the side,
     * so it will stay against the wall.
     * @param power Power to use while driving
     * @param inches How many inches to drive
     * @param hugLeft Whether to hug left or right
     */
    void DriveRobotHug(double power, int inches, boolean hugLeft)
    {
        double position = inches*COUNTS_PER_INCH;

        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        if ((!hugLeft && inches > 0) || (hugLeft && inches < 0)) {
            robot.driveSetPower(power*.5, power, power, power*.5);
        }
        else if ((!hugLeft && inches < 0) || (hugLeft && inches > 0))
        {
            robot.driveSetPower(power, power*.5, power*.5, power);
        }

        robot.driveSetTargetPosition((int)position, (int)position, (int)position, (int)position);

        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.driveAllAreBusy()) {
                dashboard.displayPrintf(3, "Left front encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(4, "Right front encoder: %d", robot.frontRightDrive.getCurrentPosition());
                dashboard.displayPrintf(5, "Left back encoder: %d", robot.frontLeftDrive.getCurrentPosition());
                dashboard.displayPrintf(6, "Right back encoder %d", robot.backRightDrive.getCurrentPosition());
            }
            sleep(10);
        }

        robot.DrivePowerAll(0);

    }

    void DriveSample () {
        robot.collectOtron.setPower(1);
        if (gold != Gold.CENTER) {
            if (gold == Gold.LEFT)
                DriveRobotTurn(.6, -32, true);
            else if (gold == Gold.RIGHT)
                DriveRobotTurn(.6, 32, true);
            DriveRobotPosition(.6, 24, true);
            if (startposition == StartPosition.SILVER && depot == Depot.NO && crater == Crater.NEAR)
                DriveRobotPosition(1, 15);
            else {
                DriveRobotPosition(.6, -24, true);
                if (gold == Gold.LEFT)
                    DriveRobotTurn(.6, 35, true);
                else if (gold == Gold.RIGHT)
                    DriveRobotTurn(.6, -32, true);
            }
        } else { // gold == Gold.CENTER
            DriveRobotPosition(.6, 20, true);
            if (!(depot == Depot.NO && startposition == StartPosition.SILVER && crater == Crater.NEAR))
                DriveRobotPosition(.6, -20, true);
        }
        robot.collectOtron.setPower(0);
        PivotArmSetRotation(1, 55, false, true);
    }

    /**
     * PivotArmSetRotation
     * Positive swings up/back
     * @param power Power level
     * @param degrees Degrees of rotation
     * @param unlatch Drives wheels backwards for unlatching purposes
     * @param asynkk Allows for another function to take place simulataneously
     */
    void PivotArmSetRotation(double power, double degrees, boolean unlatch, boolean asynkk)
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
        if (unlatch) {
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

            robot.DrivePowerAll(-.1);

        }
        if (asynkk)
            return;
        for (int i=0; i < 2; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.pivotArm1.isBusy() && robot.pivotArm2.isBusy()) {
                dashboard.displayPrintf(10, "The ENEMY gates are down!");
            }
            sleep(10);
        }
        //robot.pivotArm1.setPower(0);
        //robot.pivotArm2.setPower(0);
        robot.DrivePowerAll(0);
    }

    void PivotArmSetRotation (double power, double degrees)
    {
        PivotArmSetRotation(power, degrees, false, false);
    }

    /**
     * ExtendoArm_ACTIVATE ACTIVATES the ExtendoArm
     * Positive extends
     * @param power power of extension
     * @param inches "inches" of extension (not accurate)
     * @param asynkk allows for other functions to be run simultaneously
     */
    void ExtendoArm5000_ACTIVATE(double power, double inches, boolean asynkk)
    {
        int position = (int)(inches*EXTENDOARM_CONSTANT);
        robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extendoArm5000.setPower(power);
        robot.extendoArm5000.setTargetPosition(position);

        if (asynkk)
            return;

        while (robot.extendoArm5000.isBusy())
            dashboard.displayPrintf(10, "Encoder position: %d", robot.extendoArm5000.getCurrentPosition());
        robot.extendoArm5000.setPower(0);
    }

    /**
     * ExtendoArm_ACTIVATE_TIME ACTIVATES the ExtendoArm
     * Positive extends
     * @param power power of extension
     * @param time time of extension
     */
    void ExtendoArm5000_ACTIVATE_TIME (double power, int time)
    {
        robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.extendoArm5000.setPower(power);
        sleep(time);
        robot.extendoArm5000.setPower(0);
    }

    /**
     * Determines mineral with TensorFlow
     */
    Gold getSampleFromTensorFlow () {
        Gold thisgold = Gold.UNKNOWN;
        // Sample minerals with TensorFlow
        if (!isStopRequested()) {
            int i = 0;
            ElapsedTime holdTimer = new ElapsedTime();
            while (!opModeIsActive() && !isStopRequested() && (holdTimer.time() < 3)) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        dashboard.displayPrintf(1,"%d objects detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 1) {
                            double    left = Math.random()/2;
                            double    right = Math.random()/2;
                            double    center = Math.random()/2;
                            int       vote = 0;
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
                                thisgold = Gold.LEFT;
                                if (left > .5) {
                                    dashboard.displayPrintf(2, "Left");
                                    break;
                                }
                            }
                            else if (center > right) {
                                thisgold = Gold.CENTER;
                                if (center > .5) {
                                    dashboard.displayPrintf(2, "Center");
                                    break;
                                }
                            }
                            else {
                                thisgold = Gold.RIGHT;
                                if (right > .5) {
                                    dashboard.displayPrintf(2, "Right");
                                    break;
                                }
                            }
                        }
                    }
                }
                i++;
                dashboard.displayPrintf(2, "TensorFlow iteration count: %d", i);
            }
        }

        // If hanging, invert right/left because camera is upside-down
        if (hanging == Lift.YES) {
            if (thisgold== Gold.RIGHT)
                thisgold= Gold.LEFT;
            else if (thisgold== Gold.LEFT)
                thisgold= Gold.RIGHT;
        }
        return thisgold;
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
        FtcChoiceMenu<ExtraScore> extraScoreMenu = new FtcChoiceMenu<>("Attempt to score mineral?", craterMenu, this);

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
        craterMenu.addChoice("None", Crater.NONE, false);

        extraScoreMenu.addChoice("Yes", ExtraScore.YES, true);
        extraScoreMenu.addChoice("No", ExtraScore.NO, false);

        FtcMenu.walkMenuTree(modeMenu, this);
        runmode = modeMenu.getCurrentChoiceObject();
        delay = (int) delayMenu.getCurrentValue();
        hanging = liftMenu.getCurrentChoiceObject();
        startposition = startPositionMenu.getCurrentChoiceObject();
        depot = depotMenu.getCurrentChoiceObject();
        sampling = samplingMenu.getCurrentChoiceObject();
        crater = craterMenu.getCurrentChoiceObject();
        attemptExtraScore = extraScoreMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
