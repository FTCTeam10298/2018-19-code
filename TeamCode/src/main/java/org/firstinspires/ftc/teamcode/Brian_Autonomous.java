/* Copyright (c) 2016-19 Brain Stormz. All rights reserved.
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

@Autonomous(name="Brian Autonomous", group ="Brian")
public class Brian_Autonomous extends LinearOpMode implements FtcMenu.MenuButtons {
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
        NEAR_DEFENSE,
        FAR_DEFENSE,
        NONE
    }

    public enum Depot {
        YES,
        NO
    }

    public enum ExtraScore {
        DELIVER_ALL_THE_THINGS(2),
        YES(1),
        NO(0);

        private int numVal;

        ExtraScore(int numVal) {
            this.numVal = numVal;
        }

        public int getNumVal() {
            return numVal;
        }
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
    Brian_Hardware robot = new Brian_Hardware();

    static final double COUNTS_PER_MOTOR_REV  = 28.0;      // Rev HD Hex v2.1 Motor encoder
    static final double GEARBOX_RATIO         = 20.0;      // 40 for 40:1, 20 for 20:1
    static final double DRIVE_GEAR_REDUCTION  = 24.0/15.0; // This is > 1.0 if geared for torque
    static final double WHEEL_DIAMETER_INCHES = 3.937007874015748; // For figuring circumference
    static final double DRIVETRAIN_ERROR      = 1.04;      // Error determined from testing
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * GEARBOX_RATIO * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI) / DRIVETRAIN_ERROR;
    static final double COUNTS_PER_DEGREE     = (COUNTS_PER_INCH*0.20672)+0.003703704; // Was 0.20672; // Found by testing

    static final double PIVOTARM_CONSTANT     = (1440.0 * 10.0 / 360.0)*(0.6667); // Constant that converts pivot arm to degrees (1440*10/360 for Torquenado)
//    static final double EXTENDOARM_CONSTANT   = /*encoder counts per out shaft revolution*/ 1120.0 * (/*sprocket ratio*/ 30.0 / 15.0) / (3.0 * Math.PI); // Constant that converts ExtendoArm to inches 1120 * 2/(3 * 3.14159265)
//    static final double EXTENDOARM_CONSTANT   = 1120.0 * 2.0 * 15.0 / 25.0 / (3. * Math.PI) * (30.0 / 26.0) * 1.1; // Constant that converts ExtendoArm to "inches" (not really, but don't have time to re-adjust the whole auto)
    static final double EXTENDOARM_CONSTANT   = 1120.0 * 1.0 * 15.0 / 25.0 / (3. * Math.PI) * (15.0 / 10.0) * 1.1; // Constant that converts ExtendoArm to "inches" (not really, but don't have time to re-adjust the whole auto)

    ModernRoboticsI2cRangeSensor rangeSensor;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "Aez7J07/////AAABmS8UUfYgtke9gtnv8lpOtUwWoahEf1DgQVHSXzYz9B1RjnN9agKHZOutrQmzOMd57S7KpZKOt6yvLF4rGqzWR4Re/EhtUIe1+MD9DYlEkHHX3bio1F0kblG0BgzIAxmM+u+2L10gHO4pyuUnPohg7/T5mY912NuZGocuhq65i20+xV1b3bjStwuZaKY14lXvEklvO8ZcFvR26928fxmJIVWtRqashdFZ5nxm1w/sqJ9eWJQv3mLZt7AVrclS5NwPxzSwlX6+8IK8dWIOOJuZx0mKVlQMNoAEXioCuIuuFwAMBtm+NMkjXTxtfiShYXnfD7e1pAUsMgksQBGl1cCqcpLr8dD1msx9nOlI6fsfMY9e";
    int errors = 0;

    /**
     * `vuforia` is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * `tfod` is the variable we will use to store our instance of the Tensor Flow Object
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
            errors += 1;
        }

        if (startposition == StartPosition.GOLD && sampling == Sampling.TWO) {
            dashboard.displayPrintf(1, "You picked the wrong option, Stephanie!");
            sampling = Sampling.ONE;
            errors += 1;
        }

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            dashboard.displayPrintf(1,"THE ZTE SPEEEEED IS TOO FAST FOR TFOD");
            errors += 1;
        }

        robot.driveSetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.driveSetTargetPosition(0, 0, 0, 0);
        robot.pivotArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.pivotArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pivotArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.pivotArm1.setTargetPosition(0);
        robot.pivotArm2.setTargetPosition(0);
        robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.extendoArm5000.setTargetPosition(0);

        // Activate TensorFlow Object Detection.
        if (tfod != null) {
            tfod.activate();
        } else {
            dashboard.displayPrintf(1, "Failed to activate TFOD");
        }

        dashboard.displayPrintf(0, "Status: Ready to start");
        if (errors > 0)
            dashboard.displayPrintf(2, "!!! %d errors!", errors);

        // Sample while waiting for start
        while (!opModeIsActive() && !isStopRequested()) {
            Gold sample = getSampleFromTensorFlow();
            if (sample != Gold.UNKNOWN) {
                gold = sample;
            }
        }

        // Shutdown TensorFlow
        if (tfod != null) {
            tfod.shutdown();
        }

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        dashboard.displayPrintf(0, "Status: Running");

        if (DoTask("Motor test", runmode, false)) {
            // Motor test (Uncomment out to test drive train encoders)
            robot.driveSetMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.DrivePowerAll(0.5);

            robot.backRightDrive.setTargetPosition(5000);
            while (robot.backRightDrive.isBusy()) {
                sleep(10);
            }

            robot.backLeftDrive.setTargetPosition(5000);
            while (robot.backLeftDrive.isBusy()) {
                sleep(10);
            }

            robot.frontRightDrive.setTargetPosition(5000);
            while (robot.frontRightDrive.isBusy()) {
                sleep(10);
            }

            robot.frontLeftDrive.setTargetPosition(5000);
            while (robot.frontLeftDrive.isBusy()) {
                sleep(10);
            }

            // test spin
            DriveRobotTurn(.3, 360);
            sleep(1000);
            DriveRobotTurn(.3, -360);

            DriveRobotHug(1, 48, false);
        }

        if (DoTask("Arm Test", runmode, false)) {
            for (int i = 0; i < 8; i++) {
                PivotArmSetRotation(1, 90, false, false);
                PivotArmSetRotation(1, -90, false, false);
            }
        }

        if (DoTask("Spin test - clockwise", runmode, false)) {
            DriveRobotTurn(.2, 360*15);
        }

        if (DoTask("Spin test - counter-clockwise", runmode, false)) {
            DriveRobotTurn(.2, -360*15);
        }

        if (DoTask("Drive test (72 inches)", runmode, false)) {
            DriveRobotPosition(.2, 72, false);
        }

        // Pause the program for the selected delay period
        sleep(delay);

        // Init
        if (DoTask("Init", runmode, true)) {
            if (hanging == Lift.YES) {
                ExtendoArm5000_ACTIVATE_abs(1,4, false);
                ExtendoArm5000_ACTIVATE_abs(1,15, true);
                PivotArmSetRotation(1, -15, false, false);
                PivotArmSetRotation(.5, 95, true, false);
                DriveRobotPosition(.7, 8, true);
                if (startposition == StartPosition.GOLD && depot == Depot.YES) {
                    PivotArmSetRotation(1, -65, false, true);
                } else {
                    PivotArmSetRotation(1, -85, false, false);
                }
            } else {
                DriveRobotPosition(.6, 8, true); // FIXME: IS THIS RIGHT?
            }
        }

        if (startposition == StartPosition.GOLD && depot == Depot.YES && DoTask("Deposit team marker (gold)", runmode, true)) {
            ExtendoArm5000_ACTIVATE_abs(1, 29, true);
            DriveRobotPosition(0.4, 13, true);
            ExtendoArm5000_ACTIVATE_abs(1, 29, false); // Wait for extension to finish
            PivotArmSetRotation(1, -10, false, true);

            robot.collectOtron.setPower(.4);
            sleep(1000);
            robot.collectOtron.setPower(0);

            PivotArmSetRotation(1, 10, false, true);
            ExtendoArm5000_ACTIVATE_abs(1, 15, true);
            DriveRobotPosition(0.4, -13, true);
            ExtendoArm5000_ACTIVATE_abs(1, 15, false); // Wait for retraction to finish
            PivotArmSetRotation(1, -22, false, true);
        }

        if ((sampling == Sampling.ONE || sampling == Sampling.TWO) && DoTask("Mineral Sampling", runmode, true))
            DriveSample();

        if (DoTask("Drive my Car", runmode, true)) {
            if (startposition == StartPosition.GOLD) {
                if (sampling == Sampling.ONE && attemptExtraScore == ExtraScore.YES) {
                    PivotArmSetRotation(1, 95, false, true);
                    ExtendoArm5000_ACTIVATE_abs(1, 26, false);
                    robot.collectOtron.setPower(0);
                    PivotArmSetRotation(1, 0, false, false);
                    robot.collectorGate.setPosition(.25);
                    PivotArmSetRotation(0.5, -5, false, true);
                    sleep(300);
                    PivotArmSetRotation(0.5, 5, false, true);
                    sleep(300);
                    PivotArmSetRotation(1, -60, false, true);
                    sleep(400);
                    robot.collectorGate.setPosition(.65);
                    ExtendoArm5000_ACTIVATE_abs(1, 9, true);
                } else {
                    PivotArmSetRotation(1, 35, false, true);
                    robot.collectOtron.setPower(0);
                }

                if (crater == Crater.FAR_DEFENSE || crater == Crater.NEAR_DEFENSE) {
                    ExtendoArm5000_ACTIVATE_abs(1, 9, true);
                    sleep(1000);
                    PivotArmSetRotation(1, -55, false, false);
                }
                else {
                    DriveRobotPosition(.6, 8, true);
                }

                if (crater == Crater.FAR) {
                    DriveRobotTurn(.5, -70);
                    DriveRobotPosition(.8, 40, true);
                    DriveRobotTurn(1, -45, false);
                    ExtendoArm5000_ACTIVATE_abs(1, 19, true);
                    DriveRobotPosition(1, 30, false);
                }
                else if (crater == Crater.NEAR) {
                    DriveRobotTurn(.5, 87);
                    DriveRobotPosition(.8, 48, true);
                    DriveRobotTurn(1, 45, false);
                    ExtendoArm5000_ACTIVATE_abs(1, 19, true);
                    DriveRobotPosition(1, 11, false);
                }
                else if (crater == Crater.NEAR_DEFENSE) {
                    DriveRobotPosition(1, -2, true);
                    DriveRobotTurn(1, -42, true);
                    DriveRobotPosition(1, -40, true);
                    DriveRobotTurn(1, 30, true);
                    DriveRobotPosition(1, -34, true);
                    PivotArmSetRotation(1, 30, false, true);
                    sleep(200);
                    DriveRobotTurn(1, 70, true);
                    //DriveRobotPosition(1, 4, true);
                    ExtendoArm5000_ACTIVATE_abs(1, 19, false);
                    sleep(10000);
                }
                else if (crater == Crater.FAR_DEFENSE) {
                    DriveRobotPosition(1, -2, true);
                    DriveRobotTurn(1, 45, true);
                    DriveRobotPosition(1, -40, true);
                    DriveRobotTurn(1, -30, true);
                    DriveRobotPosition(1, -34, true);
                    PivotArmSetRotation(1, 30, false, true);
                    sleep(200);
                    DriveRobotTurn(1, -70, true);
                    //DriveRobotPosition(1, 4, true);
                    ExtendoArm5000_ACTIVATE_abs(1, 19, false);
                    sleep(10000);

                }
            } else if (startposition == StartPosition.SILVER) {
                if (sampling == Sampling.ONE && attemptExtraScore.getNumVal() >= 1) {
                    PivotArmSetRotation(1, 110, false, true);
                    ExtendoArm5000_ACTIVATE_abs(1, 26, true);
                    DriveRobotTurn(1, 25, true);
                    robot.collectOtron.setPower(0);
                    ExtendoArm5000_ACTIVATE_abs(1, 26, false); // Wait for extension to finish
                    PivotArmSetRotation(1, 0, false, false); // Wait for pivot to finish
                    sleep(200);
                    robot.collectorGate.setPosition(.25);
                    sleep(1200);
//                    robot.collectorGate.setPosition(.45);
//                    sleep(200);
//                    robot.collectorGate.setPosition(.25);
//                    sleep(900);
                    robot.collectorGate.setPosition(.65);

                    if (attemptExtraScore == ExtraScore.DELIVER_ALL_THE_THINGS && depot == Depot.NO) {
                        for (int i=0; i<2; i++) {
                            PivotArmSetRotation(1, -110, false, true);
                            robot.collectOtron.setPower(1);
                            DriveRobotPosition(.5, 10, true);
                            PivotArmSetRotation(1, 0, false, false);

                            sleep(500);

                            PivotArmSetRotation(1, 45, false, false);
                            robot.collectOtron.setPower(-1);
                            sleep(500);
                            DriveRobotPosition(.5, -10, true);
                            PivotArmSetRotation(1, 65, false, false);

                            sleep(200);
                            robot.collectorGate.setPosition(.25);
                            sleep(300);
                            robot.collectorGate.setPosition(.45);
                            sleep(200);
                            robot.collectorGate.setPosition(.25);
                            sleep(900);
                        }
                    }
                    else {
                        PivotArmSetRotation(1, -65, false, true);
                        sleep(400);
                        ExtendoArm5000_ACTIVATE_abs(1, 5, true);
                        DriveRobotPosition(.4, 3, true);
                        DriveRobotTurn(1, -25, true);
                    }
                } else {
                    PivotArmSetRotation(1, 50, false, true);
                    robot.collectOtron.setPower(0);
                }

                if (depot == Depot.NO) {
                    return;
                }

                DriveRobotPosition(.5, 9.5, true);
                DriveRobotTurn(.6, -90, true);
                // Drive to wall then to depot
                if (sampling == Sampling.ONE && attemptExtraScore == ExtraScore.YES)
                    DriveRobotPosition(.7, 52, true);
                else
                    DriveRobotPosition(.7, 50, true);
                DriveRobotTurn(.5, -35);
                //DriveSidewaysTime(.5, 1);
                DriveRobotHug(1, 38, false);

                // Deposit team marker
                DriveRobotTurn(1, -5);
                robot.markerDumper.setPosition(0.7);
                if (sampling == Sampling.ONE && attemptExtraScore == ExtraScore.NO) {
                    robot.collectOtron.setPower(-.7);
                    sleep(1000);
                    robot.collectOtron.setPower(0);
                } else {
                    sleep(600);
                }

                if (sampling == Sampling.ONE && crater == Crater.NEAR) {
                    DriveRobotPosition(1, -40, false);
                    ExtendoArm5000_ACTIVATE_abs(1, 20, true);
                    DriveRobotTurn(.7, -180);
                    DriveSidewaysTime(.5, 1);
                    DriveRobotPosition(1, 20, true);
                }
                else if (sampling == Sampling.TWO) {
                    // Drive back varying amounts depending on sample
                    if (gold == Gold.LEFT)
                        DriveRobotDistanceToObject(1, 14, true);
                    else if (gold == Gold.CENTER)
                        DriveRobotDistanceToObject(1, 26, true);
                    else
                        DriveRobotDistanceToObject(1, 36, true);

                    DriveRobotTurn(1, -90, true);

                    // Extend arm and collect gold mineral
                    robot.collectOtron.setPower(1);
                    if (gold == Gold.LEFT) {
                        PivotArmSetRotation(1, -48, false, true);
                        DriveRobotPosition(.7, 13, true);
                        ExtendoArm5000_ACTIVATE(1, 12, false);
                        ExtendoArm5000_ACTIVATE(1, -12, true);
                        DriveRobotPosition(.7, -13, true);
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
                    robot.collectOtron.setPower(0);

                    DriveRobotDistanceToObject(1, 60, false);

//                    DriveRobotTurn(1, 190); // Spin around so we don't have to in TeleOp
//                    DriveSidewaysTime(0.5, 1); // Strafe left
//                    ExtendoArm5000_ACTIVATE(1, 10, true);
//                    DriveRobotPosition(1, 36, false);

                    //DriveSidewaysTime(0.5, -1); // Strafe right
                    DriveRobotTurn(1, -8, false);
                    DriveRobotPosition(1, -36, false);
                }
            }
        }
    }

    /**
     * FUNCTIONS -----------------------------------------------------------------------------------
     */

    boolean DoTask(String taskname, RunMode debug, boolean default_value)
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
        return default_value;
    }

    /**
     * DriveRobotTime drives the robot the set number of inches at the given power level.
     * @param ms How long to drive
     * @param power Power level to set motors to, negative will drive the robot backwards
     */
    void DriveRobotTime(int ms, double power)
    {
        robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.DrivePowerAll(power);
        sleep(ms);
        robot.DrivePowerAll(0);
        robot.driveSetTargetPosition(0, 0, 0, 0);
    }

    /**
     * DriveRobotDistanceToObject drives the robot to the set number of inches from an object
     * (usually the wall) at the given power level.
     * @param inches How many inches away to the object to go to
     * @param power Power level to set motors to
     */
    void DriveRobotDistanceToObject(double power, double inches, boolean smart_accel)
    {
        double target = (float)rangeSensor.getDistance(DistanceUnit.INCH) - inches; // FIXME: how accurate is sensor?
        dashboard.displayPrintf(10, "Range Sensor: ", rangeSensor.getDistance(DistanceUnit.INCH));
        DriveRobotPosition(abs(power), target, smart_accel); // Use abs() to make sure power is positive
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

        robot.driveSetRunToPosition();

        if (smart_accel && power > 0.25)
        {
            robot.DrivePowerAll(0.25); // Use abs() to make sure power is positive
            state = 1; // ACCEL
        }
        else {
            robot.DrivePowerAll(abs(power)); // Use abs() to make sure power is positive
        }

        int flOrigTarget = robot.frontLeftDrive.getTargetPosition();
        int frOrigTarget = robot.frontRightDrive.getTargetPosition();
        int blOrigTarget = robot.backLeftDrive.getTargetPosition();
        int brOrigTarget = robot.backRightDrive.getTargetPosition();
        robot.driveAddTargetPosition((int)position, (int)position, (int)position, (int)position);

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
                        (abs(flDrive-flOrigTarget) > 2*COUNTS_PER_INCH ||
                         abs(frDrive-frOrigTarget) > 2*COUNTS_PER_INCH ||
                         abs(blDrive-blOrigTarget) > 2*COUNTS_PER_INCH ||
                         abs(brDrive-brOrigTarget) > 2*COUNTS_PER_INCH )) {
                    // We have gone 2 inches, go to full power
                    robot.DrivePowerAll(abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (abs(flDrive-flOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(frDrive-frOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(blDrive-blOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) ||
                         abs(brDrive-brOrigTarget) > COUNTS_PER_INCH*(abs(inches)-2) )) {
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

    void DriveRobotTurn (double power, double degree, boolean smart_accel)
    {
        double position = degree*COUNTS_PER_DEGREE;

        int state = 0; // 0 = NONE, 1 = ACCEL, 2 = DRIVE, 3 = DECEL

        robot.driveSetRunToPosition();

        if (smart_accel) {
            state = 1;
            robot.driveSetPower(power*0.5, -power*0.5, power*0.5, -power*0.5);
        }
        else
        {
            robot.driveSetPower(power, -power, power, -power);
        }

        int flOrigTarget = robot.frontLeftDrive.getTargetPosition();
        int frOrigTarget = robot.frontRightDrive.getTargetPosition();
        int blOrigTarget = robot.backLeftDrive.getTargetPosition();
        int brOrigTarget = robot.backRightDrive.getTargetPosition();
        robot.driveAddTargetPosition((int)position, -(int)position, (int)position, -(int)position);

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
                        (abs(flDrive-flOrigTarget) > COUNTS_PER_DEGREE*10 ||
                         abs(frDrive-frOrigTarget) > COUNTS_PER_DEGREE*10 ||
                         abs(blDrive-blOrigTarget) > COUNTS_PER_DEGREE*10 ||
                         abs(brDrive-brOrigTarget) > COUNTS_PER_DEGREE*10 )) {
                    // We have rotated 10 degrees, go to full power
                    robot.DrivePowerAll(abs(power)); // Use abs() to make sure power is positive
                    state = 2;
                }
                else if (state == 2 &&
                        (abs(flDrive-flOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(frDrive-frOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(blDrive-blOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) ||
                         abs(brDrive-brOrigTarget) > COUNTS_PER_DEGREE*(abs(degree)-10) )) {
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

        robot.driveSetRunToPosition();
        robot.driveSetTargetPosition(0, 0, 0, 0);
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

        if (gold == Gold.LEFT)
            DriveRobotTurn(.6, -33, true);
        else if (gold == Gold.RIGHT)
            DriveRobotTurn(.6, 30, true);

        PivotArmSetRotation(.5, -10, false, true);
        DriveRobotPosition(.7, 12, true);
        ExtendoArm5000_ACTIVATE_abs(1,15, false);

        if (startposition == StartPosition.SILVER && depot == Depot.NO && crater == Crater.NEAR && attemptExtraScore == ExtraScore.NO) {
            DriveRobotPosition(1, 5, false);
            sleep(30000); // Program done
            return;
        }

        if (attemptExtraScore == ExtraScore.DELIVER_ALL_THE_THINGS) {
            PivotArmSetRotation(1, 10, false, true);
            ExtendoArm5000_ACTIVATE_abs(1, 20, false);
            PivotArmSetRotation(1, -10, false, false);
            sleep(500);
            PivotArmSetRotation(1, 30, false, false);
            robot.collectOtron.setPower(-1);
            sleep(500);
        }
        else {
            ExtendoArm5000_ACTIVATE_abs(1, 26, true);
            PivotArmSetRotation(1, 25, false, true);
        }

        if (gold != Gold.CENTER)
            DriveRobotPosition(.6, -12, true);
        else
            DriveRobotPosition(.6, -16, true);

        if (gold == Gold.LEFT)
            DriveRobotTurn(.6, 33, true);
        else if (gold == Gold.RIGHT)
            DriveRobotTurn(.6, -30, true);

        if (gold != Gold.CENTER)
            DriveRobotPosition(.6, -4, true);

        if (attemptExtraScore == ExtraScore.DELIVER_ALL_THE_THINGS) {
            // Functions assume that the robot will be in the same position after DriveSample()
            // as before, so set rotation to -10 degrees with 0 power, so that the next arm moves
            // (e.g. go up to lander) it will go to the same position as if the arm wasn't moved up.
            PivotArmSetRotation(0, -210, false, true);
        }

    }

    /**
     * PivotArmSetRotation
     * Positive swings up/back
     * @param power Power level
     * @param degrees Degrees of rotation
     * @param unlatch Drives wheels backwards for unlatching purposes
     * @param asynkk Allows for another function to take place simultaneously
     */
    void PivotArmSetRotation(double power, double degrees, boolean unlatch, boolean asynkk)
    {
        int position = (int)(degrees* PIVOTARM_CONSTANT);
        if (robot.pivotArm1.getMode() != DcMotor.RunMode.RUN_TO_POSITION ||
            robot.pivotArm2.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            robot.pivotArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivotArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.pivotArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.pivotArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.pivotArm1.setPower(power);
        robot.pivotArm2.setPower(power);
        robot.pivotArm1.setTargetPosition(robot.pivotArm1.getTargetPosition()+position);
        robot.pivotArm2.setTargetPosition(robot.pivotArm2.getTargetPosition()+position);

        if (unlatch) {
            robot.driveSetMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.DrivePowerAll(-.1);
        }

        if (asynkk)
            return;

        sleep(150);

        for (int i=0; i < 2; i++) {    // Repeat check 3 times, sleeping 10ms between,
                                       // as isBusy can be a bit unreliable
            while (robot.pivotArm1.isBusy() && robot.pivotArm2.isBusy()) {
                dashboard.displayPrintf(10, "pivotArm1: %d target, %d current", robot.pivotArm1.getTargetPosition(), robot.pivotArm1.getCurrentPosition());
                dashboard.displayPrintf(11, "pivotArm2: %d target, %d current", robot.pivotArm2.getTargetPosition(), robot.pivotArm2.getCurrentPosition());
                if ((robot.pivotArm1.getVelocity() < 10 && robot.pivotArm1.getVelocity() > -10) &&
                        (robot.pivotArm2.getVelocity() < 10 && robot.pivotArm2.getVelocity() > -10) && power != 0) {
                    dashboard.displayPrintf(12, "Warning: Failsafe invoked on pivotArmSetRotation()");
                    break;
                }
            }
            sleep(10);
        }

        //robot.pivotArm1.setPower(0);
        //robot.pivotArm2.setPower(0);
        if (unlatch)
            robot.DrivePowerAll(0);
    }

    /**
     * ExtendoArm_ACTIVATE ACTIVATES the ExtendoArm
     * Positive extends
     * @param power power of extension
     * @param inches "inches" of extension (not actual inches)
     * @param asynkk allows for other functions to be run simultaneously
     */
    void ExtendoArm5000_ACTIVATE(double power, double inches, boolean asynkk)
    {
        int position = (int)(inches*EXTENDOARM_CONSTANT);
        if (robot.extendoArm5000.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.extendoArm5000.setTargetPosition(0);
        }
        robot.extendoArm5000.setPower(power);
        robot.extendoArm5000.setTargetPosition(robot.extendoArm5000.getTargetPosition() + position);

        if (asynkk)
            return;

        sleep(250);

        while (robot.extendoArm5000.isBusy()) {
            dashboard.displayPrintf(10, "ExtendoArm: %d target, %d current", robot.extendoArm5000.getTargetPosition(), robot.extendoArm5000.getCurrentPosition());
            if ((robot.extendoArm5000.getVelocity() < 10 && robot.extendoArm5000.getVelocity() > -10) && power != 0) {
                dashboard.displayPrintf(12, "Warning: Failsafe invoked on ExtendoArm5000_ACTIVATE()");
                break;
            }
        }
    }

    /**
     * ExtendoArm_ACTIVATE_abs ACTIVATES the ExtendoArm, using an absolute position (as opposed to
     * a position relative to the current target position).
     * Positive extends
     * @param power power of extension
     * @param target_inches target "inches" to extend to (not actual inches)
     * @param asynkk allows for other functions to be run simultaneously
     */
    void ExtendoArm5000_ACTIVATE_abs(double power, double target_inches, boolean asynkk)
    {
        if (robot.extendoArm5000.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            robot.extendoArm5000.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.extendoArm5000.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        robot.extendoArm5000.setPower(power);
        robot.extendoArm5000.setTargetPosition((int)(target_inches*EXTENDOARM_CONSTANT));

        if (asynkk)
            return;

        sleep(250);

        while (robot.extendoArm5000.isBusy()) {
            dashboard.displayPrintf(10, "ExtendoArm: %d target, %d current", robot.extendoArm5000.getTargetPosition(), robot.extendoArm5000.getCurrentPosition());
            if ((robot.extendoArm5000.getVelocity() < 10 && robot.extendoArm5000.getVelocity() > -10) && power != 0) {
                dashboard.displayPrintf(12, "Warning: Failsafe invoked on ExtendoArm5000_ACTIVATE()");
                break;
            }
        }
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
                int j = 0;
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        dashboard.displayPrintf(1,"%d objects detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 1) {
                            double    left   = 0;
                            double    right  = 0;
                            double    center = 0;
                            double    vote   = 0;
                            int       gold   = 0;   // How many gold minerals are detected
                            int       silver = 0; // How many silver minerals are detected
                            final int WIDTH  = updatedRecognitions.get(0).getImageWidth();

                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    vote+=recognition.getConfidence();
                                    gold++;
                                } else {
                                    vote-=recognition.getConfidence();
                                    if (recognition.getLabel().equals(LABEL_SILVER_MINERAL))
                                        silver++;
                                }
                                if ((int) recognition.getLeft() < WIDTH/3)
                                    right+=vote;
                                else if ((int) recognition.getLeft() < 2*WIDTH/3)
                                    center+=vote;
                                else
                                    left+=vote;
                                vote=0;
                            }
                            dashboard.displayPrintf(1,"%d objects detected (%d gold, %d silver)", updatedRecognitions.size(), gold, silver);
                            for (j = 0; j < updatedRecognitions.size(); j++) {
                                if (updatedRecognitions.get(j).getLabel().equals(LABEL_GOLD_MINERAL))
                                    dashboard.displayPrintf(j+3, "Gold (x = %f, y = %f)",
                                            (updatedRecognitions.get(j).getLeft()+updatedRecognitions.get(j).getRight())/2,
                                            (updatedRecognitions.get(j).getLeft()+updatedRecognitions.get(j).getRight())/2);
                                else
                                    dashboard.displayPrintf(j+3, "Silver (x = %f, y = %f)",
                                            (updatedRecognitions.get(j).getLeft()+updatedRecognitions.get(j).getRight())/2,
                                            (updatedRecognitions.get(j).getLeft()+updatedRecognitions.get(j).getRight())/2);
                            }
                            if (left > center && left > right) {
                                thisgold = Gold.LEFT;
                                if (left > .5) {
                                    dashboard.displayPrintf(2, "Gold: Left");
                                }
                            }
                            else if (center > right) {
                                thisgold = Gold.CENTER;
                                if (center > .5) {
                                    dashboard.displayPrintf(2, "Gold: Center");
                                }
                            }
                            else {
                                thisgold = Gold.RIGHT;
                                if (right > .5) {
                                    dashboard.displayPrintf(2, "Gold: Right");
                                }
                            }
                        }
                    }
                }
                i++;
                dashboard.displayPrintf(10, "TensorFlow iteration count: %d", i);
            }
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

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
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
        FtcChoiceMenu<ExtraScore> extraScoreMenu = new FtcChoiceMenu<>("Attempt to score mineral:", craterMenu, this);

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

        craterMenu.addChoice("Near", Crater.NEAR, true, extraScoreMenu);
        craterMenu.addChoice("Far", Crater.FAR, false, extraScoreMenu);
        craterMenu.addChoice("Near Defense", Crater.NEAR_DEFENSE, false, extraScoreMenu);
        craterMenu.addChoice("Far Defense", Crater.FAR_DEFENSE, false, extraScoreMenu);
        craterMenu.addChoice("None", Crater.NONE, false, extraScoreMenu);

        extraScoreMenu.addChoice("Deliver ALL the things!", ExtraScore. DELIVER_ALL_THE_THINGS, false);
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

        dashboard.displayPrintf(13, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(14, "Delay = %d msec", delay);
        dashboard.displayPrintf(15, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString());
        dashboard.displayPrintf(16, "Hanging: %s (%s)", liftMenu.getCurrentChoiceText(), hanging.toString());
        dashboard.displayPrintf(17, "Number of samples: %s (%s)", samplingMenu.getCurrentChoiceText(), sampling.toString());
        dashboard.displayPrintf(18, "Go for depot: %s (%s)", depotMenu.getCurrentChoiceText(), depot.toString());
        dashboard.displayPrintf(19, "Crater: %s (%s)", craterMenu.getCurrentChoiceText(), crater.toString());
        dashboard.displayPrintf(20, "Attempt score: %s (%s)", extraScoreMenu.getCurrentChoiceText(), attemptExtraScore.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
