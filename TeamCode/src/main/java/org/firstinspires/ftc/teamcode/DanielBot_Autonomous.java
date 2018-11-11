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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import ftclib.FtcChoiceMenu;
import ftclib.FtcMenu;
import ftclib.FtcValueMenu;
import hallib.HalDashboard;

@Autonomous(name="Daniel Autonomous", group ="DanielBot")
public class DanielBot_Autonomous extends LinearOpMode implements FtcMenu.MenuButtons {
    public enum StartPosition {
        SILVER,
        GOLD
    }
    public enum RunMode {
        RUNMODE_AUTO,
        RUNMODE_DEBUG
    }
    public enum Sampling {
        ZERO,
        ONE,
        TWO
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
    RunMode        runmode       = RunMode.RUNMODE_AUTO;
    int            delay         = 0;
    StartPosition  startposition = StartPosition.SILVER;
    Crater         crater        = Crater.NEAR;
    Depot          depot         = Depot.YES;
    Sampling       sampling      = Sampling.ZERO;

    /* Declare OpMode members. */
    private HalDashboard dashboard;
    DanielBot_Hardware   robot   = new DanielBot_Hardware();

    static final double  COUNTS_PER_MOTOR_REV      = 1120;    // Rev HD Hex v2.1 Motor encoder
    static final double  DRIVE_GEAR_REDUCTION      = .825;       // This is < 1.0 if geared for torque
    static final double  WHEEL_DIAMETER_INCHES     = 4.0;     // For figuring circumference
    static final double  COUNTS_PER_INCH           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265);

    static final double  LIFTONATOR_CONSTANT        = 280/9; //constant that converts liftonator to degrees (1120*10/360)
    static final double  EXTENDOARM_CONSTANT        = 4480/(13 * 3.14159265); //constant that converts ExtendoArm to inches 1120/(3.25 * 3.14159265)
    /**
     * Define the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * Color sensor declaration
     */

    @Override public void runOpMode() {
        // Initialize the hardware -----------------------------------------------------------------
        robot.init(hardwareMap);
        // Initialize dashboard --------------------------------------------------------------------
        dashboard = HalDashboard.createInstance(telemetry);

        // Initialize Vuforia ----------------------------------------------------------------------
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor
         * (on the RC phone); If no camera monitor is desired, use the parameterless
         * constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId",
                                                    "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // Or, don't activate the Camera Monitor view, to save power
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // Our key
        parameters.vuforiaLicenseKey = "Aez7J07/////AAABmS8UUfYgtke9gtnv8lpOtUwWoahEf1DgQVHSXzYz9B1RjnN9agKHZOutrQmzOMd57S7KpZKOt6yvLF4rGqzWR4Re/EhtUIe1+MD9DYlEkHHX3bio1F0kblG0BgzIAxmM+u+2L10gHO4pyuUnPohg7/T5mY912NuZGocuhq65i20+xV1b3bjStwuZaKY14lXvEklvO8ZcFvR26928fxmJIVWtRqashdFZ5nxm1w/sqJ9eWJQv3mLZt7AVrclS5NwPxzSwlX6+8IK8dWIOOJuZx0mKVlQMNoAEXioCuIuuFwAMBtm+NMkjXTxtfiShYXnfD7e1pAUsMgksQBGl1cCqcpLr8dD1msx9nOlI6fsfMY9e";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // Run though the menu ---------------------------------------------------------------------
        doMenus();
        dashboard.displayPrintf(0, "Status: Ready to start");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
         * ----- AUTONOMOUS START-------------------------------------------------------------------
         */

        // Pause the program for the selected delay period
        sleep(delay);


        if (DoTask("Init", runmode)) {
            robot.pivotLock.setPosition(0);
            sleep(100);
            SlavedLift(1, 90);
            robot.extensionLock.setPosition(1);
            sleep(100);
            ExtendoArm5000_ACTIVATE(.5, 6);
        }

        if (DoTask("Mineral Sampling", runmode)) {

        }

        if (DoTask("Drive my Car", runmode)) {

        }

        // drive
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
     * SlavedLift
     * Positive swings back
     * @param power Power level
     * @param degrees Degrees of rotation
     */
    void SlavedLift (double power, double degrees)
    {
        int position = (int)(degrees*LIFTONATOR_CONSTANT);
        robot.liftinator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftinator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftinator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftinator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftinator1.setPower(power);
        robot.liftinator2.setPower(power);
        robot.liftinator1.setTargetPosition(position);
        robot.liftinator2.setTargetPosition(position);
        for (int i=0; i < 5; i++) {    // Repeat check 5 times, sleeping 10ms between,
            // as isBusy can be a bit unreliable
            while (robot.liftinator1.isBusy() && robot.liftinator2.isBusy()) {
                dashboard.displayPrintf(10, "The ENEMY gates are down!");
            }
            sleep(10);
        }
        robot.liftinator1.setPower(0);
        robot.liftinator2.setPower(0);
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
        FtcChoiceMenu<Sampling> samplingMenu = new FtcChoiceMenu<>("Number of Samples:", startPositionMenu, this);
        FtcChoiceMenu<Depot> depotMenu = new FtcChoiceMenu<>("Go for Depot:", samplingMenu, this);
        FtcChoiceMenu<Crater> craterMenu = new FtcChoiceMenu<>("Crater:", depotMenu, this);

        modeMenu.addChoice("Auto", RunMode.RUNMODE_AUTO, true, delayMenu);
        modeMenu.addChoice("Debug", RunMode.RUNMODE_DEBUG, false, delayMenu);

        delayMenu.setChildMenu(startPositionMenu);

        startPositionMenu.addChoice("1", StartPosition.SILVER, true, samplingMenu);
        startPositionMenu.addChoice("2", StartPosition.GOLD, false, samplingMenu);

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
        startposition = startPositionMenu.getCurrentChoiceObject();
        crater = craterMenu.getCurrentChoiceObject();

        dashboard.displayPrintf(9, "Mode: %s (%s)", modeMenu.getCurrentChoiceText(), runmode.toString());
        dashboard.displayPrintf(11, "Delay = %d msec", delay);
        dashboard.displayPrintf(12, "Start position: %s (%s)", startPositionMenu.getCurrentChoiceText(), startposition.toString());
    }
    // END MENU ------------------------------------------------------------------------------------
}
