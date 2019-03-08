/*
Copyright (c) 2018, FTC team #10298 Brain Stormz

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Brain Stormz nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

/**
 * This file provides Teleop driving for our robot.
 * The code is structured as an Iterative OpMode.
 *
 * This OpMode uses the common Brian Stormz hardware class to define the devices on the robot.
 * All device access is managed through the FBrian_Stormz_Hardware class.
 */

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Brian TeleOp", group="Brian")
public class Brian_TeleOp extends OpMode {

    /* Declare OpMode members. */
    Brian_Hardware robot = new Brian_Hardware(); // use the class created to define FutureBot's hardware

    double  x = 0;
    double  y = 0;
    double  z = 0;

    double inertia      = 0.5;

    double time_a       = 0;
    double dt           = 0;
    double time_stopped = 0;
    int    direction    = 1;

    boolean collectOtronACTIVE     = false;
    boolean collectOtronSWITCHING  = false;
    boolean collectOtronREVERSE    = false;
    boolean refinatorACTIVE        = false;
    boolean quickmodeSWITCHING     = false;
    boolean quickmodeACTIVE        = false;

    // Code to run once when the driver hits INIT
    @Override
    public void init() {

         // Initialize the hardware variables.
         // The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting
        telemetry.addData("Say", "Robot ready");

    }

    // Code to run in a loop after the driver hits play until they hit the stop button
    @Override
    public void loop() {

        // Calculate loop Period (dt).
        // Let's not repeat last year's failure/laziness that killed our performance at Regionals...
        dt = getRuntime() - time_a;
        time_a = getRuntime();
        telemetry.addData("Loop Time", "%f", dt);
        telemetry.addData("Inertia", "%f", inertia);
        telemetry.addData("Arm power", "%f", robot.pivotArm1.getPower());
        if (quickmodeACTIVE)
            telemetry.addData("Arm QuickMode", "Enabled");
        else
            telemetry.addData("Arm QuickMode", "Disabled");

        // Send telemetry message to signify robot running
        telemetry.addData("Say", "N8 is the gr8est without deb8");
        //telemetry.addData("Extension arm encoder count", robot.extendoArm5000.getCurrentPosition());

        if (gamepad1.dpad_down || gamepad2.dpad_down) {
            robot.DrivePowerAll(.5);
        } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
            robot.DrivePowerAll(-.5);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            DriveSideways(.5);
        } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
            DriveSideways(-.5);
        }
        // Drone drive
        else {
            if (gamepad1.left_stick_y > .1 || gamepad1.left_stick_y < -.1) {
                y = gamepad1.left_stick_y;
            } else if (gamepad2.left_stick_y > .1 || gamepad2.left_stick_y < -.1) {
                y = gamepad2.left_stick_y;
            }
            else {
                y = 0;
            }

            if (gamepad1.left_stick_x > .1 || gamepad1.left_stick_x < -.1) {
                x = gamepad1.left_stick_x;
            } else if (gamepad2.left_stick_x > .1 || gamepad2.left_stick_x < -.1) {
                x = gamepad2.left_stick_x;
            }
            else {
                x = 0;
            }

            if (gamepad1.right_stick_x > .1 || gamepad1.right_stick_x < -.1) {
                z = -gamepad1.right_stick_x / 2;
            } else if (gamepad2.right_stick_x > .1 || gamepad2.right_stick_x < -.1) {
                z = -gamepad2.right_stick_x / 2;
            }
            else {
                z = 0;
            }

            double maxvalue = abs(y + x - z);
            if (abs(y + x - z) > maxvalue) {
                maxvalue = abs(y + x - z);
            }
            if (abs(y - x + z) > maxvalue) {
                maxvalue = abs(y - x + z);
            }
            if (abs(y + x + z) > maxvalue) {
                maxvalue = abs(y + x + z);
            }
            if (abs(y - x - z) > maxvalue) {
                maxvalue = abs(y - x - z);
            }
            if (maxvalue < 1.0) {
                maxvalue = 1;
            }

            double frontLeftPower  = (-1 * Range.clip(((y - x + z) / maxvalue), -1.0, 1.0));
            double frontRightPower = (-1 * Range.clip(((y + x - z) / maxvalue), -1.0, 1.0));
            double backLeftPower   = (-1 * Range.clip(((y + x + z) / maxvalue), -1.0, 1.0));
            double backRightPower  = (-1 * Range.clip(((y - x - z) / maxvalue), -1.0, 1.0));

            if ((frontLeftPower > 0.1 || frontRightPower > 0.1 || backLeftPower > 0.1 || backRightPower > 0.1)
                    || (frontLeftPower < -0.1 || frontRightPower < -0.1 || backLeftPower < -0.1 || backRightPower < -0.1))
            {
                inertia += (0.4*dt);
                inertia = Range.clip(inertia, 0, 1);
            }
            else
            {
                inertia = 0.5;
            }

            robot.driveSetPower(frontLeftPower*inertia, frontRightPower*inertia,
                                backLeftPower*inertia, backRightPower*inertia);
        }

        if (gamepad1.y || gamepad2.y) {
            refinatorACTIVE = false;
            time_stopped = 0;
            robot.pivotArm1.setPower(1);
            robot.pivotArm2.setPower(1);
            direction = 1;
        }
        else if (gamepad1.a || gamepad2.a) {
            refinatorACTIVE = false;
            time_stopped = 0;
            robot.pivotArm1.setPower(-1);
            robot.pivotArm2.setPower(-1);
            direction = -1;
        }
        else if (gamepad1.x || gamepad2.x) {
            refinatorACTIVE = true;
            time_stopped = 0;
            robot.pivotArm1.setPower(-.3);
            robot.pivotArm2.setPower(-.3);
            direction = -1;
        }
        else if (!refinatorACTIVE) {
//            if (time_stopped < 0.25) {
//                if ((robot.pivotArm1.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT) ||
//                robot.pivotArm2.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.FLOAT) {
//                    robot.pivotArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                    robot.pivotArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                }
//            } else {
//                if ((robot.pivotArm1.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) ||
//                        robot.pivotArm2.getZeroPowerBehavior() != DcMotor.ZeroPowerBehavior.BRAKE) {
//                    robot.pivotArm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    robot.pivotArm2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                }
//            }
            if (quickmodeACTIVE) {
                robot.pivotArm1.setPower(0);
                robot.pivotArm2.setPower(0);
            }
            time_stopped += dt;
            robot.pivotArm1.setPower(direction*Range.clip(abs(robot.pivotArm1.getPower())-(dt*4*abs(robot.pivotArm1.getPower()))-dt*0.5, 0, 1));
            robot.pivotArm2.setPower(direction*Range.clip(abs(robot.pivotArm2.getPower())-(dt*4*abs(robot.pivotArm2.getPower()))-dt*0.5, 0, 1));
        }

        if (gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad2.left_stick_button || gamepad2.right_stick_button) {
            quickmodeSWITCHING = true;
        }
        else if (quickmodeSWITCHING) {
            quickmodeSWITCHING = false;
            if (quickmodeACTIVE)
                quickmodeACTIVE = false;
            else
                quickmodeACTIVE = true;
        }

        if (gamepad1.right_trigger > 0)
            robot.extendoArm5000.setPower(-gamepad1.right_trigger);
        else if (gamepad2.right_trigger > 0)
            robot.extendoArm5000.setPower(-gamepad2.right_trigger);
        else if (gamepad1.left_trigger > 0)
            robot.extendoArm5000.setPower(gamepad1.left_trigger);
        else if (gamepad2.left_trigger > 0)
            robot.extendoArm5000.setPower(gamepad2.left_trigger);
        else
            robot.extendoArm5000.setPower(0);

        if (gamepad1.left_bumper || gamepad1.right_bumper || gamepad2.left_bumper || gamepad2.right_bumper)
            collectOtronSWITCHING = true;
        else if (collectOtronSWITCHING) {
            collectOtronSWITCHING = false;
            if (collectOtronACTIVE)
                collectOtronACTIVE = false;
            else
                collectOtronACTIVE = true;
        }

        if (gamepad1.left_bumper || gamepad2.left_bumper)
            collectOtronREVERSE = true;
        else if (gamepad1.right_bumper || gamepad2.right_bumper)
            collectOtronREVERSE = false;

        if (collectOtronACTIVE && !collectOtronREVERSE) {
            robot.collectOtron.setPower(1);
        }
        else if (collectOtronACTIVE)
            robot.collectOtron.setPower(-1);
        else
            robot.collectOtron.setPower(0);

        // Collector lid
        if (gamepad1.b || gamepad2.b)
            robot.collectorGate.setPosition(.25);
        else {
            robot.collectorGate.setPosition(.65);
        }

    }

    /**
    FUNCTIONS------------------------------------------------------------------------------------------------------
     */
    void DriveSideways (double power)
    {
        robot.driveSetPower(-power, power, power, -power);
    }
}
