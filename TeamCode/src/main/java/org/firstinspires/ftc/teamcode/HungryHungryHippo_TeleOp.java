/* Copyright (c) 2017 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Hungry Hungry Hippo.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common HungryHippo hardware class to define the devices on the robot.
 * All device access is managed through the HardwareHungryHippo class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a Hungry Hungry Hippo
 * It opens and closes the mouth using the left and right bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Hungry Hungry Hippo", group="HungryHippo")
public class HungryHungryHippo_TeleOp extends OpMode {

    /* Declare OpMode members. */
    HungryHungryHippo_Hardware robot       = new HungryHungryHippo_Hardware();
    boolean secretSwitch = false;
    boolean secretSwitcher = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hippo is HUNGRY HUNGRY");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;

        if (gamepad1.left_trigger>.5&&gamepad1.right_trigger>.5&&gamepad1.left_stick_button&&gamepad1.right_stick_button&&gamepad1.y)
            secretSwitcher = true;
        if (secretSwitcher&&!(gamepad1.left_trigger>.5&&gamepad1.right_trigger>.5&&gamepad1.left_stick_button&&gamepad1.right_stick_button&&gamepad1.y)) {
            secretSwitch = !secretSwitch;
            secretSwitcher = false;
        }

        if (!secretSwitch){
            left/=2.5;
            right/=2.5;
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        if (gamepad1.right_bumper)
            robot.hippoMouth.setPower(1);
        else if (gamepad1.left_bumper)
            robot.hippoMouth.setPower(-1);
        else
            robot.hippoMouth.setPower(0);

        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        if (secretSwitch)
            telemetry.addData("Full Capacity:", secretSwitch);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
