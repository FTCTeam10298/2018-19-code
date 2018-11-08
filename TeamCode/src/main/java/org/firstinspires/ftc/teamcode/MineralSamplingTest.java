package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.corningrobotics.enderbots.endercv.CameraViewDisplay;
import org.opencv.core.MatOfPoint;

import java.util.List;

/**
 * Created by guinea on 10/5/17.
 * -------------------------------------------------------------------------------------
 * Copyright (c) 2018 FTC Team 5484 Enderbots
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * 
 * By downloading, copying, installing or using the software you agree to this license.
 * If you do not agree to this license, do not download, install,
 * copy or use the software.
 * -------------------------------------------------------------------------------------
 * This is a sample opmode that demonstrates the use of an OpenCVPipeline with FTC code.
 * When the x button is pressed on controller one, the camera is set to show areas of the image
 * where a certain color is, in this case, blue.
 *
 * Additionally, the centers of the bounding rectangles of the contours are sent to telemetry.
 */
@TeleOp(name="MineralSamplingTest")
public class MineralSamplingTest extends OpMode {
    private MineralSampling mineralSampling;

    private double[] colorZ = new double[4];
    private double hue = 0;
    private double sat = 0;
    private double val = 0;
    @Override
    public void init() {
        mineralSampling = new MineralSampling();
        // can replace with ActivityViewDisplay.getInstance() for fullscreen
        mineralSampling.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        mineralSampling.setShowCountours(false);
        // start the vision system
        mineralSampling.enable();
    }

    @Override
    public void loop() {
        // update the settings of the vision pipeline
        mineralSampling.setShowCountours(gamepad1.x);
        mineralSampling.setShowThresholded(gamepad1.y);

        colorZ = mineralSampling.getCenter();

        hue = colorZ[0];
        sat = colorZ[1];
        val = colorZ[2];

        // get a list of contours from the vision system
        List<MatOfPoint> contoursGold = mineralSampling.getContoursGold();
        List<MatOfPoint> contoursSilver = mineralSampling.getCountoursSilver();
//        for (int i = 0; i < contours.size(); i++) {
//            // get the bounding rectangle of a single contour, we use it to get the x/y center
//            // yes there's a mass center using Imgproc.moments but w/e
//            Rect boundingRect = Imgproc.boundingRect(contours.get(i));
//            telemetry.addData("contour" + Integer.toString(i),
//                    String.format(Locale.getDefault(), "(%d, %d)", (boundingRect.x + boundingRect.width) / 2, (boundingRect.y + boundingRect.height) / 2));
//        }

        mineralSampling.setThresholds(true, 10, 128, 100, 30, 255, 255);
        mineralSampling.setThresholds(false, 0, 0, 200, 255, 40, 255);


        int[] score = mineralSampling.getScore();
        telemetry.addData("Center Hue:", hue);
        telemetry.addData("Center Saturation:", sat);
        telemetry.addData("Center Value:", val);
        telemetry.addData("Score Left:", score[0]);
        telemetry.addData("Score Center:", score[1]);
        telemetry.addData("Score Right:", score[2]);
    }

    public void stop() {
        // stop the vision system
        mineralSampling.disable();
    }
}
