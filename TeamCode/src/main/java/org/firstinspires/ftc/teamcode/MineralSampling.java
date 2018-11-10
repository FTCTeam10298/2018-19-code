package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
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
 * A nice demo class for using OpenCVPipeline. This one also demonstrates how to use OpenCV to threshold
 * for a certain color (blue) and find contours of objects of that color, which is very common in
 * robotics OpenCV applications.
 */

public class MineralSampling extends OpenCVPipeline {
    private boolean showContours = true;
    private boolean showThresholdedGold = false;
    private boolean showThresholdedSilver = false;
    // To keep it such that we don't have to instantiate a new Mat every call to processFrame,
    // we declare the Mats up here and reuse them. This is easier on the garbage collector.
    private Mat hsv = new Mat();
    private Mat thresholdedGold = new Mat();
    private Mat thresholdedSilver = new Mat();
    private int h1G = 10;
    private int s1G = 128;
    private int v1G = 100;
    private int h2G = 30;
    private int s2G = 255;
    private int v2G = 255;

    private int h1S = 0;
    private int s1S = 0;
    private int v1S = 200;
    private int h2S = 255;
    private int s2S = 40;
    private int v2S = 255;
    private int[] m_score = new int[3];
    private int[] score = new int[3];

    // yellow

    public double[] color = new double[4];

    // this is just here so we can expose it later thru getContours.
    private List<MatOfPoint> contoursGold    = new ArrayList<>();
    private List<MatOfPoint> countoursSilver = new ArrayList<>();

    public synchronized void setShowCountours(boolean enabled) {
        showContours = enabled;
    }
    public synchronized void setShowThresholded (boolean enabled) {
        showThresholdedGold = enabled;
        showThresholdedSilver = enabled;
    }
    public synchronized List<MatOfPoint> getContoursGold() {
        return contoursGold;
    }
    public synchronized List<MatOfPoint> getCountoursSilver() {
        return countoursSilver;
    }

    public synchronized void setThresholds (boolean gold, int hu1, int sa1, int va1, int hu2, int sa2, int va2){

        if (gold) {
            h1G = Range.clip(hu1, 0, 255);
            s1G = Range.clip(sa1, 0, 255);
            v1G = Range.clip(va1, 0, 255);
            h2G = Range.clip(hu2, 0, 255);
            s2G = Range.clip(sa2, 0, 255);
            v2G = Range.clip(va2, 0, 255);
        }
        else {
            h1S = Range.clip(hu1, 0, 255);
            s1S = Range.clip(sa1, 0, 255);
            v1S = Range.clip(va1, 0, 255);
            h2S = Range.clip(hu2, 0, 255);
            s2S = Range.clip(sa2, 0, 255);
            v2S = Range.clip(va2, 0, 255);
        }

    }

    public synchronized double[] getCenter () { return color; }
    public synchronized int[]    getScore  (){
        return score;
    }

    // This is called every camera frame.
    @Override
    public Mat processFrame(Mat rgba, Mat gray) {
        // First, we change the colorspace from RGBA to HSV, which is usually better for color
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV, 3);
        // Then, we threshold our hsv image so that we get a black/white binary image where white
        // is the blues listed in the specified range of values
        // you can use a program like WPILib GRIP to find these values, or just play around.
        Core.inRange(hsv, new Scalar(h1G, s1G, v1G), new Scalar(h2G, s2G, v2G), thresholdedGold);
        Core.inRange(hsv, new Scalar(h1S, s1S, v1S), new Scalar(h2S, s2S, v2S), thresholdedSilver);

        // we blur the thresholded image to remove noise
        // there are other types of blur like box blur or gaussian which can be explored.
        Imgproc.blur(thresholdedGold, thresholdedGold, new Size(3, 3));
        Imgproc.blur(thresholdedSilver, thresholdedSilver, new Size(3, 3));

        color = hsv.get(hsv.height()/2,hsv.width()/2);
        // create a list to hold our contours.
        // Conceptually, there is going to be a single contour for the outline of every blue object
        // that we can find. We can iterate over them to find objects of interest.
        // the Imgproc module has many functions to analyze individual contours by their area, avg position, etc.
        contoursGold = new ArrayList<>();
        countoursSilver = new ArrayList<>();
        // this function fills our contours variable with the outlines of blue objects we found
        Imgproc.findContours(thresholdedGold, contoursGold, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.findContours(thresholdedSilver, countoursSilver, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        // Then we display our nice little binary threshold on screen
        if (showContours) {
            // this draws the outlines of the blue contours over our original image.
            // they are highlighted in green.

            Imgproc.drawContours(rgba, countoursSilver, -1, new Scalar(255, 0, 0), 2, 8);
            Imgproc.drawContours(rgba, contoursGold, -1, new Scalar(0, 255, 0), 2, 8);
        }
        if (showThresholdedGold) {

            rgba = thresholdedGold;
        }
        if (showThresholdedSilver) {

            rgba = thresholdedSilver;
        }

        m_score[0] = 0;
        m_score[1] = 0;
        m_score[2] = 0;
        for (int i = 0; i < thresholdedGold.height()-4; i = i+2) {
            for (int j = 0; j < thresholdedGold.width()-4; j = j+2){
                if (j<=(thresholdedGold.width()/3))
                    m_score[0]+=(thresholdedGold.get(i, j)[0])/255;
                else if (j<=((2*thresholdedGold.width()/3)+10))
                    m_score[1]+=(thresholdedGold.get(i, j)[0])/255;
                else
                    m_score[2]+=(thresholdedGold.get(i, j)[0])/255;
            }
        }
        for (int i = 0; i < thresholdedSilver.height()-4; i = i+2) {
            for (int j = 0; j < thresholdedSilver.width()-4; j = j+2){
                if (j<=(thresholdedSilver.width()/3))
                    m_score[0]-=(thresholdedSilver.get(i, j)[0])/255;
                else if (j<=((2*thresholdedSilver.width()/3)+10))
                    m_score[1]-=(thresholdedSilver.get(i, j)[0])/255;
                else
                    m_score[2]-=(thresholdedSilver.get(i, j)[0])/255;
            }
        }
        score[0] = m_score[0];
        score[1] = m_score[1];
        score[2] = m_score[2];

        return rgba; // display the image seen by the camera
    }
}
