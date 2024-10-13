package com.team6647.util;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;

public class CameraServerThread {
    private static Thread visionThread;
    public static void init(){
        visionThread = new Thread(()->{
            UsbCamera camera = CameraServer.startAutomaticCapture();
            camera.setResolution(640, 480);
            camera.setFPS(15);
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
            CvSink  cvSink = CameraServer.getVideo();
            Mat source = new Mat();
            Mat output = new Mat();
            while (!Thread.interrupted()) {
                if(cvSink.grabFrame(source)==0){
                    continue;
                }
                Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
                outputStream.putFrame(output);
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        );
        visionThread.setDaemon(true);
        visionThread.start();
    }
}
