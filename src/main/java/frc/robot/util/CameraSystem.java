package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;;

@SuppressWarnings("unused")
public class CameraSystem extends SubsystemBase {
  private UsbCamera usbCameraFront, usbCameraCage;
  private HttpCamera limelightCamera;

  public void init() {
    try {
      usbCameraFront = CameraServer.startAutomaticCapture(0);
      usbCameraFront.setResolution(320, 240);
      usbCameraFront.setFPS(30);
      System.out.println("[CameraSystem] Front camera started!");
  } catch (Exception e) {
      System.out.println("[CameraSystem] Front camera not detected: " + e.getMessage());
  }
  
  /*
  try {
      usbCameraCage = CameraServer.startAutomaticCapture(1);
      usbCameraCage.setResolution(320, 240);
      usbCameraCage.setFPS(30);
      System.out.println("[CameraSystem] Cage camera started!");
  } catch (Exception e) {
      System.out.println("[CameraSystem] Cage camera not detected: " + e.getMessage());
  } */

    // Start capturing from the Limelight camera
    limelightCamera = new HttpCamera(CameraConstants.kLimelightName,
        "http://roborio-3527-frc.local:5800/stream.mjpg");

    ShuffleboardConstants.kDriverTab.add("Limelight Camera", limelightCamera)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(2, 2);
    ShuffleboardConstants.kDriverTab.add("Front Camera", usbCameraFront)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 2)
        .withSize(2, 2);/*
    ShuffleboardConstants.kDriverTab.add("Cage Stream", usbCameraCage)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(4, 2)
        .withSize(2, 2); */

    // Autonomus Elastic Tab

    ShuffleboardConstants.kAutonomousTab.add("Limelight Stream", limelightCamera)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(0, 0)
        .withSize(4, 4);
  }

  public CameraSystem() {
  }

}
