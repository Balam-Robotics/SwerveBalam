package frc.robot.util;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
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
    private MjpegServer mjpegServer1, mjpegServer2;

    public CameraSystem() {
        // Start capturing from the USB camera
        UsbCamera usbCamera = CameraServer.startAutomaticCapture();
        usbCamera.setResolution(640, 480);
        usbCamera.setFPS(30);

        // Start camera server
        mjpegServer1 = new MjpegServer("server_usb Camera 0", 1182);
        mjpegServer1.setSource(usbCamera);

        // Start capturing from the Limelight camera
        HttpCamera limelightCamera = new HttpCamera(CameraConstants.kLimelightName,
                "http://roborio-3527-frc.local:1181/stream.mjpg");

        // Start second sercer
        mjpegServer2 = new MjpegServer("server_Limelight", 1182);
        mjpegServer2.setSource(limelightCamera);

        if (OIConstants.kDebug) {
            ShuffleboardConstants.kDebugTab.add("Camera Server", mjpegServer1.getSource())
                    .withWidget(BuiltInWidgets.kCameraStream)
                    .withPosition(2, 3)
                    .withSize(2, 2);
            ShuffleboardConstants.kDebugTab.add("Limelight Stream", mjpegServer2.getSource())
                    .withWidget(BuiltInWidgets.kCameraStream)
                    .withPosition(4, 3)
                    .withSize(2, 2);
        } else {
            ShuffleboardConstants.kDriverTab.add("Camera Server", mjpegServer1.getSource())
                    .withWidget(BuiltInWidgets.kCameraStream)
                    .withPosition(2, 0)
                    .withSize(2, 20);
            ShuffleboardConstants.kDriverTab.add("Limelight Stream", mjpegServer2.getSource())
                    .withWidget(BuiltInWidgets.kCameraStream)
                    .withPosition(2, 2)
                    .withSize(2, 2);
        }
    }

}
