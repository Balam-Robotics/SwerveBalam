// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShuffleboardConstants;

public class NavX extends SubsystemBase {
  /** Creates a new NavX. */

  private AHRS m_gyro;

  public NavX() {
    m_gyro = new AHRS(SPI.Port.kMXP);
  }

  public void init() {
    System.out.println("NavX Subsystem Init");
  }

  private GenericEntry isConnected = ShuffleboardConstants.SwerveTab.add("Connected", false).getEntry();
  private GenericEntry isCalibrating = ShuffleboardConstants.SwerveTab.add("Calibrating", false).getEntry();
  private GenericEntry gyroYaw = ShuffleboardConstants.SwerveTab.add("Yaw", 0).getEntry();
  private GenericEntry gyroPitch = ShuffleboardConstants.SwerveTab.add("Pitch", 0).getEntry();
  private GenericEntry gyroRoll = ShuffleboardConstants.SwerveTab.add("Roll", 0).getEntry();

  private GenericEntry gyroAngle = ShuffleboardConstants.SwerveTab.add("Angle", 0).getEntry();
  private GenericEntry isMoving = ShuffleboardConstants.SwerveTab.add("Moving", false).getEntry();
  private GenericEntry isRotating = ShuffleboardConstants.SwerveTab.add("Rotating", false).getEntry();

  @Override
  public void periodic() {

    /* Display 6-axis Processed Angle Data */
    isConnected.setBoolean(m_gyro.isConnected());
    isCalibrating.setBoolean(m_gyro.isCalibrating());
    gyroYaw.setDouble(Math.round(m_gyro.getYaw()));
    gyroPitch.setDouble(Math.round(m_gyro.getPitch()));
    gyroRoll.setDouble(Math.round(m_gyro.getRoll()));

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */

    gyroAngle.setDouble(Math.round(m_gyro.getAngle()));

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    isMoving.setBoolean(m_gyro.isMoving());
    isRotating.setBoolean(m_gyro.isRotating());
  }
}
