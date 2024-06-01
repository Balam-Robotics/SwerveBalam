// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShuffleboardConstants;

public class LimelightSubsystem extends SubsystemBase {

  NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight-balam");
  
  private GenericEntry tx = ShuffleboardConstants.VisionTab.add("Tx", 0).getEntry();
  private GenericEntry ty = ShuffleboardConstants.VisionTab.add("Tx", 0).getEntry();
  private GenericEntry ta = ShuffleboardConstants.VisionTab.add("Tx", 0).getEntry();
  private GenericEntry tv = ShuffleboardConstants.VisionTab.add("Tx", 0).getEntry();

  public LimelightSubsystem() {
  }

  public void init() {
    System.out.println("Limelight Subsystem Init");
    limelight.getEntry("ledMode").setNumber(1);
  }

  public void toggleLED(boolean val) {
    limelight.getEntry("ledMode").setNumber(val ? 3 : 1);
  }

  @Override
  public void periodic() {

    if (limelight.getEntry("tv").getDouble(0) == 1) {

      tx.setDouble(limelight.getEntry("tx").getDouble(0));
      ty.setDouble(limelight.getEntry("ty").getDouble(0));
      ta.setDouble(limelight.getEntry("ta").getDouble(0));
      tv.setDouble(limelight.getEntry("tv").getDouble(0));

    }

  }
}
