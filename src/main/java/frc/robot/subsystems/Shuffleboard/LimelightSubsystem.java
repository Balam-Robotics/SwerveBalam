// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.subsystems.LimelightHelpers;

public class LimelightSubsystem extends SubsystemBase {

  private final String limelight = "limelight-balam";
  
  private GenericEntry tx = ShuffleboardConstants.VisionTab.add("Tx", 0).getEntry();
  private GenericEntry ty = ShuffleboardConstants.VisionTab.add("Ty", 0).getEntry();
  private GenericEntry ta = ShuffleboardConstants.VisionTab.add("Ta", 0).getEntry();
  private GenericEntry tv = ShuffleboardConstants.VisionTab.add("Tv", false).getEntry();

  public LimelightSubsystem() {

    LimelightHelpers.SetFiducialIDFiltersOverride(limelight, LimelightConstants.kValidAprilTagIds);

  }

  public void init() {
    System.out.println("Limelight Subsystem Init");
    LimelightHelpers.setLEDMode_ForceOff(limelight);
  }

  public void toggleLED(boolean val) {
    if (val) {
      LimelightHelpers.setLEDMode_ForceOn(limelight);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(limelight);
    }
  }

  @Override
  public void periodic() {

    tx.setDouble(LimelightHelpers.getTX(limelight));
    ty.setDouble(LimelightHelpers.getTY(limelight));
    ta.setDouble(LimelightHelpers.getTA(limelight));
    tv.setBoolean(LimelightHelpers.getTV(limelight));

  }
}
