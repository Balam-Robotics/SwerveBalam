// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  
  private CANSparkMax upperLeft;
  private CANSparkMax upperRight;

  private CANSparkMax lowerLeft;
  private CANSparkMax lowerRight;

  public ShooterSubsystem() {
    upperLeft = new CANSparkMax(ShooterConstants.kUpperLeftCanId, MotorType.kBrushless);
    upperRight = new CANSparkMax(ShooterConstants.kUpperRightCanId, MotorType.kBrushless);

    lowerLeft = new CANSparkMax(ShooterConstants.kLowerLeftCanId, MotorType.kBrushless);
    lowerRight = new CANSparkMax(ShooterConstants.kLowerRightCanId, MotorType.kBrushless);
  }

  public void grabNote() {
    upperLeft.set(1);
    upperRight.set(-1);

    lowerLeft.set(-1);
    lowerRight.set(1);
  }

  public void prepareLaunch() {
    upperLeft.set(-1);
    upperRight.set(1);

    lowerLeft.set(0.1);
    lowerRight.set(-0.1);
  }

  public void launchNote() {
    upperLeft.set(-1);
    upperRight.set(1);

    lowerLeft.set(-1);
    lowerRight.set(1);
  }

  public void stopShooter() {
    upperLeft.set(0);
    upperRight.set(0);
    lowerLeft.set(0);
    lowerRight.set(0);
  }

}
