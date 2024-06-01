// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  private CANSparkMax upperIntake;
  private CANSparkMax lowerIntake;

  public IntakeSubsystem() {
    upperIntake = new CANSparkMax(IntakeConstants.kUpperIntakeCanId, MotorType.kBrushless);
    lowerIntake = new CANSparkMax(IntakeConstants.kLowerIntakeCanId, MotorType.kBrushless);
  }

  public void enableIntake() {
    upperIntake.set(1);
    lowerIntake.set(-1);
  }

  public void disableIntake() {
    upperIntake.set(0);
    lowerIntake.set(0);
  }

}
