// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 




  .______        ___       __          ___      .___  ___. 
  |   _  \      /   \     |  |        /   \     |   \/   | 
  |  |_)  |    /  ^  \    |  |       /  ^  \    |  \  /  | 
  |   _  <    /  /_\  \   |  |      /  /_\  \   |  |\/|  | 
  |  |_)  |  /  _____  \  |  `----./  _____  \  |  |  |  | 
  |______/  /__/     \__\ |_______/__/     \__\ |__|  |__| 
  




*/      

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  public static final SparkMaxConfig primaryMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

    m_primaryMotor = new SparkMax(ClimberConstants.kPrimaryMotorId, MotorType.kBrushless);
    m_secondaryMotor = new SparkMax(ClimberConstants.kSecondaryMotorId, MotorType.kBrushless);

    primaryMotorConfig
      .idleMode(ClimberConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ClimberConstants.kPrimaryCurrentLimit)
      .voltageCompensation(12);
    secondaryMotorConfig
      .follow(ClimberConstants.kPrimaryMotorId, true)
      .idleMode(ClimberConstants.kSecondaryIdleMode)
      .smartCurrentLimit(ClimberConstants.kSecondaryCurrentLimit)
      .inverted(true)
      .voltageCompensation(12);

      m_primaryMotor.configure(primaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setClimberSpeed(double speed) {
    System.out.println("Setting climber speed to " + speed);
    m_primaryMotor.set(speed);
  }

  public void stopClimber() {
    System.out.println("Stopping climber");
    m_primaryMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
