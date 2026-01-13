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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class ClimberSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  private RelativeEncoder m_encoder;

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

    m_encoder = m_primaryMotor.getEncoder();

  }

  public void setClimberSpeed(double speed) {
    m_primaryMotor.set(speed);
  }

  public void stopClimber() {
    m_primaryMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }
  public double getEncoderVelocity() {
    return  m_encoder.getVelocity();
  }

  private GenericEntry climberPositionEntry = ShuffleboardConstants.kClimberTab.add("Climber Position", 0.0)
  .withWidget(BuiltInWidgets.kGraph)
  .withSize(2, 2)
  .withPosition(0, 0)
  .getEntry();
  private GenericEntry climberVelocityEntry = ShuffleboardConstants.kClimberTab.add("Climber Velocity", 0.0)
  .withWidget(BuiltInWidgets.kAccelerometer)
  .withSize(2, 1)
  .withPosition(2, 0)
  .getEntry();
  private GenericEntry primaryMotorCurrent = ShuffleboardConstants.kClimberTab.add("Primary Motor Current", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 0)
  .getEntry();
  private GenericEntry secondaryMotorCurrent = ShuffleboardConstants.kClimberTab.add("Secondary Motor Current", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 1)
  .getEntry();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    climberPositionEntry.setDouble(getEncoderPosition());
    climberVelocityEntry.setDouble(getEncoderVelocity());
    primaryMotorCurrent.setDouble(m_primaryMotor.getOutputCurrent());
    secondaryMotorCurrent.setDouble(m_secondaryMotor.getOutputCurrent());
  }
}

