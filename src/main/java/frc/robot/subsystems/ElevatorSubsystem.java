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

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class ElevatorSubsystem extends SubsystemBase {

  private SparkMax m_primaryMotor;
  private SparkMax m_secondaryMotor;

  private RelativeEncoder m_primaryEncoder;

  public static final SparkMaxConfig primaryMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    m_primaryMotor = new SparkMax(ElevatorConstants.kPrimaryElevatorMotorId, MotorType.kBrushless);
    m_secondaryMotor = new SparkMax(ElevatorConstants.kSecondaryElevatorMotorId, MotorType.kBrushless);

    primaryMotorConfig
      .idleMode(ElevatorConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ElevatorConstants.kPrimaryCurrentLimit);
    primaryMotorConfig.closedLoop
      .pidf(0.02, 0, 0 , 0);
    secondaryMotorConfig
      .follow(ElevatorConstants.kPrimaryElevatorMotorId, true) // Sepa si esto va a funcionar
      .idleMode(ElevatorConstants.kPrimaryIdleMode)
      .smartCurrentLimit(ElevatorConstants.kSecondaryCurrentLimit)
      .inverted(true);
    
    m_primaryMotor.configure(primaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_secondaryMotor.configure(secondaryMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_primaryEncoder = m_primaryMotor.getEncoder();
    m_primaryEncoder.setPosition(0);
  }

  public void setElevatorPosition(double position) {
    m_primaryMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }
  public void setDEBUGElevatorPosition() {
    m_primaryMotor.getClosedLoopController().setReference(elevatorPIDEntry.getDouble(0.0), ControlType.kPosition);
  }

  private GenericEntry elevatorPIDEntry = ShuffleboardConstants.kElevatorTab.add("Elevator PID", 0)
  .withSize(2, 1)
  .withPosition(2, 1)
  .withProperties(Map.of("min_value", 0, "max_value", 50, "show_submit_button", true))
  .getEntry();

  public void setElevatorSpeed(double speed) {
    m_primaryMotor.set(speed);
  }

  public void stopElevator() {
    m_primaryMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_primaryEncoder.getPosition(); 
  }

  public double getEncoderVelocity() {
    return m_primaryEncoder.getVelocity();
  }

  
  private GenericEntry elevatorPosition = ShuffleboardConstants.kElevatorTab.add("Elevator Position", 0.0)
  .withWidget(BuiltInWidgets.kGraph)
  .withSize(2,2)
  .withPosition(0, 0)
  .getEntry();
  private GenericEntry elevatorVelocity = ShuffleboardConstants.kElevatorTab.add("Elevator Velocity", 0.0)
  .withWidget(BuiltInWidgets.kAccelerometer)
  .withSize(2, 1)
  .withPosition(2, 0)
  .getEntry();
  private GenericEntry primaryMotorCurrent = ShuffleboardConstants.kElevatorTab.add("Primary Motor Current", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 0)
  .getEntry();
  private GenericEntry secondaryMotorCurrent = ShuffleboardConstants.kElevatorTab.add("Secondary Motor Current", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 1)
  .getEntry();
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     
    elevatorPosition.setDouble(getEncoderPosition());
    elevatorVelocity.setDouble(getEncoderVelocity());
    primaryMotorCurrent.setDouble(m_primaryMotor.getOutputCurrent());
    secondaryMotorCurrent.setDouble(m_secondaryMotor.getOutputCurrent());
    
    //System.out.printf("Elevator Position: %.2f | Velocity: %.2f | Primary Current: %.2f | Secondary Current: %.2f\n", 
      //encoder.getPosition(), encoder.getVelocity(), m_primaryMotor.getOutputCurrent(), m_secondaryMotor.getOutputCurrent());
  }
}
