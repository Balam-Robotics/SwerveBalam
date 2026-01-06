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

import com.revrobotics.AbsoluteEncoder;
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

  private AbsoluteEncoder m_primaryEncoder;
  private int m_counter;  

  public static final SparkMaxConfig primaryMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig secondaryMotorConfig = new SparkMaxConfig();

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    //System.out.println("Creates a new ElevatorSubsystem");
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

    m_primaryEncoder = m_primaryMotor.getAbsoluteEncoder();
  }

  public void setElevatorPosition(double position) {
    System.out.println("Moving elevator position to " + position +  " | Relative Encoder position " + m_primaryEncoder.getPosition());
    m_primaryMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  private GenericEntry shuffleBoardPos = ShuffleboardConstants.kDebugTab.add("Elevator PID", 0)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withSize(2, 1)
  .withPosition(0, 2)
  .withProperties(Map.of("min_value", -55, "max_value", 0))
  .getEntry();

  public void setShuffleboardPIDElevator() {
    m_primaryMotor.getClosedLoopController().setReference(shuffleBoardPos.getDouble(0), ControlType.kPosition);
  }

  public void xd(double speed) {
    System.out.println("Changing elevator speed to " + speed);
    if ((speed < 0 && getEncoderPosition() >= ElevatorConstants.kMinHeight) || (speed > 0 && getEncoderPosition() <= ElevatorConstants.kMaxHeight) ) {
      m_primaryMotor.set(speed);
    }
  }
  public void setElevatorSpeed(double speed) {
    System.out.println("Changing elevator speed to " + speed);
        m_primaryMotor.set(speed);
        heightController((int) Math.round(speed));
        System.out.println("Absolute Encoder: " + getEncoderPosition() + 360 * m_counter);
  }

  public void heightController(int input) {
    if(input > 0 && getEncoderPosition() == 360) {
      m_counter += 1;
    } else if(input < 0 && getEncoderPosition() == 0) {
      m_counter += 0;
    }
  }

  public void stopElevator() {
    System.out.println("Stopping Elevator");
    m_primaryMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_primaryEncoder.getPosition(); 
  }

  public double getEncoderVelocity() {
    return m_primaryEncoder.getVelocity();
  }

  public double getElevatorHeight() {
    return getEncoderPosition() + 360 * Math.floor(getEncoderPosition() / 360);
  }

  private GenericEntry elevatorPosition = ShuffleboardConstants.kElevatorTab.add("Elevator Position", 0.0).getEntry();
  private GenericEntry elevatorVelocity = ShuffleboardConstants.kElevatorTab.add("Elevator Velocity", 0.0).getEntry();
  private GenericEntry elevatorHeight = ShuffleboardConstants.kElevatorTab.add("Elevator Height", 0.0).getEntry();
  private GenericEntry primaryMotorCurrent = ShuffleboardConstants.kElevatorTab.add("Primary Motor Current", 0.0).getEntry();
  private GenericEntry secondaryMotorCurrent = ShuffleboardConstants.kElevatorTab.add("Secondary Motor Current", 0.0).getEntry();

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorPosition.setDouble(getEncoderPosition());
    elevatorVelocity.setDouble(getEncoderVelocity());
    elevatorHeight.setDouble(getElevatorHeight());
    primaryMotorCurrent.setDouble(m_primaryMotor.getOutputCurrent());
    secondaryMotorCurrent.setDouble(m_secondaryMotor.getOutputCurrent());
  }
}
