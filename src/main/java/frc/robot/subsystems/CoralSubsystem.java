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
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ShuffleboardConstants;

public class CoralSubsystem extends SubsystemBase {

  private SparkMax m_wristMotor;
  private SparkMax m_intakeMotor;

  private RelativeEncoder m_wristEncoder;
  private RelativeEncoder m_intakeEncoder;

  public static final SparkMaxConfig wristMotorConfig = new SparkMaxConfig();
  public static final SparkMaxConfig intakeMotorConfig = new SparkMaxConfig();

  public boolean isEjectingCoral = false;
  public boolean isIntakingCoral = false;

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {

    m_wristMotor = new SparkMax(CoralConstants.kWristMotorId, MotorType.kBrushless);
    m_intakeMotor = new SparkMax(CoralConstants.kIntakeMotorId, MotorType.kBrushless);

    wristMotorConfig
    
      .idleMode(CoralConstants.kWristIdleMode)
      .smartCurrentLimit(CoralConstants.kWristCurrentLimit);
    wristMotorConfig.closedLoop
      .pidf(CoralConstants.kWristPIDkP, CoralConstants.kWristPIDkI, CoralConstants.kWristPIDkD, 0);
    intakeMotorConfig
      .idleMode(CoralConstants.kIntakeIdleMode)
      .smartCurrentLimit(CoralConstants.kIntakeCurrentLimit);

      m_wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_wristEncoder = m_wristMotor.getEncoder();
      m_wristEncoder.setPosition(0);

      m_intakeEncoder = m_intakeMotor.getEncoder();
      m_intakeEncoder.setPosition(0);
  }

  public void setWristAngle(double position) {
    m_wristMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void setWristSpeed(double speed) {
    m_wristMotor.set(speed);
  }

  public void setDEBUGWristAngle() {
    m_wristMotor.getClosedLoopController().setReference(wristPIDEntry.getDouble(0.0), ControlType.kPosition);
  }

  private GenericEntry wristPIDEntry = ShuffleboardConstants.kCoralTab.add("Wrist PID", 0.0)
  .withSize(2,1)
  .withPosition(2, 1)
  .withProperties(Map.of("show_submit_button", true))
  .getEntry();
  private GenericEntry wristPosition = ShuffleboardConstants.kCoralTab.add("Wrist Angle", 0.0)
  .withWidget(BuiltInWidgets.kGraph)
  .withSize(2, 2) 
  .withPosition(0, 0)
  .getEntry();
  private GenericEntry wristVelocity = ShuffleboardConstants.kCoralTab.add("Wrist Velocity", 0.0)
  .withWidget(BuiltInWidgets.kAccelerometer)
  .withSize(2, 1)
  .withPosition(2, 0) 
  .getEntry();
  

  private ShuffleboardLayout valuesLayout = ShuffleboardConstants.kCoralTab
  .getLayout("Coral Booleans", BuiltInLayouts.kList)
  .withSize(2, 3)
  .withPosition(6, 0);

  {
    valuesLayout.addBoolean("Ejecting Coral", () -> isEjectingCoral);
    valuesLayout.addBoolean("Intaking Coral", () -> isIntakingCoral);
  }
 

  private GenericEntry wristVoltage = ShuffleboardConstants.kCoralTab.add("Wrist Voltage", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 0)
  .getEntry();
  private GenericEntry coralVoltage = ShuffleboardConstants.kCoralTab.add("Intake Voltage", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 1)
  .getEntry();

  public void intakeCoral() {
    isIntakingCoral = true;
    isEjectingCoral = false;
    m_intakeMotor.set(-1);
  }

  public void ejectCoral() {
    isEjectingCoral = true;
    isIntakingCoral = false;
    m_intakeMotor.set(1);
  }

  public void stopCoral() {
    isEjectingCoral = false;
    isIntakingCoral = false;
    m_intakeMotor.set(0);
  }

  public double getEncoderPosition() {
    return m_wristEncoder.getPosition();
  }

  public double getEncoderVelocity() {
    return m_intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristPosition.setDouble(getEncoderPosition());
    wristVelocity.setDouble(getEncoderVelocity());
    wristVoltage.setDouble(m_wristMotor.getOutputCurrent());
    coralVoltage.setDouble(m_intakeMotor.getOutputCurrent());
  }
}
