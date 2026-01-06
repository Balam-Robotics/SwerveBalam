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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.util.GameTimer;

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

    m_wristMotor = new SparkMax(CoralIntakeConstants.kWristMotorId, MotorType.kBrushless);
    m_intakeMotor = new SparkMax(CoralIntakeConstants.kIntakeMotorId, MotorType.kBrushless);

    wristMotorConfig
    
      .idleMode(CoralIntakeConstants.kWristIdleMode)
      .smartCurrentLimit(CoralIntakeConstants.kWristCurrentLimit);
    wristMotorConfig.closedLoop
      .pidf(CoralIntakeConstants.kWristPIDkP, CoralIntakeConstants.kWristPIDkI, CoralIntakeConstants.kWristPIDkD, 0);
    intakeMotorConfig
      .idleMode(CoralIntakeConstants.kIntakeIdleMode)
      .smartCurrentLimit(CoralIntakeConstants.kIntakeCurrentLimit);

      m_wristMotor.configure(wristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      m_intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      m_wristEncoder = m_wristMotor.getEncoder();
      m_wristEncoder.setPosition(0);

      m_intakeEncoder = m_intakeMotor.getEncoder();
      m_intakeEncoder.setPosition(0);
      test();
  }

  public void setWristAngle(double position) {
    System.out.println("Setting wrist angle to " + position);
    m_wristMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  GameTimer gameTimer = new GameTimer();
  PIDController shuffleboardPIDController = new PIDController(CoralIntakeConstants.kWristPIDkP, CoralIntakeConstants.kWristPIDkI, CoralIntakeConstants.kWristPIDkD);
  public void test() {
    ShuffleboardConstants.kCoralTab.add("Wirst PID Controller", shuffleboardPIDController)
    .withWidget(BuiltInWidgets.kPIDController)
    .withSize(2, 1);
  } 
  

  private GenericEntry shuffleBoardPos = ShuffleboardConstants.kDebugTab.add("Wrist PID", 0.0)
  .withWidget(BuiltInWidgets.kNumberSlider)
  .withSize(2,1)
  .withPosition(0, 3)
  .withProperties(Map.of("min_value", 0, "max_value", 7))
  .getEntry();
  private GenericEntry wristPosition = ShuffleboardConstants.kCoralTab.add("Wrist Angle", 0.0)
  .withWidget(BuiltInWidgets.kDial)
  .withPosition(2, 0)
  .withSize(2, 2) 
  .getEntry();
  private GenericEntry wristVelocity = ShuffleboardConstants.kCoralTab.add("Wrist Velocity", 0.0)
  .withWidget(BuiltInWidgets.kDial)
  .withPosition(3, 0)
  .withSize(2, 2)
  .getEntry();
  
  private GenericEntry ejectingCoralEntry = ShuffleboardConstants.kCoralTab.add("Ejecting Coral", false)
  .withWidget(BuiltInWidgets.kBooleanBox)
  .withPosition(4, 0)
  .withSize(1, 1)
  .getEntry();
  private GenericEntry intakingCoralEntry = ShuffleboardConstants.kCoralTab.add("Intaking Coral", false)
  .withWidget(BuiltInWidgets.kBooleanBox)
  .withPosition(5, 0)
  .withSize(1, 1)
  .getEntry();

  public void setShuffleboardPIDWrist() {
    m_wristMotor.getClosedLoopController().setReference(shuffleBoardPos.getDouble(0), ControlType.kPosition);
    //m_wristMotor.getClosedLoopController().setReference(test.getSetpoint(), ControlType.kPosition);
  }

  public void adjustWristAngle(double angleRadians) {
    System.out.println("Adjusting wrist angle by " + angleRadians);
    m_wristMotor.getEncoder().setPosition(getEncoderPosition() + angleRadians);
  }

  public void intakeCoral() {
    System.out.println("Intaking coral");
    isIntakingCoral = true;
    isEjectingCoral = false;
    m_intakeMotor.set(1);
  }

  public void ejectCoral() {
    System.out.println("Ejecting coral");
    isEjectingCoral = true;
    isIntakingCoral = false;
    m_intakeMotor.set(-1);
  }

  public void stopCoral() {
    System.out.println("Stopping coral intake");
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
    ejectingCoralEntry.setBoolean(isEjectingCoral);
    intakingCoralEntry.setBoolean(isIntakingCoral);
    wristPosition.setDouble(getEncoderPosition());
    wristVelocity.setDouble(getEncoderVelocity());

    //System.out.println(shuffleboardPIDController.getSetpoint());
  }
}
