// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 




        :::::::::      :::     :::            :::       :::   :::          ::::::::  ::::::::::  ::::::::  ::::::::::: 
     :+:    :+:   :+: :+:   :+:          :+: :+:    :+:+: :+:+:        :+:    :+: :+:    :+: :+:    :+: :+:     :+:  
    +:+    +:+  +:+   +:+  +:+         +:+   +:+  +:+ +:+:+ +:+              +:+ +:+              +:+         +:+    
   +#++:++#+  +#++:++#++: +#+        +#++:++#++: +#+  +:+  +#+           +#++:  +#++:++#+      +#+          +#+      
  +#+    +#+ +#+     +#+ +#+        +#+     +#+ +#+       +#+              +#+        +#+   +#+           +#+        
 #+#    #+# #+#     #+# #+#        #+#     #+# #+#       #+#       #+#    #+# #+#    #+#  #+#           #+#          
#########  ###     ### ########## ###     ### ###       ###        ########   ########  ##########     ###   
  




   */

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.Configs;

public class AlgaeSubsystem extends SubsystemBase {

  private SparkMax m_PrimaryIntakeMotor;
  private SparkMax m_SecondaryIntakeMotor;

  private SparkMax m_PrimaryAlgaeWristMotor; 
  private SparkMax m_SecondaryAlgaeWristMotor; 

  private RelativeEncoder m_encoder;

  public boolean isWristMoving = false;

  public boolean isEjectingAlgae = false;
  public boolean isIntakingAlgae = false;

  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {

    m_PrimaryIntakeMotor = new SparkMax(AlgaeConstants.kPrimaryIntakeMotorId, MotorType.kBrushless);
    m_SecondaryIntakeMotor = new SparkMax(AlgaeConstants.kSecondaryIntakeMotorId, MotorType.kBrushless);
    m_PrimaryAlgaeWristMotor = new SparkMax(AlgaeConstants.kPrimaryWristMotorId, MotorType.kBrushless);
    m_SecondaryAlgaeWristMotor = new SparkMax(AlgaeConstants.kSecondaryWristMotorId, MotorType.kBrushless); 

    m_PrimaryIntakeMotor.configure(Configs.AlgaeConfig.primaryIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_SecondaryIntakeMotor.configure(Configs.AlgaeConfig.secondaryIntakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_PrimaryAlgaeWristMotor.configure(Configs.AlgaeConfig.primaryAlgaeWristMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_SecondaryAlgaeWristMotor.configure(Configs.AlgaeConfig.secondaryAlgaeWristMotorCongif, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    m_encoder = m_PrimaryAlgaeWristMotor.getEncoder();

  }

  public void setWristAngle(double position) {
    m_PrimaryAlgaeWristMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public void setDEBUGWristAngle() {
    m_PrimaryAlgaeWristMotor.getClosedLoopController().setReference(algaePIDEntry.getDouble(0.0), ControlType.kPosition);
  }

  public void setWristSpeed(double speed) {
    // TODO: TEST LIMITS SUPPORT
    m_PrimaryIntakeMotor.set(speed);
  }

  public void stopWrist() {
    m_PrimaryIntakeMotor.set(0);
  }

  private GenericEntry algaePIDEntry = ShuffleboardConstants.kAlgaeTab.add("Algae PID", 0.0)
  .withSize(2, 1)
  .withPosition(2, 1)
  .withProperties(Map.of("show_submit_button", true))
  .getEntry();
  private GenericEntry algaePosition = ShuffleboardConstants.kAlgaeTab.add("Algae Wrist Angle", 0.0)
  .withWidget(BuiltInWidgets.kGraph)
  .withSize(2, 2) 
  .withPosition(0, 0)
  .getEntry();
  private GenericEntry algaeVelocity = ShuffleboardConstants.kAlgaeTab.add("Algae Wrist Velocity", 0.0)
  .withWidget(BuiltInWidgets.kAccelerometer)
  .withSize(2, 1)
  .withPosition(2, 0) 
  .getEntry();
  

  private ShuffleboardLayout valuesLayout = ShuffleboardConstants.kAlgaeTab
  .getLayout("Algae Booleans", BuiltInLayouts.kList)
  .withSize(2, 3)
  .withPosition(6, 0);

  {
    valuesLayout.addBoolean("Ejecting Algae", () -> isEjectingAlgae);
    valuesLayout.addBoolean("Intaking Algae", () -> isIntakingAlgae);
  }
 

  private GenericEntry wristVoltage = ShuffleboardConstants.kAlgaeTab.add("Wrist Voltage", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 0)
  .getEntry();
  private GenericEntry algaeVoltage = ShuffleboardConstants.kAlgaeTab.add("Intake Voltage", 0.0)
  .withWidget(BuiltInWidgets.kVoltageView)
  .withSize(2, 1)
  .withPosition(4, 1)
  .getEntry();

  public void intakeAlgae() {
    isIntakingAlgae = true;
    isEjectingAlgae = false;
    m_PrimaryIntakeMotor.set(1);
  }

  public void ejectAlgae() {
    isEjectingAlgae = true;
    isIntakingAlgae = false;
    m_PrimaryIntakeMotor.set(-1);
  }

  public void stopIntake() {
    isEjectingAlgae = false;
    isIntakingAlgae = false;
    m_PrimaryIntakeMotor.set(0);
  }
  
  public double getEncoderPosition() {
    return m_encoder.getPosition();
  }

  public double getEncoderVelocity() {
    return m_encoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    algaePosition.setDouble(getEncoderPosition());
    algaeVelocity.setDouble(getEncoderVelocity());
    wristVoltage.setDouble(m_PrimaryAlgaeWristMotor.getOutputCurrent());
    algaeVoltage.setDouble(m_PrimaryIntakeMotor.getOutputCurrent());
  }
}




