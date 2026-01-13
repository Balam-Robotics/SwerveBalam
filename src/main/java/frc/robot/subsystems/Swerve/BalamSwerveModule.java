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

package frc.robot.subsystems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

/** Add your docs here. */
public class BalamSwerveModule {

  private SparkMax sm_drivingMotor;
  private SparkMax sm_turningMotor;

  private RelativeEncoder sm_drivingEncoder;
  private AbsoluteEncoder sm_turningEncoder;

  private SwerveModuleState sm_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private SwerveModuleState sm_setpoints = new SwerveModuleState(0.0, new Rotation2d());

  private SparkClosedLoopController sm_drivePIDController;
  private SparkClosedLoopController sm_turningPIDController;

  private double m_chassisAngularOffset = 0;

  public BalamSwerveModule(int drivingId, int turningId, double chassisAngularOffset) {

    // Swerve Module Spark Max setup by Rev Robotics

    sm_drivingMotor = new SparkMax(drivingId, MotorType.kBrushless);
    sm_turningMotor = new SparkMax(turningId, MotorType.kBrushless);

    // Encoders

    sm_drivingEncoder = sm_drivingMotor.getEncoder();
    sm_turningEncoder = sm_turningMotor.getAbsoluteEncoder();

    // PIDs

    sm_drivePIDController = sm_drivingMotor.getClosedLoopController();
    sm_turningPIDController = sm_turningMotor.getClosedLoopController();

    sm_drivingMotor.configure(Configs.BalamSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    sm_turningMotor.configure(Configs.BalamSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    sm_desiredState.angle = new Rotation2d(sm_turningEncoder.getPosition());
    sm_drivingEncoder.setPosition(0);
    m_chassisAngularOffset = chassisAngularOffset;

  }

  // Get the current state (Velocity and Position) of the current module

  public SwerveModuleState getState() {
    return new SwerveModuleState(sm_drivingEncoder.getVelocity(),
        new Rotation2d(sm_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // Get thec current position (Velocity and Position) of the current module

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(sm_drivingEncoder.getPosition(),
        new Rotation2d(sm_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  // Advantage Scope comparation between Module PID and Module State

  public SwerveModuleState getSetpoints() {
    return sm_setpoints;
  }

  // Change the current state of the module with @desiredState

  public void setdesiredState(SwerveModuleState desiredState) {

    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));
    
   correctedDesiredState.optimize(new Rotation2d(sm_turningEncoder.getPosition()));

    SwerveModuleState optimizedDesiredState = correctedDesiredState;
      
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.05) {
      stop();
      return;
    }

    sm_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
    sm_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

    sm_desiredState = desiredState;
    sm_setpoints = desiredState;

  }

  public void stop() {
    sm_drivingMotor.set(0);
    sm_turningMotor.set(0);
  }

}
