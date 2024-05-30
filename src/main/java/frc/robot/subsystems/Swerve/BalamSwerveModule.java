// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class BalamSwerveModule {


  private CANSparkMax sm_drivingMotor;
  private CANSparkMax sm_turningMotor;

  private RelativeEncoder sm_drivingEncoder;
  private AbsoluteEncoder sm_turningEncoder;

  private SwerveModuleState sm_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  private SwerveModuleState sm_setpoints = new SwerveModuleState(0.0, new Rotation2d());

  private SparkPIDController sm_drivePIDController;
  private SparkPIDController sm_turningPIDController;

  private double m_chassisAngularOffset = 0;

  public BalamSwerveModule(int drivingId, int turningId, double chassisAngularOffset) {
    
    sm_drivingMotor = new CANSparkMax(drivingId, MotorType.kBrushless);
    sm_turningMotor = new CANSparkMax(turningId, MotorType.kBrushless);

    sm_drivingMotor.restoreFactoryDefaults();
    sm_turningMotor.restoreFactoryDefaults();

    sm_drivingEncoder = sm_drivingMotor.getEncoder();
    sm_turningEncoder = sm_turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

    // Pid

    sm_drivePIDController = sm_drivingMotor.getPIDController();
    sm_turningPIDController = sm_turningMotor.getPIDController();

    sm_drivePIDController.setFeedbackDevice(sm_drivingEncoder);
    sm_turningPIDController.setFeedbackDevice(sm_turningEncoder);

    sm_turningPIDController.setPositionPIDWrappingEnabled(true);
    sm_turningPIDController.setPositionPIDWrappingMinInput(0);
    sm_turningPIDController.setPositionPIDWrappingMaxInput(2 * Math.PI);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    sm_drivePIDController.setP(0.04);
    sm_drivePIDController.setI(0);
    sm_drivePIDController.setD(0);
    sm_drivePIDController.setOutputRange(-1,1);
    sm_drivePIDController.setFF(ModuleConstants.kDrivingFF);
    sm_drivePIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    sm_turningPIDController.setP(1);
    sm_turningPIDController.setI(0);
    sm_turningPIDController.setD(1);
    sm_turningPIDController.setOutputRange(-1,1);
    sm_turningPIDController.setFF(ModuleConstants.kTurningFF);
    sm_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    // Misc

    sm_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    sm_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    
    sm_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    sm_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    sm_turningEncoder.setInverted(true);

    // Other

    sm_drivingMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    sm_turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    sm_drivingMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    sm_turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    sm_drivingMotor.burnFlash();
    sm_turningMotor.burnFlash();

    // Final

    sm_desiredState.angle = new Rotation2d(sm_turningEncoder.getPosition());
    sm_drivingEncoder.setPosition(0);
    m_chassisAngularOffset = chassisAngularOffset;

  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(sm_drivingEncoder.getVelocity(), new Rotation2d(sm_turningEncoder.getPosition()- m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(sm_drivingEncoder.getPosition(), new Rotation2d(sm_turningEncoder.getPosition() - m_chassisAngularOffset ));
  }
  
  public SwerveModuleState getSetpoints() {
    return sm_setpoints;
  }


  public void setdesiredState(SwerveModuleState desiredState) {
    
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(sm_turningEncoder.getPosition()));
    
    // Stop if no input

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.05) {
      stop();
      return;
    }

    // Set points
    sm_drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    sm_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    sm_desiredState = desiredState;
    sm_setpoints = desiredState;

  }

  public void stop() {
    sm_drivingMotor.set(0);
    sm_turningMotor.set(0);
  }
}
