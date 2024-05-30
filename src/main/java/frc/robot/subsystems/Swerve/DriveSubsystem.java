// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShuffleboardConstants;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveSubsystem extends SubsystemBase {

  // Create each swerve module

  private final BalamSwerveModule m_frontLeft = new BalamSwerveModule(
      ModuleConstants.frontLeftDriveId,
      ModuleConstants.frontLeftTurningId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final BalamSwerveModule m_frontRight = new BalamSwerveModule(
      ModuleConstants.frontRightDriveId,
      ModuleConstants.frontRightTurningId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final BalamSwerveModule m_backLeft = new BalamSwerveModule(
      ModuleConstants.backLeftDriveId,
      ModuleConstants.backLeftTurningId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final BalamSwerveModule m_backRight = new BalamSwerveModule(
      ModuleConstants.backRightDriveId,
      ModuleConstants.backRightTurningId,
      DriveConstants.kBackRightChassisAngularOffset);

  // Advantage Scope

  public StructArrayPublisher<SwerveModuleState> publish_SwerveStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveModuleStates/Measured", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> publish_SwerverSetpoints = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveModuleStates/Setpoints", SwerveModuleState.struct).publish();

  StructPublisher<Rotation2d> publish_robotRotation = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/Robot2D", Rotation2d.struct).publish();
  StructPublisher<Pose2d> publish_robotPose = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/RobotPose2D", Pose2d.struct).publish();
  StructPublisher<Pose2d> publish_poseEstimator = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/PoseEstimation", Pose2d.struct).publish();

  // NavX Gyroscope

  AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  public boolean isFieldOriented = true;

  public boolean changeDriveMod() {
    
    return !isFieldOriented;
  }

  // Odometry

  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics, m_gyro.getRotation2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      }, new Pose2d(5.0, 5.0, m_gyro.getRotation2d()));

  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, 
  getRotation2d(),
  new SwerveModulePosition[] {m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()},
  new Pose2d(0, 0, getRotation2d())
  );

  private Field2d field;

// Drive Subsystem Functions

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Rotation2d getRotation2d() {
    return m_odometry.getPoseMeters().getRotation();
  }

  public void resetPose(Pose2d reseted) {
    m_odometry.resetPosition(
        getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, reseted);
    poseEstimator.resetPosition(getRotation2d(), 
    new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
    }, reseted);
  }

  // Reset the robots front

  public void zeroHeading() {
    m_gyro.reset();
  }

  public ChassisSpeeds getRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
        m_backLeft.getState(), m_backRight.getState());
  }

  // Relative Robot DriveSubsystem for Pathplanner

  public void setChassisSpeed(ChassisSpeeds desired) {
    SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(desired);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, 4.8);
    m_frontLeft.setdesiredState(newStates[0]);
    m_frontRight.setdesiredState(newStates[1]);
    m_backLeft.setdesiredState(newStates[2]);
    m_backRight.setdesiredState(newStates[3]);
  }

  // Relative and FieldOriented DriveSubsystem with Controller Inputs

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldOriented) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    fieldOriented = isFieldOriented;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setdesiredState(swerveModuleStates[0]);
    m_frontRight.setdesiredState(swerveModuleStates[1]);
    m_backLeft.setdesiredState(swerveModuleStates[2]);
    m_backRight.setdesiredState(swerveModuleStates[3]);
  }

  // Drive Subsystem Constructor and Periodic

  public DriveSubsystem() {

    field = new Field2d();
    ShuffleboardConstants.SwerveTab.add("Field", field);

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });

    AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetPose, 
            this::getRelativeChassisSpeeds, 
            this::setChassisSpeed, 
            new HolonomicPathFollowerConfig(
                new PIDConstants(2.6, 0, 0), 
                new PIDConstants(2, 0, 0), 
                4.8,
                0.46,
                new ReplanningConfig()
            ), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Blue;
                }
                return false;
            },
            this);

  }

  @Override
  public void periodic() {

    m_odometry.update(m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
    poseEstimator.update(getRotation2d(), new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });
        field.setRobotPose(m_odometry.getPoseMeters());

    SwerveModuleState[] physicPoints = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState(),
    };

    SwerveModuleState[] setPoints = new SwerveModuleState[] {
        m_frontLeft.getSetpoints(),
        m_frontRight.getSetpoints(),
        m_backLeft.getSetpoints(),
        m_backRight.getSetpoints()
    };

    publish_SwerveStates.set(physicPoints);
    publish_SwerverSetpoints.set(setPoints);
    publish_robotRotation.set(getRotation2d());
    publish_robotPose.set(getPose());
    publish_poseEstimator.set(poseEstimator.getEstimatedPosition());

  }

}
