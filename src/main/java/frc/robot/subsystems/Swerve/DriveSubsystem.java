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

package frc.robot.subsystems.Swerve;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoAlignConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

/**
 * The DriveSubsystem class is responsible for controlling the swerve drive
 * system of the robot.
 * It manages the individual swerve modules, handles odometry, and provides
 * methods for driving the robot.
 * It also integrates with the NavX gyroscope for orientation and supports
 * field-oriented driving.
 * The class includes functionality for auto-alignment using a Limelight camera
 * and publishes relevant data to NetworkTables for monitoring.
 * It also includes integration with PathPlanner for autonomous path following.
 * The subsystem updates its state periodically and provides commands for
 * changing drive modes.
 * It is designed to be used with the WPILib command-based framework.
 * 
 * @author BALAM 3527
 * @version 1.24, 09/09/2025
 *
 *          Hours Consumed Coding This: ~56
 * 
 */

public class DriveSubsystem extends SubsystemBase {

  /**
   * Swerve Modules
   * 
   * Each swerve module is represented by an instance of the BalamSwerveModule
   * class.
   * The modules are initialized with their respective drive and turning motor
   * IDs,
   * as well as their chassis angular offsets.
   * 
   * The modules are named based on their position on the robot: front left, front
   * right,
   * back left, and back right.
   * 
   * These modules are responsible for controlling the individual wheel movements,
   * including driving and steering.
   * 
   * The configuration constants for each module are defined in the DriveConstants
   * class.
   */

  private final BalamSwerveModule m_frontLeft = new BalamSwerveModule(
      DriveConstants.frontLeftDriveId,
      DriveConstants.frontLeftTurningId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final BalamSwerveModule m_frontRight = new BalamSwerveModule(
      DriveConstants.frontRightDriveId,
      DriveConstants.frontRightTurningId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final BalamSwerveModule m_backLeft = new BalamSwerveModule(
      DriveConstants.backLeftDriveId,
      DriveConstants.backLeftTurningId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final BalamSwerveModule m_backRight = new BalamSwerveModule(
      DriveConstants.backRightDriveId,
      DriveConstants.backRightTurningId,
      DriveConstants.kBackRightChassisAngularOffset);

  /**
   * NetworkTables Publishers
   * 
   * These publishers are used to send data to NetworkTables for monitoring and
   * debugging purposes.
   * They publish the measured states of the swerve modules, the setpoints for the
   * swerve modules,
   * the robot's rotation, and the robot's pose.
   * 
   * They utilize the NetworkTableInstance to create topics for each type of data,
   * and they use the appropriate struct types for the data being published.
   * 
   * This allows for real-time monitoring of the robot's state during operation.
   * The published data can be viewed using tools like AdvantageScope or custom
   * dashboards.
   */

  private StructArrayPublisher<SwerveModuleState> publish_SwerveStates = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveModuleStates/Measured", SwerveModuleState.struct).publish();

  private StructArrayPublisher<SwerveModuleState> publish_SwerverSetpoints = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveModuleStates/Setpoints", SwerveModuleState.struct).publish();

  private StructPublisher<Rotation2d> publish_robotRotation = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/Robot2D", Rotation2d.struct).publish();

  private StructPublisher<Pose2d> publish_robotPose = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/RobotPose2D", Pose2d.struct).publish();

  final StructPublisher<Pose2d> publish_poseEstimator = NetworkTableInstance.getDefault()
      .getStructTopic("/Odometry/PoseEstimation", Pose2d.struct).publish();

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  {
    if (!RobotBase.isSimulation()) {
    var alliance = DriverStation.getAlliance();
    if (alliance.get() == DriverStation.Alliance.Blue) {
      m_gyro.setAngleAdjustment(180);
    }
  }
  }

  private ShuffleboardLayout gyroLayout = ShuffleboardConstants.kDriverTab
      .getLayout("Gyro Data", BuiltInLayouts.kList)
      .withSize(2, 5)
      .withPosition(6, 0);

  private boolean m_isFieldOriented = true;
  /*
   * {
   * ShuffleboardConstants.kDriverTab.add("Field Oriented", m_isFieldOriented)
   * .withWidget(BuiltInWidgets.kToggleButton)
   * .withSize(2, 1)
   * .withPosition(0, 0);
   * ShuffleboardConstants.kDriverTab.add("Field Oriented Boolean",
   * m_isFieldOriented)
   * .withWidget(BuiltInWidgets.kBooleanBox)
   * .withSize(1, 01)
   * .withPosition(0, 1);
   * }
   */

  public void changeDriveMode() {
    m_isFieldOriented = !m_isFieldOriented;
  }

  public Command changeDriveModeCmd() {
    return this.runOnce(() -> m_isFieldOriented = !m_isFieldOriented);
  }

  // Odometry

  // private Pose2d limelightPose2d =
  // LimelightHelpers.getBotPose2d_wpiBlue("limelight-balam"); //WIP

  /**
   * Odometry and Pose Estimation
   * 
   * These classes track the robot's position on the field.
   * They combine swerve module states and gyro data to produce
   * an estimate of the robot's current pose.
   * 
   * - {@link SwerveDriveOdometry} maintains pose purely from kinematics
   * and encoder/gyro measurements. It provides a baseline position
   * estimate without vision corrections.
   * 
   * - {@link SwerveDrivePoseEstimator} extends odometry by fusing
   * vision measurements (e.g., AprilTags, Limelight, cameras) with
   * kinematics. This produces a more accurate and drift-resistant
   * estimate of the robot's location.
   * 
   * Both are initialized with a starting {@link Pose2d}, which can be set
   * to match the robot's starting location on the field. These values
   * can be reset at any point (e.g., at the start of autonomous).
   */

  private SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      getHeading(),
      getSwerveModulePositions(),
      new Pose2d(1.21, 5.53, getHeading())); // new Pose2d(1.21, 5.53, getHeading()

  private SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getHeading(),
      getSwerveModulePositions(),
      new Pose2d(3.0, 7.0, getHeading()));

  private Field2d field, field2;

  // ----------------- Drive Subsystem Functions -----------------

  // Gyro Functions

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  // Odometry Functions

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation2d() {
    return m_poseEstimator.getEstimatedPosition().getRotation();
  }

  public void resetPose(Pose2d reseted) {
    m_odometry.resetPosition(
        getHeading(),
        getSwerveModulePositions(),
        reseted);
    if (visionPose() != null) {
      m_poseEstimator.resetPosition(
        getHeading(),
        getSwerveModulePositions(),
        visionPose());
    }
    
  }

  // Relative Robot DriveSubsystem for Pathplanner

  public ChassisSpeeds getRelativeChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
  }

  public void setChassisSpeed(ChassisSpeeds desired) {
    SwerveModuleState[] newStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(desired);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, 4.8);
    setDesiredStates(newStates);
  }

  public void stopChassis() {
    setChassisSpeed(new ChassisSpeeds(0, 0, 0));
  }

  // Relative and FieldOriented DriveSubsystem with Controller Inputs

  public void drive(double xSpeed, double ySpeed, double rot, boolean overrideFieldOriented) {

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    boolean fieldOriented;

    if (overrideFieldOriented) {
      fieldOriented = true;
    } else {
      fieldOriented = m_isFieldOriented;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setDesiredStates(swerveModuleStates);

  }

  // Swerve Common Functions

  public void setDesiredStates(SwerveModuleState[] swerveModuleStates) {
    m_frontLeft.setdesiredState(swerveModuleStates[0]);
    m_frontRight.setdesiredState(swerveModuleStates[1]);
    m_backLeft.setdesiredState(swerveModuleStates[2]);
    m_backRight.setdesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getSwerveModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
    };
  }

  public SwerveModuleState[] getSwerveModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    };
  }

  public SwerveModuleState[] getSwerveModuleSetpoints() {
    return new SwerveModuleState[] {
        m_frontLeft.getSetpoints(),
        m_frontRight.getSetpoints(),
        m_backLeft.getSetpoints(),
        m_backRight.getSetpoints()
    };
  }

  // -- Auto Align with PID Controller -- //

  PIDController alignPID_STRAFE = new PIDController(2.5, 0.05, 0.001);
  PIDController alignPID_FOWARD = new PIDController(2, 0, 0.1);
  PIDController alignPID_ROTATION = new PIDController(0.1, 0, 0);
  double LEFT_CORAL_OFFSET = AutoAlignConstants.LEFT_CORAL_OFFSET;
  double RIGHT_CORAL_OFFSET = AutoAlignConstants.RIGHT_CORAL_OFFSET;
  private GenericEntry LEFT_OFFSET, RIGHT_OFFSET;

  {
    LEFT_OFFSET = ShuffleboardConstants.kDebugTab.add("Left Coral Offset", LEFT_CORAL_OFFSET)
        .withSize(2, 1)
        .withPosition(0, 0)
        .withProperties(Map.of("show_submit_button", true))
        .getEntry();

    RIGHT_OFFSET = ShuffleboardConstants.kDebugTab.add("Right Coral Offset", RIGHT_CORAL_OFFSET)
        .withSize(2, 1)
        .withPosition(0, 1)
        .withProperties(Map.of("show_submit_button", true))
        .getEntry();
  }

  public ChassisSpeeds alignWithPID(Constants.Direction direction) {

    boolean tv = LimelightHelpers.getTV(CameraConstants.kLimelightName);
    if (!tv)
      return new ChassisSpeeds(0, 0, 0);

    double[] botpose = LimelightHelpers.getBotPose_TargetSpace(CameraConstants.kLimelightName);
    if (botpose == null || botpose.length < 2) {
      return new ChassisSpeeds(0, 0, 0);
    }

    double xMeters = botpose[0];
    double yMeters = botpose[2];
    double currentYaw = botpose[4];
    //System.out.printf("X: %s.2f, Y: %s.2f, Z: %s", xMeters, yMeters, currentYaw);

    double coralTargetX = 0.0;
    if (direction == Constants.Direction.RIGHT) {
      coralTargetX = OIConstants.kDebug ? RIGHT_OFFSET.getDouble(0.0) : RIGHT_CORAL_OFFSET;
    } else if (direction == Constants.Direction.LEFT) {
      coralTargetX = OIConstants.kDebug ? LEFT_OFFSET.getDouble(0.0) : LEFT_CORAL_OFFSET;
    } else if (direction == Constants.Direction.CENTER) {
      coralTargetX = 0.0;
    }

    double robotStrafeSetpoint = coralTargetX;
    alignPID_STRAFE.setSetpoint(robotStrafeSetpoint);

    double xSpeed = -alignPID_STRAFE.calculate(xMeters);
    xSpeed = MathUtil.clamp(xSpeed, -AutoAlignConstants.MAX_SPEED, AutoAlignConstants.MAX_SPEED);

    double ySpeed = -alignPID_FOWARD.calculate(-yMeters);
    ySpeed = MathUtil.clamp(ySpeed, -AutoAlignConstants.MAX_SPEED, AutoAlignConstants.MAX_SPEED);

    double rotationSpeed = -alignPID_ROTATION.calculate(currentYaw);
    rotationSpeed = MathUtil.clamp(rotationSpeed, -AutoAlignConstants.MAX_ROTATION_SPEED,
        AutoAlignConstants.MAX_ROTATION_SPEED);

    if (alignPID_STRAFE.atSetpoint())
      xSpeed = 0;
    if (alignPID_FOWARD.atSetpoint())
      ySpeed = 0;
    if (alignPID_ROTATION.atSetpoint())
      rotationSpeed = 0;

    if (Math.abs(xMeters) < 0.01)
      xSpeed = 0;
    if (Math.abs(yMeters - alignPID_FOWARD.getSetpoint()) < 0.02)
      ySpeed = 0;
    if (Math.abs(currentYaw) < 1.0)
      rotationSpeed = 0;

    return new ChassisSpeeds(ySpeed, xSpeed, rotationSpeed); // ySpeed, xSpeed, rotationSpeed
  }

  // Drive Subsystem Constructor and Periodic

  public DriveSubsystem() {

    // Align PID

    alignPID_STRAFE.reset();
    alignPID_STRAFE.setTolerance(0.01);
    alignPID_STRAFE.setSetpoint(0.0);

    alignPID_FOWARD.reset();
    alignPID_FOWARD.setTolerance(0.02);
    alignPID_FOWARD.setSetpoint(1.3); // Recommended 1.3 for more range

    alignPID_ROTATION.reset();
    alignPID_ROTATION.setTolerance(1.0);
    alignPID_ROTATION.setSetpoint(0.0);

    ShuffleboardConstants.kTestTab.add("Align Strafe PID", alignPID_STRAFE)
        .withWidget(BuiltInWidgets.kPIDController)
        .withSize(2, 3)
        .withPosition(0, 0);
    ShuffleboardConstants.kTestTab.add("Align Forward PID", alignPID_FOWARD)
        .withWidget(BuiltInWidgets.kPIDController)
        .withSize(2, 3)
        .withPosition(2, 0);
    ShuffleboardConstants.kTestTab.add("Align Rotation PID", alignPID_ROTATION)
        .withWidget(BuiltInWidgets.kPIDController)
        .withSize(2, 3)
        .withPosition(4, 0);

    // Elastic Test
    create_swerve_display(ShuffleboardConstants.kDriverTab, true);
    create_swerve_display(ShuffleboardConstants.kAutonomousTab, true);

    // Shuffleboard Gyro
    gyroLayout.addBoolean("Gyro Connected", () -> m_gyro.isConnected());
    gyroLayout.addBoolean("Gyro Calibrating", () -> m_gyro.isCalibrating()).withWidget(BuiltInWidgets.kBooleanBox)
        .withProperties(Map.of("colorWhenFalse", 0xffffcc00));
    // .withProperties(Map.of("Color when true", "4CAF50", "Color when false",
    // "#ffcc00"));
    // .withProperties(Map.of("false_color", "0xffffcc00"));
    gyroLayout.addBoolean("Field Oriented", () -> m_isFieldOriented);
    gyroLayout.add("Gyro Angle", m_gyro).withWidget(BuiltInWidgets.kGyro);

    // Shuffleboard 2D Field

    field = new Field2d();
    field2 = new Field2d();
    ShuffleboardConstants.kDriverTab.add("Field", field)
        .withSize(3, 2)
        .withPosition(8, 3);
        
    ShuffleboardConstants.kAutonomousTab.add("Autonomous Field", field2)
        .withSize(4, 3)
        .withPosition(6, 1);

    // Pathplanner trajectory for AdvantageScope

    PathPlannerLogging.setLogActivePathCallback((poses) -> {
      field.getObject("path").setPoses(poses);
    });

    // Pathplanner

    // RobotConfig config = new RobotConfig(0, 0, null, null);
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      config = new RobotConfig(0, 0, null, null);
    }

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRelativeChassisSpeeds,
        (speeds, feedforwards) -> setChassisSpeed(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(
                AutoConstants.kAutoTranslationP,
                AutoConstants.kAutoTranslationI,
                AutoConstants.kAutoTranslationD),
            new PIDConstants(
                AutoConstants.kAutoRotationP,
                AutoConstants.kAutoRotationI,
                AutoConstants.kAutoRotationD)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

  }

  public void create_swerve_display(ShuffleboardTab tab, boolean real) {
    tab.add("Swerve State", builder -> {
      builder.setSmartDashboardType("SwerveDrive");

      builder.addDoubleProperty(
          "Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
      builder.addDoubleProperty(
          "Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

      builder.addDoubleProperty(
          "Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
      builder.addDoubleProperty(
          "Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

      builder.addDoubleProperty(
          "Back Left Angle", () -> m_backLeft.getState().angle.getRadians(), null);
      builder.addDoubleProperty(
          "Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

      builder.addDoubleProperty(
          "Back Right Angle", () -> m_backRight.getState().angle.getRadians(), null);
      builder.addDoubleProperty(
          "Back Right Velocity", () -> m_backRight.getState().speedMetersPerSecond, null);

      builder.addDoubleProperty(
          "Robot Angle", () -> getHeading().getRadians(), null);
    }).withSize(2, 2).withPosition(4, 0)
        .withProperties(Map.of("show_robot_rotation", true, "rotation_unit", "radians"));

    if (!real) {
      tab.add("Swerve Visualizer", builder -> {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty(
            "Front Left Angle", () -> m_frontLeft.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty(
            "Front Left Velocity", () -> m_frontLeft.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty(
            "Front Right Angle", () -> m_frontRight.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty(
            "Front Right Velocity", () -> m_frontRight.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty(
            "Back Left Angle", () -> m_backLeft.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty(
            "Back Left Velocity", () -> m_backLeft.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty(
            "Back Right Angle", () -> m_backRight.getSetpoints().angle.getRadians(), null);
        builder.addDoubleProperty(
            "Back Right Velocity", () -> m_backRight.getSetpoints().speedMetersPerSecond, null);

        builder.addDoubleProperty(
            "Robot Angle", () -> getHeading().getRadians(), null);
      }).withSize(2, 2).withPosition(4, 2);
    }
  }

  private Pose2d visionPose() {
    boolean doRejectUpdate = false;
    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.kLimelightName);

    if (mt1 == null) {
      DriverStation.reportWarning("Limelight disconnected or not returning data", false);
      return null;
    }

    if (mt1.tagCount >= 2 && mt1.rawFiducials.length == 2) {
      if (mt1.rawFiducials[0].ambiguity > 0.7) doRejectUpdate = true;
      if (mt1.rawFiducials[0].distToCamera > 3) doRejectUpdate = true;
    }
    if (mt1.tagCount == 0) doRejectUpdate = true;

    if (!doRejectUpdate) {
      return new Pose2d(mt1.pose.getX(), mt1.pose.getY(), mt1.pose.getRotation());
    }

    return null;
  }

  private void updateVisionOdometry() {
    m_poseEstimator.update(m_gyro.getRotation2d(), getSwerveModulePositions());

    boolean useMegaTag2 = true;
    boolean doRejectUpdate = false;

    if (!useMegaTag2) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(CameraConstants.kLimelightName);

      if (mt1 == null) {
        DriverStation.reportWarning("Limelight disconnected or not returning data", false);
        return;
      }

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > 0.7) doRejectUpdate = true;
        if (mt1.rawFiducials[0].distToCamera > 3) doRejectUpdate = true;
      }
      if (mt1.tagCount == 0) doRejectUpdate = true;

      if (!doRejectUpdate) {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, .9999999));
        m_poseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
      }
    } else if (useMegaTag2) {
        LimelightHelpers.SetRobotOrientation(CameraConstants.kLimelightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(CameraConstants.kLimelightName);

        if (mt2 == null) {
          DriverStation.reportWarning("Limelight disconnected or not returning data", false);
          return;
        }

        if (Math.abs(m_gyro.getRate()) > 720) doRejectUpdate = true;
        if (mt2.tagCount == 0) doRejectUpdate = true;

        if (!doRejectUpdate && mt2.tagCount >= 1) {
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, .9999999)); // .6 if .7 dosnt work
          m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
    }
  }

  @Override
  public void periodic() {

    // Odometry Update

    if (!RobotBase.isSimulation()) {
      updateVisionOdometry();
    }

    field.setRobotPose(m_poseEstimator.getEstimatedPosition());
    field2.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // Publish Advantage Scope Data and Shuffleboard Data

    SwerveModuleState[] physicPoints = getSwerveModuleStates();
    SwerveModuleState[] setPoints = getSwerveModuleSetpoints();
/*
    publish_SwerveStates.set(physicPoints);
    publish_SwerverSetpoints.set(setPoints);
    publish_robotRotation.set(getRotation2d());
    publish_robotPose.set(getPose());
    publish_poseEstimator.set(m_poseEstimator.getEstimatedPosition());
 */
  }

}
