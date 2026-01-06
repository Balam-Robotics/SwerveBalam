// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Clase que maneja todas las constantes del robot
 * Configuracion del robot
 * 
 * @author BALAM 3527
 * @version 1.7, 09/09/2025
 * 
 */

public final class Constants {

    /**
     * Clase que maneja todas las constantes del sistema de swerve
     * Configuracion del sistema de swerve
     * @author REV Robotics
     */

    public static final class DriveConstants {

        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        public static final double kTrackWidth = Units.inchesToMeters(26.5);
        public static final double kWheelBase = Units.inchesToMeters(32);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARX MAX CAN ID

        public static final int frontLeftDriveId = 32;
        public static final int frontLeftTurningId = 31;
        public static final int frontRightDriveId = 22;
        public static final int frontRightTurningId = 21;

        public static final int backLeftDriveId = 2;
        public static final int backLeftTurningId = 1;
        public static final int backRightDriveId = 12;
        public static final int backRightTurningId = 11;

    }

    /**
     * Clase que maneja todas las constantes de cada modulo individual del Swerve
     * Configuracion de cada modulo del swerve
     * @author REV Robotics
     */

    public static final class ModuleConstants {

        public static final int kDrivingMotorPinionTeeth = 14;

        public static final double kDrivingMotorFreeSpeedRps = 5676 / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction; // meters
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
                / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps

    }

    /**
     * Clase que maneja todas las constantes del sistema de control
     * Configuracion del sistema de control
     * @author BALAM 3527
     */

    public static final class OIConstants {
        public static final boolean kDebug = false;
        public static final boolean kDemo = false;
        public static final boolean kOneDriver = false;

        public static final int kDriveControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kDriveDeadband = 0.2; // default : 0.2
    }

    /**
     * Clase que maneja todas las constantes del autonomo
     * Configuracion del autonomo
     * @author BALAM 3527 & REV Robotics
     */

    public static final class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 0.5;
        public static final double kMaxAccelerationMetersPerSecond = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // Units.degreesToRadians(540)
        public static final double kMaxAngularAccelerationRadiansPerSecond = Math.PI; // Units.degreesToRadians(720)

        public static final double kAutoTranslationP = 2.5;
        public static final double kAutoTranslationI = 0;
        public static final double kAutoTranslationD = 0;

        public static final double kAutoRotationP = 1.5;
        public static final double kAutoRotationI = 0;
        public static final double kAutoRotationD = 0;
        
        public static final double kMaxModuleSpeed = 4.8; // Max speed for each module
        public static final double kDriveBaseRadius = 0.46; // Distance from robot center to one module

    }

    /**
     * Clase que maneja todas las constantes del Limelight y las AprilTags
     * Configuracion del Limelight y Camaras
     * @author BALAM 3527
     */

    public static final class CameraConstants {
        public static final String kLimelightName = "limelight-balam";
        public static final int[] kValidAprilTagIds = {4};
    }

    /**
     * Clase que maneja todas las constantes del Shuffleboard
     * Configuracion del Shuffleboard
     * @author BALAM 3527
     */

    public static final class ShuffleboardConstants {
        public static final ShuffleboardTab kSwerveTab = Shuffleboard.getTab("Swerve");
        public static final ShuffleboardTab kCoralTab = Shuffleboard.getTab("Coral");
        public static final ShuffleboardTab kElevatorTab = Shuffleboard.getTab("Elevator");
        public static final ShuffleboardTab kClimberTab = Shuffleboard.getTab("Climber");
        public static final ShuffleboardTab kAutoTab = Shuffleboard.getTab("Swerve/Auto");
        public static final ShuffleboardTab kCameraTab = Shuffleboard.getTab("Swerve/Camera");
        public static final ShuffleboardTab kSpecialTab = Shuffleboard.getTab("Swerve/Special");
        public static final ShuffleboardTab kDriverTab = Shuffleboard.getTab("Driver");
        public static final ShuffleboardTab kOperatorTab = Shuffleboard.getTab("Swerve/Operator");
        public static final ShuffleboardTab kDebugTab = Shuffleboard.getTab("Debug");
        public static final ShuffleboardTab kTestTab = Shuffleboard.getTab("Test");
    }

    /**
     * Clase que maneja todas las constantes del sistema de elevador
     * Configuracion del sistema de elevador
     * @author BALAM 3527
     */

    public static final class ElevatorConstants {
        public static final int kPrimaryElevatorMotorId = 51; // Left Motor looking from the battery
        public static final int kSecondaryElevatorMotorId = 52; // Right Motor looking from the battery

        public static final int kPrimaryCurrentLimit = 20;
        public static final int kSecondaryCurrentLimit = 20;

        public static final IdleMode kPrimaryIdleMode = IdleMode.kBrake;
        public static final IdleMode kSecondaryIdleMode = IdleMode.kBrake;

        public static final int kMaxHeight = -70;
        public static final int kMinHeight = 0;
    }

    /**
     * Clase que maneja todas las constantes del sistema de intake
     * Configuracion del sistema de intake
     * @author BALAM 3527
     */

    public static final class CoralIntakeConstants {
        public static final int kWristMotorId = 41;
        public static final int kIntakeMotorId = 42;

        public static final int kWristCurrentLimit = 40;
        public static final int kIntakeCurrentLimit = 15;

        public static final IdleMode kWristIdleMode = IdleMode.kBrake;
        public static final IdleMode kIntakeIdleMode = IdleMode.kBrake;

        public static final double kWristMaxPosition = 0;
        public static final double kWristMinPosition = 0;

        public static final double kWristPIDkP = 0.05;
        public static final double kWristPIDkI = 0;
        public static final double kWristPIDkD = 0;
    }

    /**
     * Clase que maneja todas las constantes del sistema de escalada
     * Configuracion del sistema de escalada
     * @author BALAM 3527
     */

    public static final class ClimberConstants {
        public static final int kPrimaryMotorId = 3;
        public static final int kSecondaryMotorId = 4;

        public static final int kPrimaryCurrentLimit = 40;
        public static final int kSecondaryCurrentLimit = 40;

        public static final IdleMode kPrimaryIdleMode = IdleMode.kBrake;
        public static final IdleMode kSecondaryIdleMode = IdleMode.kBrake;

        public static final double kMaxEncoderPosition = 0;
        public static final double kMinEncoderPosition = 0;
    }

    /**
     * Clase que maneja todas las constantes de configuracion del robot
     * Configuracion de constantes especiales
     * @author BALAM 3527
     */

    public static final class SpecialConstants {
        public static final double SOURCE_HEIGHT = -20;
        public static final double L1_HEIGHT = 0;
        public static final double L2_HEIGHT = -25;
        public static final double L3_HEIGHT = -55;
        
        public static final double DEFAULT_ANGLE = 0.5;
        public static final double SOURCE_ANGLE = 1.5;
        public static final double L1_ANGLE = 4;
        public static final double L2_ANGLE = 4;
        public static final double L3_ANGLE = 4;

    }

    public static final class AutoAlignConstants {
        public static final double RIGHT_CORAL_OFFSET = 0.25;
        public static final double LEFT_CORAL_OFFSET = -0.2;

        public static final double MAX_SPEED = 1;
        public static final double MAX_ROTATION_SPEED = 0.5;
    }

    /**
     * Enum to represent LEFT and RIGHT directions
     */
    public enum Direction {
        LEFT,
        RIGHT,
        CENTER
    }

}
