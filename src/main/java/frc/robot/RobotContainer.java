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

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoLimelightCommand;
import frc.robot.subsystems.Shuffleboard.LimelightSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  // Drive Controller

  private XboxController m_controller = new XboxController(OIConstants.kDriveControllerPort);

  private JoystickButton xButton = new JoystickButton(m_controller, XboxController.Button.kX.value);
  private JoystickButton yButton = new JoystickButton(m_controller, XboxController.Button.kY.value);
  private JoystickButton bButton = new JoystickButton(m_controller, XboxController.Button.kB.value);
  private JoystickButton aButton = new JoystickButton(m_controller, XboxController.Button.kA.value);

  // Subsystems

  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  public LimelightSubsystem m_limelight = new LimelightSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();    
    registedCommands();

    m_robotDrive.zeroHeading();
    m_limelight.init();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));

  }

  private void configureBindings() {

    xButton.whileTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
        
    yButton.whileTrue(new AutoLimelightCommand(m_robotDrive, LimelightConstants.kLimelightName));

    aButton.whileTrue(new RunCommand(() -> m_robotDrive.zeroPose(), m_robotDrive));
          
    bButton.onTrue(m_robotDrive.changeDriveModeCmd());

  } 

  private void registedCommands() {
    NamedCommands.registerCommand("PRINT", new PrintCommand("Auto ended"));
    NamedCommands.registerCommand("LimelightAmp", new AutoLimelightCommand(m_robotDrive, LimelightConstants.kLimelightName).withTimeout(1.50));
  }

  public SequentialCommandGroup getAutonomousCommand() {
    
    List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
    PathPlannerPath[] path = auto.toArray(new PathPlannerPath[0]);
    
    PathConstraints constraints = new PathConstraints(
        AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecond, 
        AutoConstants.kMaxAngularSpeedRadiansPerSecond, AutoConstants.kMaxAngularAccelerationRadiansPerSecond
    );

    Command pathfinderConstructor = AutoBuilder.pathfindToPose(
        path[0].getPreviewStartingHolonomicPose(),  
        constraints,
        3.0
    );  

    Command pathfindingCommand = CommandUtil.wrappedEventCommand(pathfinderConstructor);
    Command autoCommand = CommandUtil.wrappedEventCommand(autoChooser.getSelected());

    var alliance = DriverStation.getAlliance();
    boolean flipPath = false;
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      flipPath = true;
    } else if (alliance.get() == DriverStation.Alliance.Blue) {
      flipPath = false;
    }

    SequentialCommandGroup finalCommand;

    if (flipPath) {
      finalCommand = new SequentialCommandGroup(autoCommand);
    } else {
      finalCommand = new SequentialCommandGroup(pathfindingCommand.andThen(autoCommand));
    }
    
    return finalCommand;
    
  }
}
