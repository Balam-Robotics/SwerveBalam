// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Shuffleboard.LimelightSubsystem;

import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  private XboxController m_controller = new XboxController(OIConstants.kDriveControllerPort);

  private DriveSubsystem m_robotDrive = new DriveSubsystem();

  public LimelightSubsystem m_limelight = new LimelightSubsystem();

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();
    m_robotDrive.zeroHeading();
    m_limelight.init();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    NamedCommands.registerCommand("PRINT", new PrintCommand("Auto ended"));

    NamedCommands.registerCommand("ActivateIntake", new PrintCommand("Grabing Note")); // Intake
    NamedCommands.registerCommand("ActivateShooter", new PrintCommand("Shooting Note")); // Shooter 

    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive));

  }

  private void configureBindings() {

    new JoystickButton(m_controller, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(), 
            m_robotDrive));
        
    new JoystickButton(m_controller, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.zeroPose(), 
          m_robotDrive));
          
    new JoystickButton(m_controller, XboxController.Button.kB.value).onTrue(m_robotDrive.changeDriveModeCmd());

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

    SequentialCommandGroup finalCommand = new SequentialCommandGroup(pathfindingCommand.andThen(autoCommand));

    m_limelight.toggleLED(true);
    
    return finalCommand;
    
  }
}
