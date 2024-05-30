// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  private XboxController m_controller = new XboxController(0);
  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private NavX navx_Data = new NavX();

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<String> pathfinderChooser;

  public RobotContainer() {
    configureBindings();
    navx_Data.init();

    NamedCommands.registerCommand("PRINT", new PrintCommand("Auto ended"));

    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband), !true),
      m_robotDrive));

    autoChooser = AutoBuilder.buildAutoChooser();
    pathfinderChooser = new SendableChooser<>();

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    SmartDashboard.putData("Pathfinder Chooser", pathfinderChooser);
    pathfinderChooser.addOption("Testing Auto", "Testing Auto");
    pathfinderChooser.addOption("WIP", "Wip");

  }

  private void configureBindings() {

    new JoystickButton(m_controller, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(), 
            m_robotDrive));

    if(m_controller.getBButtonPressed()) {
      m_robotDrive.changeDriveMod();
    }

  }

  public Command getAutonomousCommand() {

    List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
    PathPlannerPath[] path = auto.toArray(new PathPlannerPath[0]);
    
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0, 
        Units.degreesToRadians(540), Units.degreesToRadians(720)
    );

    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
        path[0],  
        constraints,
        3.0
    );
 
    return pathfindingCommand.andThen(autoChooser.getSelected()).withTimeout(15);
    
  }
}
