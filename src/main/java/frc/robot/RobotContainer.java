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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Shuffleboard.LimelightSubsystem;
import frc.robot.subsystems.Shuffleboard.NavXSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class RobotContainer {

  private XboxController m_controller = new XboxController(0);

  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();

  private NavXSubsystem navx_Data = new NavXSubsystem();
  private LimelightSubsystem m_limelight = new LimelightSubsystem();

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<String> pathfinderChooser;

  public RobotContainer() {
    configureBindings();
    navx_Data.init();

    NamedCommands.registerCommand("PRINT", new PrintCommand("Auto ended"));

    NamedCommands.registerCommand("ActivateIntake", new PrintCommand("Grabing Note")); // Intake
    NamedCommands.registerCommand("ActivateShooter", new PrintCommand("Shooting Note")); // Shooter 

    m_robotDrive.setDefaultCommand(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_controller.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_controller.getRightX(), OIConstants.kDriveDeadband), 
        !true), // Useless
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

    new JoystickButton(m_controller, XboxController.Button.kB.value)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.changeDriveMod(), 
        m_robotDrive));

    new JoystickButton(m_controller, XboxController.Button.kA.value)
      .whileTrue(new IntakeCommand(m_intakeSubsystem, m_shooterSubsystem));

    new JoystickButton(m_controller, XboxController.Button.kY.value)
    .whileFalse(new ShooterCommand(m_shooterSubsystem));

  }

  public Command getAutonomousCommand() {

    List<PathPlannerPath> auto = PathPlannerAuto.getPathGroupFromAutoFile(autoChooser.getSelected().getName());
    PathPlannerPath[] path = auto.toArray(new PathPlannerPath[0]);
    
    PathConstraints constraints = new PathConstraints(
        0.5, 1.0, 
        Units.degreesToRadians(540), Units.degreesToRadians(720)
    );

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        path[0].getPreviewStartingHolonomicPose(),  
        constraints,
        3.0
    );
 
    SequentialCommandGroup finalCommand = new SequentialCommandGroup(pathfindingCommand.andThen(autoChooser.getSelected()).withTimeout(15));

    m_limelight.toggleLED(true);
    return finalCommand;
    
  }
}
