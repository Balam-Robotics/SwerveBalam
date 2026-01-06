// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*

        

 888888ba   .d888888  dP         .d888888  8888ba.88ba     d8888b. 888888P d8888b. d88888P 
 88    `8b d8'    88  88        d8'    88  88  `8b  `8b        `88 88'         `88     d8' 
a88aaaa8P' 88aaaaa88a 88        88aaaaa88a 88   88   88     aaad8' 88baaa. .aaadP'    d8'  
 88   `8b. 88     88  88        88     88  88   88   88        `88     `88 88'       d8'   
 88    .88 88     88  88        88     88  88   88   88        .88      88 88.      d8'    
 88888888P 88     88  88888888P 88     88  dP   dP   dP    d88888P d88888P Y88888P d8'     
                                                                                           
                                                                                           

*/     

package frc.robot;


import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.Constants.SpecialConstants;
import frc.robot.commands.AutoEjectCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.util.CameraSystem;
import frc.robot.util.GameTimer;
import frc.robot.util.TejuinoBoard;

/**
 * Clase que maneja todas las funciones del robot
 * Cerebro del Robot
 * 
 * @author Balam 3527
 * @version 1.14, 09/09/2025
 *
 * Dolor de cabeza de programar y hacer documentacion 
 * 
 */

public class RobotContainer {

  /**
   * 
   * Controladores para manejar el robot
   * @param m_driverController Control que manjera el Swerve y Climber
   * @param m_operatorController Control que manjera los subsistemas como Elevador y Garra
   * 
   */

  private CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDebug ? OIConstants.kOperatorControllerPort : OIConstants.kDriveControllerPort);
  private CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kDebug ? OIConstants.kDriveControllerPort : OIConstants.kOperatorControllerPort);
  

  /**
   * 
   * Subsistemas del Robot
   * @param m_robotDrive Subsistema del Swerve y sus controladores
   * @param m_elevatorSubsystem Subsistema del elevador
   * @param m_coralSubsystem Subsistema del manipulador del Coral
   * @param m_climberSubsystem Subsistema del climber 
   * 
   * @param m_cameraSystem Subsistema del sistema de camara del robot
   * 
   */

  private DriveSubsystem m_robotDrive = new DriveSubsystem();
  private ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  @SuppressWarnings("unused")
  //private CameraSystem m_cameraSystem = new CameraSystem();

  /**
   * @param autoChoose Variable para seleccionar Autonomo durante modo autonomo 
   */

  private final SendableChooser<Command> autoChooser;

  // Controlador LED
  public final TejuinoBoard tejuino_board = new TejuinoBoard();

  /**
   * 
   * Crea un nuevo RobotContainer que contiene todo lo necesario del robot
   * 
   */

  public RobotContainer() {

    // -- Set Up -- //

    if (OIConstants.kOneDriver) {
      m_operatorController = m_driverController;
    }

    configureBindings();  // Configurar botones de Driver y Operador 
    registedCommands(); // Registar comandos para el modo autonomo
    setupElastic(); // Inicializar la pantalla del Driver en Elastic

    m_robotDrive.zeroHeading(); // Reiniciar gyroscopio

    autoChooser = AutoBuilder.buildAutoChooser(); // Inicializar la variable autoChooser
    ShuffleboardConstants.kDriverTab.add("Autonomous Selector", autoChooser)
    .withSize(3,1)
    .withPosition(8, 0);

    tejuino_board.init(0); // Inicializar controlador LED
    tejuino_board.all_leds_red(0); // Prender LEDs en rojo al iniciar el robot
    tejuino_board.all_leds_blue(tejuino_board.LED_STRIP_1);

    /**
     * 
     * Funcion que manda los inputs del control m_driverController al subsistema m_robotDrive para que el robot se mueva en Modo Teleoperado
     * 
     */

    m_robotDrive.setDefaultCommand(Commands.runOnce(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), 
        false), 
      m_robotDrive).withName("Swerve Drive Command"));
 
  }

  /**
   * 
   * Inicializar los Comandos y Grupos de Comandos del Autonomo y Teleoperado
   *
   * 
   * Comandos para el Elevador
   * 
   * @param Command -> Comandos individuales que realizan una sola accion
   * @param ParallelCommandGroup -> Multiples comandos al mismo tiempo para realizar mas de una accion al mismo tiempo, eg: Subir el Elevador y el Manipulador en posicion de Intake
   * 
   */ 

  Command liftToSourceCommand = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.SOURCE_HEIGHT), m_elevatorSubsystem).withName("Lift to Source Command");
  Command wristToSourceCommand = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.SOURCE_ANGLE), m_coralSubsystem).withName("Wrist to Source Command");
  ParallelCommandGroup sourceCommandGroup = new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);

  // L1 Commands

  Command liftToL1Command = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L1_HEIGHT), m_elevatorSubsystem).withName("Lift to L1 Command");
  Command wristToL1Command = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L1_ANGLE), m_coralSubsystem).withName("Wrist to L1 Command");
  ParallelCommandGroup l1CommandGroup = new ParallelCommandGroup(liftToL1Command, wristToL1Command);  
  
  // L2 Commands

  Command liftToL2Command = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L2_HEIGHT), m_elevatorSubsystem).withName("Lift to L2 Command");
  Command wristToL2Command = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L2_ANGLE), m_coralSubsystem).withName("Wrist to L2 Command");
  ParallelCommandGroup l2CommandGroup = new ParallelCommandGroup(liftToL2Command, wristToL2Command);  
  
  // L3 Commands

  Command liftToL3Command = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(SpecialConstants.L3_HEIGHT), m_elevatorSubsystem).withName("Lift to L3 Command");
  Command wristToL3Command = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.L3_ANGLE), m_coralSubsystem).withName("Wrist to L3 Command");
  ParallelCommandGroup l3CommandGroup = new ParallelCommandGroup(liftToL3Command, wristToL3Command);  

  // Reset Elevator Position 

  Command resetElevatorCommand = Commands.runOnce(() -> m_elevatorSubsystem.setElevatorPosition(0), m_elevatorSubsystem).withName("Reset Elevator Command");
  Command resetWristCommand = Commands.runOnce(() -> m_coralSubsystem.setWristAngle(SpecialConstants.DEFAULT_ANGLE), m_coralSubsystem).withName("Reset Wrist Command");
  ParallelCommandGroup resetCommandGroup = new ParallelCommandGroup(resetElevatorCommand, resetWristCommand);

  // Manual Lift
  Command manualLiftCommand = new RunCommand(() -> m_elevatorSubsystem.setElevatorSpeed(MathUtil.applyDeadband(m_operatorController.getRightY(), OIConstants.kDriveDeadband) * 0.5), m_elevatorSubsystem).withName("Manual Lift Command");
  Command stopManualLiftCommand = new RunCommand(() -> m_elevatorSubsystem.stopElevator(), m_elevatorSubsystem).withName("Stop Manual Lift Command");

  // PID Controllers
  Command pidLiftCommand = new RunCommand(() -> m_elevatorSubsystem.setShuffleboardPIDElevator(), m_elevatorSubsystem).withName("PID Elevator Command");
  Command pidWristCommand = new RunCommand(() -> m_coralSubsystem.setShuffleboardPIDWrist(), m_coralSubsystem).withName("PID Wrist Command");

  // Climber Commands

  Command climbUpCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(0.3), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);
  Command climbDownCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(-0.3), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);
  Command climbHoldCommand = new StartEndCommand(() -> m_climberSubsystem.setClimberSpeed(0.1), () -> m_climberSubsystem.stopClimber(), m_climberSubsystem);

  // Intake Commands

  Command intakeCoralCommand = new StartEndCommand(() -> m_coralSubsystem.intakeCoral(), () -> m_coralSubsystem.stopCoral(), m_coralSubsystem).withName("Intake Coral Command");
  Command ejectCoralCommand = new StartEndCommand(() -> m_coralSubsystem.ejectCoral(), () -> m_coralSubsystem.stopCoral(), m_coralSubsystem).withName("Eject Coral Command");  

  // Auto-Align Commands
  // TODO : TERMIANR COMANDOS PARA ALINEAR AUTOMATICAMENTE

  //Command leftAutoAlignCommand = new StartEndCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.autoAlign("left")), () -> m_robotDrive.setChassisSpeed(new ChassisSpeeds(0, 0, 0)), m_robotDrive);
  //Command rightAutoAlignCommand = new StartEndCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.autoAlign("right")), () -> m_robotDrive.setChassisSpeed(new ChassisSpeeds(0, 0, 0)), m_robotDrive);

  Command leftAutoAlightCommand = new RunCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.alignWithPID(Constants.Direction.LEFT)), m_robotDrive).withName("Left Coral Auto Align");
  Command rightAutoAlightCommand = new RunCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.alignWithPID(Constants.Direction.RIGHT)), m_robotDrive).withName("Right Coral Auto Align");
  Command centerAutoAlignCommand = new RunCommand(() -> m_robotDrive.setChassisSpeed(m_robotDrive.alignWithPID(Constants.Direction.CENTER)), m_robotDrive).withName("Center Coral Auto Align");

  Command moveForwardCommand = new RunCommand(() -> m_robotDrive.setChassisSpeed(new ChassisSpeeds(0.3, 0, 0)), m_robotDrive);

  //Pathplaner commands

  Command resetGyroCommand = Commands.runOnce(() -> m_robotDrive.zeroHeading(), m_robotDrive);

  /**
   * 
   * Confiugrar botones para el Driver y Operador
   * 
   */

  private void configureBindings() {

    //  --- Drive Controller Bindings --- //

    m_driverController.x().whileTrue(Commands.runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.b().onTrue(m_robotDrive.changeDriveModeCmd());

    m_driverController.povLeft().whileTrue(leftAutoAlightCommand);
    m_driverController.povRight().whileTrue(rightAutoAlightCommand);

    //m_driverController.povUp().whileTrue(climbUpCommand); // Move the climber up with the cross Up @DRIVER
    m_driverController.povUp().whileTrue(moveForwardCommand);
    m_driverController.povDown().whileTrue(climbDownCommand); // Move the climber down with the Cross Down @DRIVER
    //m_driverController.povLeft().whileTrue(climbHoldCommand); @deprecrated

    // --- Operator Controller Bindings --- //

    m_operatorController.leftBumper().whileTrue(ejectCoralCommand); // Eject coral with Left Bumper @OPERATOR
    m_operatorController.leftTrigger().whileTrue(intakeCoralCommand); // Intake coral with Left Trigger @OPERATOR
   
    m_operatorController.y().onTrue(l3CommandGroup); // Mover elevador y manipulador  en posicion del Nivel 3 del Arecife @OPERATOR
    m_operatorController.b().onTrue(l2CommandGroup); // Mover elevador y manipulador  en posicion del Nivel 2 del Arecife @OPERATOR
    m_operatorController.a().onTrue(l1CommandGroup); // Mover elevador y manipulador en posicion del Nivel 1 del Arecife @OPERATOR
    m_operatorController.x().onTrue(resetCommandGroup); // Resetear el elevador y manipulador @OPERATOR
    m_operatorController.povLeft().onTrue(sourceCommandGroup); // Source Command
    m_operatorController.povRight().onTrue(resetCommandGroup); // Reset

    m_operatorController.back().whileTrue(manualLiftCommand); // Iniciar modo manual del elevador @OPERATOR
    m_operatorController.back().whileFalse(stopManualLiftCommand); // Desactivar modo manual del elevador @OPERATOR
  
    /**
     * 
     * Modo kDebug para pruebas de PID durante construction o pits
     * DESACTIVAR SI O SI ANTES DE UNA MATCH O DAMOS PENA AJENA
     * 
     * @param kDebug
     * @see Constants.java
     * 
     */

    if (OIConstants.kDebug) {
      m_operatorController.povUp().whileTrue(pidLiftCommand);
      m_operatorController.povDown().whileTrue(pidWristCommand);
    }

  } 

  /**
   * 
   * Registrar comandos del robot para modo Autonomo
   * NO TOCAR O EL AUTONOMO SE MUERE
   * 
   */

  private void registedCommands() {

    NamedCommands.registerCommand("l1Command", l1CommandGroup);
    NamedCommands.registerCommand("l2Command", l2CommandGroup);
    NamedCommands.registerCommand("l3Command", l3CommandGroup);
    NamedCommands.registerCommand("sourceCommand", sourceCommandGroup);
    NamedCommands.registerCommand("intakeCoral", new AutoIntakeCommand(m_coralSubsystem).withTimeout(3));
    NamedCommands.registerCommand("ejectCoral", new AutoEjectCommand(m_coralSubsystem).withTimeout(1 ));
    NamedCommands.registerCommand("resetGyro", resetGyroCommand);
    
  }

  /**
   * 
   * Inicializar tablero en Elastic para el Driver y Operador
   * 
   */

   PowerDistribution pdh = new PowerDistribution(1, PowerDistribution.ModuleType.kRev); // DEFAULT 63, PUT 1 OTHERWISE THE SIMULATOR FUCKIGN CRASHES

  private void setupElastic() {

    // --- Drive Tab --- //
    
    GameTimer gameTimer = new GameTimer();
    SendableRegistry.add(gameTimer, "Match Time");
    ShuffleboardConstants.kDriverTab.add(gameTimer)
    .withWidget("Match Time")
    .withProperties(Map.of("Font color", "black"))
    .withSize(2, 1)
    .withPosition(4, 4);

    ShuffleboardLayout robotSubsystems = ShuffleboardConstants.kDriverTab
    .getLayout("Robot Subsystems", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(0, 0);

    robotSubsystems.add(m_robotDrive);
    robotSubsystems.add(m_coralSubsystem);
    robotSubsystems.add(m_elevatorSubsystem);
    robotSubsystems.add(m_climberSubsystem);

    // --- Debug Tab --- //

    ShuffleboardConstants.kDebugTab.add("Command Scheduler", CommandScheduler.getInstance())
    .withSize(2, 3)
    .withPosition(4, 0);;
    
    ShuffleboardConstants.kDebugTab.add("Power Distribution Hub", pdh)
    .withWidget(BuiltInWidgets.kPowerDistribution)
    .withSize(3, 4)
    .withPosition(6, 0);

  }

  /**
   * 
   * Funcion que selecciona el autonomo del robot
   * En caso de que kDemo esta activo este hara un SOLO comando, DESACTIVAR ANTES DE MATCH
   * 
   * @return Regresa el autonomo seleccionado en el tablero Elastic 
   * 
   */

  public Command getAutonomousCommand() {
    if (OIConstants.kDemo) { 
      SequentialCommandGroup prueba = new SequentialCommandGroup(liftToSourceCommand, new WaitCommand(1.5), liftToL1Command, new WaitCommand(1.5));
      SequentialCommandGroup yes = new SequentialCommandGroup(prueba, prueba, prueba);

      return yes;
    }

    return centerAutoAlignCommand;
    //return autoChooser.getSelected();
  }
}
