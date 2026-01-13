// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.util.Elastic;
import frc.robot.util.ElasticNotification;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.TejuinoBoard;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Elastic.Notification notification = new Elastic.Notification();
  private TejuinoBoard LED_CONTROL;
  private final SendableChooser<Runnable> ledChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    LimelightHelpers.setLEDMode_ForceOff(CameraConstants.kLimelightName);
    m_robotContainer = new RobotContainer();
    DataLogManager.start();

    // CameraServer.startAutomaticCapture(); // Start USB Camera
    if (OIConstants.kLEDController) {
      LED_CONTROL = m_robotContainer.tejuino_board; // Start LED Controller
      LED_CONTROL.escuderia_effect(LED_CONTROL.LED_STRIP_0);
    }

    Elastic.sendNotification(notification
        .withLevel(Elastic.NotificationLevel.INFO)
        .withTitle("Robot Init")
        .withDescription("Robot initialized successfully.")
        .withDisplaySeconds(5.0));
    if (OIConstants.kLEDController) {
      confiureChoosers();
    }
    if (OIConstants.kDebug)
      Elastic.sendNotification(notification
          .withLevel(Elastic.NotificationLevel.WARNING)
          .withTitle("DEBUG MODE ON")
          .withDescription("MODO DEBUG ACTIVADO - NO COMPETENCIA")
          .withDisplaySeconds(10));
    if (OIConstants.kDemo)
      Elastic.sendNotification(notification
          .withLevel(Elastic.NotificationLevel.WARNING)
          .withTitle("DEMO MODE ON")
          .withDescription("MODO DEMO ACTIVADO - NO COMPETENCIA")
          .withDisplaySeconds(10));
  }

  private void autoLED() {
    if (OIConstants.kLEDController) {
      var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Red) {
        LED_CONTROL.all_leds_red(LED_CONTROL.LED_STRIP_1);
      } else if (alliance.get() == DriverStation.Alliance.Blue) {
        LED_CONTROL.all_leds_blue(LED_CONTROL.LED_STRIP_1);
      }
    }
  }

  public void confiureChoosers() {
    ledChooser.setDefaultOption("Off", () -> LED_CONTROL.all_leds_white(LED_CONTROL.LED_STRIP_1));
    ledChooser.addOption("Red", () -> LED_CONTROL.all_leds_red(LED_CONTROL.LED_STRIP_1));
    ledChooser.addOption("Blue", () -> LED_CONTROL.all_leds_blue(LED_CONTROL.LED_STRIP_1));
    ledChooser.addOption("Green", () -> LED_CONTROL.all_leds_green(LED_CONTROL.LED_STRIP_1));
    ledChooser.addOption("Rainbow", () -> LED_CONTROL.rainbow_effect(LED_CONTROL.LED_STRIP_1));
    ledChooser.addOption("Automatic", () -> autoLED());
    ShuffleboardConstants.kDriverTab.add("LED Mode", ledChooser)
        .withSize(3, 1)
        .withPosition(8, 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    Elastic.selectTab("Autonomous");
    ElasticNotification.sendNotification(notification, "Autonomous Init", "Autonomous mode initialized.",
        Elastic.NotificationLevel.WARNING, 5.0);
    if (OIConstants.kLEDController) {
      LED_CONTROL.all_leds_white(LED_CONTROL.LED_STRIP_1);
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if (m_autonomousCommand.isFinished()) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Elastic.selectTab("Teleoperated");
    ElasticNotification.sendNotification(notification, "Teleop Init", "Teleoperated mode initialized.",
        Elastic.NotificationLevel.WARNING, 5.0);

    if (OIConstants.kLEDController) {
      var alliance = DriverStation.getAlliance();
      if (alliance.get() == DriverStation.Alliance.Red) {
        LED_CONTROL.all_leds_red(LED_CONTROL.LED_STRIP_1);
      } else if (alliance.get() == DriverStation.Alliance.Blue) {
        LED_CONTROL.all_leds_blue(LED_CONTROL.LED_STRIP_1);
      }
    }

  }

  @Override
  public void teleopPeriodic() {
    if (OIConstants.kLEDController) {
      ledChooser.getSelected().run();
    }
  }

  @Override
  public void teleopExit() {
    if (OIConstants.kLEDController) {
      LED_CONTROL.all_leds_blue(LED_CONTROL.LED_STRIP_1);
    }
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
