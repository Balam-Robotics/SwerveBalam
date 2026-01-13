// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.util.VL53L0X;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToSource extends Command {

  private final DriveSubsystem drive;
  private final VL53L0X sensor;
  private final double targetMin = 270;
  private final double targetMax = 276;
  private final double speed;

  /** Creates a new MoveToSource. */
  public MoveToSource(DriveSubsystem drive, VL53L0X sensor, double speed) {
    this.drive = drive;
    this.sensor = sensor;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setChassisSpeed(new ChassisSpeeds(0, 0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = sensor.getDistance();
    if (distance < targetMin) {
      drive.stopChassis();
    } else if (distance > targetMax) {
      drive.setChassisSpeed(new ChassisSpeeds(speed, 0, 0));
    } else {
      drive.stopChassis();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { 
    drive.stopChassis();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double distance = sensor.getMedianDistance();
    return distance >= targetMin && distance <= targetMax;
  }
}
