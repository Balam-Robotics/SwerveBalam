// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class AutoLimelightCommand extends Command {

  private DriveSubsystem m_robotDrive;
  private String m_limelight;
  private double tx;
  private boolean tv;

  public AutoLimelightCommand(DriveSubsystem driveSubsystem, String limelight) {
    m_robotDrive = driveSubsystem;
    m_limelight = limelight;
    System.out.println("Limelight Command");
    addRequirements(driveSubsystem);
  }

  private void fixShootingPosition(ChassisSpeeds desired) {
    m_robotDrive.setChassisSpeed(desired);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setLEDMode_ForceOn(m_limelight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    tx = LimelightHelpers.getTX(m_limelight);
    tv = LimelightHelpers.getTV(m_limelight);

    if (tv) {
      if (tx > 5) {
        fixShootingPosition(new ChassisSpeeds(0,  0, -Math.PI / 5));
      } else if (tx < -5) {
        fixShootingPosition(new ChassisSpeeds(0, 0, Math.PI / 5));
      } else if (tx < 2.5 || tx > -2.5) {
        fixShootingPosition(new ChassisSpeeds(0, 0, 0));
      } 
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    fixShootingPosition(new ChassisSpeeds(0, 0, 0));
    LimelightHelpers.setLEDMode_ForceOff(m_limelight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
