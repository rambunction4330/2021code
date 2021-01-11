/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateUntillPowerPort extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private final VisionSubsystem m_visionSubsystem;

  public RotateUntillPowerPort(DriveSubsystem drive, VisionSubsystem vision) {
    m_driveSubsystem = drive;
    m_visionSubsystem = vision;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_driveSubsystem.autoCartesian(0, 0, 0.75);
  }

  @Override
  public void execute() {
    System.out.println(m_visionSubsystem.getPowerPortBearing());
    m_driveSubsystem.autoCartesian(0, 0, 0.75);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.autoCartesian(0,0,0);
    System.out.println("done!");
  }

  @Override
  public boolean isFinished() {
    return m_visionSubsystem.getPowerPortBearing() != Double.MIN_VALUE;
  }
}
