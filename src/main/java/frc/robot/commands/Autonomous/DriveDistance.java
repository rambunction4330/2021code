/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DriveDistance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_driveSubsystem;
  private Double speed, distance;


  public DriveDistance(DriveSubsystem subsystem, Double spd, Double dst) {
    m_driveSubsystem = subsystem;
    this.distance = dst;
    this.speed = spd;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_driveSubsystem.driveCartesan(speed,  0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_driveSubsystem.getFrontLeftMotor().getSelectedSensorPosition() / 4096.0) >= distance;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.driveCartesan(0.0, 0.0, 0.0);
  }
}
