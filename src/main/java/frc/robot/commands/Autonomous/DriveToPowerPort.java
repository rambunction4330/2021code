/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem;
import frc.robot.subsystems.Inputs.LidarSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class DriveToPowerPort extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private PIDController controller;
  private Double distance;
  private Double minVisionDistance = 1.5;

  public DriveToPowerPort(DriveSubsystem drive, VisionSubsystem vision, Double targetDistance) {
    driveSubsystem = drive;
    visionSubsystem = vision;
    distance = targetDistance;

    controller = new PIDController(Constants.tP, Constants.tI, Constants.tD);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(0.1, 0.05);
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = controller.calculate(driveSubsystem.getHeading(), 0.0);
    System.out.println(visionSubsystem.getPowerPortBearing());

        driveSubsystem.autoCartesian(-visionSubsystem.getPowerPortBearing() / 6, visionSubsystem.getPowerDistance() / (3.0 + minVisionDistance), output / 180);
//        driveSubsystem.driveCartesan(visionSubsystem.getPowerDistance() / (3.0 + minVisionDistance), -visionSubsystem.getPowerPortBearing() / 6, output / 180);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.driveCartesan(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return false;
//           visionSubsystem.getPowerDistance() <= distance;

  }


}
