/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem;
import frc.robot.subsystems.PowerCells.ShooterSubsystem;
import frc.robot.subsystems.PowerCells.TransportSubsystem;

public class Auto extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  private boolean atPort = false;
  private PIDController rotationController = new PIDController(0.2, 0.0, 0.02);
  private PIDController xController = new PIDController(0.5, 0.0, 0.02);
  private PIDController yController = new PIDController(1.0, 0.0, 0.0);

  private double startTime;
  private double forwardTime = 1000;
  private double distanceToPort;
  private boolean turnLeft;


  public Auto(DriveSubsystem drive, VisionSubsystem vision, Double endDistance, Boolean isLeftOfTargetWhileFacing) {
    driveSubsystem = drive;
    visionSubsystem = vision;
    distanceToPort = endDistance;
    turnLeft = !isLeftOfTargetWhileFacing;

    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    driveSubsystem.autoCartesian(0.0, 0.6, 0.0);
    System.out.println("Starting Auto!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (System.currentTimeMillis() - startTime <= forwardTime) {
      System.out.println("Driving Forward!");
      // Drive straight forward for a bit to ensure points even if vision fails
      driveSubsystem.autoCartesian(0.0, 0.6, 0.0);

    } else if (visionSubsystem.getPowerPortBearing() == Double.MIN_VALUE) {
      System.out.println("Finding VisionPort!");
      double rotationOutput = rotationController.calculate(driveSubsystem.getHeading(), 0.0) / 180.0;
      // Turn so that the vision target is in view
      // Make positive if the robot is on the right
      // Make negitive if the robot is on the left
      if (turnLeft) driveSubsystem.autoCartesian(0.8, 0.0, rotationOutput);
      else driveSubsystem.autoCartesian(-0.8, 0.0, rotationOutput);
      
    } else {
      // Navigate to port untill the robot is within shoting range
      System.out.println("Driving to VisionPort!");
      System.out.println(visionSubsystem.getPowerDistance() + "!");
      if (visionSubsystem.getPowerDistance() > distanceToPort) {
        driveToPowerPort();
      } else {
        // Set variable to stop the command
        System.out.println("Done!");
        atPort = true;
        driveSubsystem.autoCartesian(0.0, 0.0, 0.0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //stop driving once at the port
    driveSubsystem.autoCartesian(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atPort;
  }

  private void driveToPowerPort() {
    // PID calculations for the robot
    // X --> tries to keep the vision bearing in the center by moving side to side
    // Y --> drives the robot to the port
    // Rotation --> keeps the robot correctly oriented on the field
    double xOutput = xController.calculate(visionSubsystem.getPowerPortBearing(), 0.0) / 3.0;
    double yOutput = yController.calculate(visionSubsystem.getLoadingBayDistanc(), distanceToPort) / 3.0;
    double rotationOutput = rotationController.calculate(driveSubsystem.getHeading(), 0.0) / 180.0;
//    System.out.println(rotationOutput);
    System.out.println(visionSubsystem.getPowerPortBearing());

    //drive what the PID Calculated
    driveSubsystem.autoCartesian(xOutput, 0.0, rotationOutput);
  }
}
