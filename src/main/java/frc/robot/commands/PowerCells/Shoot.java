/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Possible way to make the Robot shoot
// would be called with .WhileActiveContinus function
package frc.robot.commands.PowerCells;

import frc.robot.subsystems.PowerCells.ShooterSubsystem;
import frc.robot.subsystems.PowerCells.TransportSubsystem;
import frc.robot.subsystems.Inputs.JoystickSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.Timer;

public class Shoot extends CommandBase {
  /**
  private ShooterSubsystem shooterSubsystem;
  private TransportSubsystem transportSubsystem;
  private JoystickSubsystem joystickSubsystem;
  private static final Timer timer = new Timer();

  public Shoot(ShooterSubsystem shooter, TransportSubsystem transport, JoystickSubsystem stick) {

    shooterSubsystem = shooter;
    transportSubsystem = transport;
    joystickSubsystem = stick;
    
    timer.reset();
    timer.start();
    addRequirements(shooter, transport, stick);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transportSubsystem.setPower(-0.5);
    shooterSubsystem.setPower(0.65);
    TimeUnit.MILLISECONDS.sleep(200);
    transportSubsystem.setPower(0.0);
    TimeUnit.MILLISECONDS.sleep(1000);
    shooterSubsystem.setPower(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transportSubsystem.setPower(0.6);
    TimeUnit.MILLISECONDS.sleep(200);
    transportSubsystem.setPower(0.0);
    TimeUnit.MILLISECONDS.sleep(500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setPower(0.0);
    transportSubsystem.setPower(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  **/
}
