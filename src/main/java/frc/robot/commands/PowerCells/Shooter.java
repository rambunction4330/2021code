/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Inputs.JoystickSubsystem;
import frc.robot.subsystems.PowerCells.ShooterSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public final class Shooter {
  private ShooterSubsystem m_shooterSubsystem;
  private JoystickSubsystem m_joystickSubsystem;

  public Shooter(ShooterSubsystem shoot, JoystickSubsystem joy) {
    m_shooterSubsystem = shoot;
    m_joystickSubsystem = joy;
  }

  public class ForceRun extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    Double speed;

    public ForceRun(double spd) {
      speed = spd;
      addRequirements(m_shooterSubsystem);
    }

    @Override
    public void initialize() {
      System.out.println(speed + 0.002*(-m_joystickSubsystem.getJoystick().getThrottle() + 1));
      m_shooterSubsystem.m_flyWheel.set(speed + 0.002*(-m_joystickSubsystem.getJoystick().getThrottle() + 1)); }
  }

  public class Run extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    Double goalSpeed;
    Double currentSpeed;

    public Run(Double speed) {
      addRequirements(m_shooterSubsystem);
      goalSpeed = speed;
    }

    @Override
    public void initialize() {
      m_shooterSubsystem.m_flyWheel.set(1.0);
      currentSpeed = 1.0;
    }

    @Override
    public void execute() {
      m_shooterSubsystem.m_flyWheel.set(currentSpeed);
      currentSpeed = currentSpeed + ((goalSpeed - currentSpeed) / 10);
    }

    @Override
    public boolean isFinished() {
      return Math.abs(goalSpeed - currentSpeed) < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
      m_shooterSubsystem.m_flyWheel.set(goalSpeed);
    }
  }

  public class Stop extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Stop() { addRequirements(m_shooterSubsystem); }

    @Override
    public void initialize() { m_shooterSubsystem.m_flyWheel.set(0.0); }
  }
}
