/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber.ClimberSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public final class Lifter {
  private final ClimberSubsystem m_climbSubsystem;
  public Lifter(ClimberSubsystem climb) { m_climbSubsystem = climb; }

  public class Raise extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Raise() { addRequirements(m_climbSubsystem); }

    @Override
    public void initialize() { m_climbSubsystem.m_lifter.set(-0.7); }
  }

  public class Lower extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Lower() { addRequirements(m_climbSubsystem); }

    @Override
    public void initialize() {
      m_climbSubsystem.m_lifter.set(0.3);
    }
  }

  public class Stop extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public Stop() { addRequirements(m_climbSubsystem); }

    @Override
    public void initialize() {
      m_climbSubsystem.m_lifter.set(0.0);
    }
  }
}
