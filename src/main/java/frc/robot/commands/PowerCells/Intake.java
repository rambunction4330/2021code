/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PowerCells.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public final class Intake {
  private final IntakeSubsystem m_intakeSubsystem;
  private final Double speed = 0.4;

  public Intake(IntakeSubsystem intake) { m_intakeSubsystem = intake; }

  public class Run extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public Run() { addRequirements(m_intakeSubsystem); }

    @Override
    public void initialize() {
      m_intakeSubsystem.m_intake.set(speed);
      m_intakeSubsystem.m_innerIntake.set(0.35);
    }
  }

  public class Stop extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public Stop() { addRequirements(m_intakeSubsystem); }

    @Override
    public void initialize() {
      m_intakeSubsystem.m_intake.set(0.0);
      m_intakeSubsystem.m_innerIntake.set(0.0);
    }
  }

  public class Reverse extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public Reverse() { addRequirements(m_intakeSubsystem); }

    @Override
    public void initialize() {
      m_intakeSubsystem.m_intake.set(-1 * speed);
      m_intakeSubsystem.m_innerIntake.set(-0.5);
    }
  }
}
