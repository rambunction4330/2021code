/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PowerCells.IntakeSubsystem;
import frc.robot.subsystems.PowerCells.TransportSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public final class Storage {
  private final TransportSubsystem m_transportSubsystem;

  public Storage(TransportSubsystem transp) { m_transportSubsystem = transp; }

  public class Run extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public Run() { addRequirements(m_transportSubsystem); }

    @Override
    public void initialize() {
      m_transportSubsystem.m_storage.set(0.6);
    }
  }

  public class Stop extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public Stop() { addRequirements(m_transportSubsystem); }

    @Override
    public void initialize() {
      m_transportSubsystem.m_storage.set(0.0);
    }
  }

  public class ForceRun extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    Double speed;

    public ForceRun(double spd) {
      speed = spd;
      addRequirements(m_transportSubsystem);
    }

    @Override
    public void initialize() { m_transportSubsystem.m_storage.set(speed); }
  }
}
