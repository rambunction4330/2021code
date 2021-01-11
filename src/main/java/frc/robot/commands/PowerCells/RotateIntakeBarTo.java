/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.PowerCells;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.subsystems.PowerCells.IntakeSubsystem;

/**
 * An example command that uses an example subsystem.
 */
public class RotateIntakeBarTo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_intakeSubsystem;

  private Double target = Constants.upPosEncoderTicks;


  public RotateIntakeBarTo(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;

  }

  @Override
  public void execute() {
    m_intakeSubsystem.m_intakeDeploy.config_kF(0, kF());
    m_intakeSubsystem.m_intakeDeploy.set(ControlMode.MotionMagic, target);
  }

  @Override
  public boolean isFinished() { return false; }



  public class moveUp extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    @Override
    public void initialize() {
      target = Constants.upPosEncoderTicks;
    }
  }
  public class moveDown extends InstantCommand {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    @Override
    public void initialize() {
      target = Constants.downPosEncoderTicks;    }
  }

  private Double kF() {
    return ((Constants.armLen*Constants.armWeight*Constants.armResistance) / (Constants.kt) ) * Math.cos(Math.toRadians(m_intakeSubsystem.intakeEncoderTicks/1024.0));
  }

}



