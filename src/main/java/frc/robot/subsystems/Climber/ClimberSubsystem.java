/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClimberSubsystem extends SubsystemBase {

  public WPI_VictorSPX m_lifter = new WPI_VictorSPX(Constants.climbLifterPort);
  public WPI_VictorSPX m_winch = new WPI_VictorSPX(Constants.climbWinchPort);

  public ClimberSubsystem() {

  }

  @Override
  public void periodic() { }
}