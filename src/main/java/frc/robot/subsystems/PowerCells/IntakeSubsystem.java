/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.PowerCells;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class IntakeSubsystem extends SubsystemBase {

  public WPI_TalonSRX m_intakeDeploy = new WPI_TalonSRX(Constants.intakeDeployPort);
  public WPI_VictorSPX m_intake = new WPI_VictorSPX(Constants.intakePort);
  public WPI_TalonSRX m_innerIntake = new WPI_TalonSRX(Constants.insideIntakePort);

  public Integer intakeEncoderTicks = 0;

  public IntakeSubsystem() {
    m_intakeDeploy.setSelectedSensorPosition(0);
    m_intakeDeploy.setSensorPhase(false);
    m_intakeDeploy.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    m_intakeDeploy.configMotionSCurveStrength(2);

    m_intakeDeploy.config_kP(0, 0.2903*4 ); //0.2903 (maybe)
    m_intakeDeploy.config_kI(0, 0.0);
    m_intakeDeploy.config_kD(0, 0.0);

    m_intakeDeploy.config_kF(0, 0.5);

    m_intakeDeploy.configMotionAcceleration(160*4);
    m_intakeDeploy.configMotionCruiseVelocity(1600*2);

  }

  public Integer getDeployTicks() {
    return intakeEncoderTicks;
  }

  @Override
  public void periodic() {
    intakeEncoderTicks = (int) m_intakeDeploy.getSelectedSensorPosition();
  }


}
