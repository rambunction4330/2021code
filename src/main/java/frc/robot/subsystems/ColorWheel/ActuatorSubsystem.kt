/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.subsystems.ColorWheel

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.revrobotics.ColorMatch
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants


class ActuatorSubsystem : SubsystemBase() {

    var rotationActuator = Servo(Constants.rotationActuatorPort)


    override fun periodic() { }

    fun moveActuatorIn() { rotationActuator.position = 0.17 }
    fun moveActuatorOut() { rotationActuator.position = 0.82 }



    init {
        rotationActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0)

        moveActuatorIn()
    }
}