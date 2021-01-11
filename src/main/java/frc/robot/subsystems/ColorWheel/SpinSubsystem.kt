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


class SpinSubsystem : SubsystemBase() {

    var rotatorMotor = WPI_VictorSPX(Constants.rotatorPort)
    var colorSensor = ColorSensorV3(I2C.Port.kMXP)
    private var color = colorSensor.color
    private val m_colorMatcher = ColorMatch()
    private val kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429)
    private val kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240)
    private val kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114)
    private val kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113)


    override fun periodic() { }



    fun getColor(): Color {
        color = colorSensor.color
        return color
    }

    fun getNearestColor(): String {
        color = colorSensor.color

        val match = m_colorMatcher.matchClosestColor(color)

        when {
            match.color === kBlueTarget -> {
                return "Blue"
            }
            match.color === kRedTarget -> {
                return "Red"
            }
            match.color === kGreenTarget -> {
                return "Green"
            }
            match.color === kYellowTarget -> {
                return "Yellow"
            }
            else -> {
                return "Unknown"
            }
        }
    }

    init {
        m_colorMatcher.addColorMatch(kBlueTarget)
        m_colorMatcher.addColorMatch(kGreenTarget)
        m_colorMatcher.addColorMatch(kRedTarget)
        m_colorMatcher.addColorMatch(kYellowTarget)

    }
}