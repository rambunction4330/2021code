/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.ColorWheel

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ColorWheel.SpinSubsystem

/**
 * An example command that uses an example subsystem.
 */
class SpinToColorTarget(val m_SpinSubsystem: SpinSubsystem) : CommandBase() {
    private val m_colorList = listOf<String>("B", "G", "R", "Y")

    private val m_color = DriverStation.getInstance().gameSpecificMessage
    private val hasColor = m_color.equals("Y") || m_color.equals("B") || m_color.equals("G") || m_color.equals("R")
    private val m_colorTarget = m_colorList.get((m_colorList.indexOf(m_color) + 2) % 4)

    init {
        addRequirements(m_SpinSubsystem)
    }


    var spinPos = true
    var currentPos = m_SpinSubsystem.getNearestColor()

    override fun initialize() {
        if (!hasColor) {
            println("No Color Received!")
            cancel()
        }
        else {
            println("Starting to Spin!")
            m_SpinSubsystem.rotatorMotor.set(-0.66)
        }
    }

    override fun execute() {
    }

    override fun end(interrupted: Boolean) {
        m_SpinSubsystem.rotatorMotor.set(0.0)
        println("Done Spinning!")
    }

    override fun isFinished(): Boolean {
        return m_SpinSubsystem.getNearestColor().equals(m_colorTarget)
    }

}