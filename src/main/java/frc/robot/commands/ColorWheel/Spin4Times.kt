/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.ColorWheel

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ColorWheel.SpinSubsystem

/**
 * An example command that uses an example subsystem.
 */

class Spin4Times(val m_spinSubsystem: SpinSubsystem) : CommandBase() {
    init { addRequirements(m_spinSubsystem) }

    var lastPos = m_spinSubsystem.getNearestColor()

    var count = 0

    override fun initialize() {
        println("Starting to Spin!")
        count = 0
        m_spinSubsystem.rotatorMotor.set(0.66)
        addRequirements(m_spinSubsystem)
    }

    override fun execute() {
        if (!m_spinSubsystem.getNearestColor().equals(lastPos)) {
            count++
            if (count - 8 == 0)  println("One Rotation Done!")
            if (count - 16 == 0) println("Two Rotations Done!")
            if (count - 24 == 0) println("Three Rotation Done!")
        }
        lastPos = m_spinSubsystem.getNearestColor()
    }

    override fun end(interrupted: Boolean) {
        m_spinSubsystem.rotatorMotor.set(0.0)
        println("Done Spinning!")
    }

    override fun isFinished(): Boolean {
        return (count > 32)
    }


}