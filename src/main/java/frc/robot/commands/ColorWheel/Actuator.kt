/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.commands.ColorWheel

import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.subsystems.ColorWheel.ActuatorSubsystem
import frc.robot.subsystems.ColorWheel.SpinSubsystem

/**
 * An example command that uses an example subsystem.
 */
class Actuator(act: ActuatorSubsystem) {
    val m_actuatorSubsystem: ActuatorSubsystem = act

    inner class ExtendActuator: InstantCommand() {

        override fun initialize() { m_actuatorSubsystem.moveActuatorOut() }

        init { addRequirements(m_actuatorSubsystem) }
    }

    inner class RetractActuator() : InstantCommand() {

        override fun initialize() { m_actuatorSubsystem.moveActuatorIn() }

        init { addRequirements(m_actuatorSubsystem) }
    }

}