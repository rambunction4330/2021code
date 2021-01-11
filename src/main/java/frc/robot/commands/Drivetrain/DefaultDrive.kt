/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Drivetrain

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.Drivetrain.DriveSubsystem
import frc.robot.subsystems.Inputs.JoystickSubsystem
import frc.robot.subsystems.Inputs.XboxSubsystem

/**
 * An example command that uses an example subsystem.
 */

class DefaultDrive(private val m_DriveSubsystem: DriveSubsystem, private val m_joystickSubsystem: JoystickSubsystem) : CommandBase() {

    /**
 * Creates a new ExampleCommand.
 *
 * subsystem The subsystem used by this command.
 */

    var maxVel = 0.0
    var maxAcc = 0.0
    var end = false

    init { addRequirements(m_DriveSubsystem) }

    override fun initialize() {
        end = false
    }

    override fun execute() {


        //FLIGHTSTICK
        m_DriveSubsystem.driveCartesan(m_joystickSubsystem.joystick.x, m_joystickSubsystem.joystick.y, m_joystickSubsystem.joystick.twist)

        //XBOX
//        m_DriveSubsystem.driveCartesan(m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getY(GenericHID.Hand.kLeft), m_xboxSubsystem.xboxController.getX(GenericHID.Hand.kRight))
    }

    override fun end(interrupted: Boolean) {
        m_DriveSubsystem.driveCartesan(0.0, 0.0, 0.0)
        end = false
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return end
    }
}
