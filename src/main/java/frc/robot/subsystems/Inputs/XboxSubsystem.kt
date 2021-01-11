/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot.subsystems.Inputs

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants.xboxPort

class XboxSubsystem : SubsystemBase() {
    /**
     * Creates a new ExampleSubsystem.
     */
    var xboxController: XboxController

    init {
        xboxController = XboxController(xboxPort)
    }

    override fun periodic() { }
}