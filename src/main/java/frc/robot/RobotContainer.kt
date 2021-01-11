/*----------------------------------------------------------------------------*/ /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */ /* Open Source Software - may be modified and shared by FRC teams. The code   */ /* must be accompanied by the FIRST BSD license file in the root directory of */ /* the project.                                                               */ /*----------------------------------------------------------------------------*/
package frc.robot

import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.ProfiledPIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.Trajectory
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import edu.wpi.first.wpilibj2.command.button.Button
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import frc.robot.commands.Autonomous.*
import frc.robot.commands.Climber.Lifter
import frc.robot.commands.Climber.Winch
import frc.robot.commands.ColorWheel.Actuator
import frc.robot.commands.ColorWheel.Spin4Times
import frc.robot.commands.ColorWheel.SpinToColorTarget
import frc.robot.commands.Drivetrain.DefaultDrive
import frc.robot.commands.Drivetrain.RotateToPowerPort
import frc.robot.commands.PowerCells.Intake
import frc.robot.commands.PowerCells.RotateIntakeBarTo
import frc.robot.commands.PowerCells.Shooter
import frc.robot.commands.PowerCells.Storage
import frc.robot.subsystems.Climber.ClimberSubsystem
import frc.robot.subsystems.ColorWheel.ActuatorSubsystem
import frc.robot.subsystems.ColorWheel.SpinSubsystem
import frc.robot.subsystems.Drivetrain.DriveSubsystem
import frc.robot.subsystems.Inputs.JoystickSubsystem
import frc.robot.subsystems.Inputs.Jetson.VisionSubsystem
import frc.robot.subsystems.Inputs.LidarSubsystem
import frc.robot.subsystems.PowerCells.IntakeSubsystem
import frc.robot.subsystems.PowerCells.ShooterSubsystem
import frc.robot.subsystems.PowerCells.TransportSubsystem
import org.ejml.equation.IntegerSequence
import java.util.function.BooleanSupplier
import java.util.function.Consumer
import java.util.function.Supplier


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [
 * Robot]
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {

    private val driverJoystick: Joystick
    private val alternateJoystick: Joystick

    /** Subsystems*/
    private val m_driverJoystickSubsystem = JoystickSubsystem(Constants.driverJoystickPort)
    private val m_alternateJoystickSubsystem = JoystickSubsystem(Constants.alternateJoystickPort)

//    private val m_xboxSubsystem = XboxSubsystem()


    private val m_driveSubsystem = DriveSubsystem()
    private val m_intakeSubsystem = IntakeSubsystem()
    private val m_climberSubsystem = ClimberSubsystem()
    private val m_LidarSubsystem = LidarSubsystem()
    private val m_visionSubsystem = VisionSubsystem(Constants.visionHost, Constants.visionPort)
    private val m_shooterSubsystem = ShooterSubsystem()
    private val m_transportSubsystem = TransportSubsystem()

    /**Commands */
    private val m_defaultDrive = DefaultDrive(m_driveSubsystem, m_driverJoystickSubsystem)

    private val Lifter = Lifter(m_climberSubsystem)
    private val Winch = Winch(m_climberSubsystem)
    private val Intake = Intake(m_intakeSubsystem)
    private val shooter = Shooter(m_shooterSubsystem, m_driverJoystickSubsystem)
    private val storage = Storage(m_transportSubsystem)
    private val Bar = RotateIntakeBarTo(m_intakeSubsystem)
    private val rotateToPowerPort = RotateToPowerPort(m_driveSubsystem, m_visionSubsystem, m_driverJoystickSubsystem)
//    private val spin4 = Spin4Times(m_controlPanelSubsystem)
//    private val spinToColor = SpinToColorTarget(m_controlPanelSubsystem)

  /** AUTO Commands */
    private val nullDrive = NullDrive(m_driveSubsystem);


    var runTime = 0.1
    var waitTime = 0.8
//    var power = 0.5 //Three Balls
//    var power = 0.46 //Two Balls
    var power = 0.32 //0.422

    val shoot = storage.ForceRun(-0.8).andThen(WaitCommand(0.2)).andThen(storage.Stop()).andThen(WaitCommand(2.0))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(storage.Run()).andThen(WaitCommand(runTime)).andThen(storage.Stop()).andThen(WaitCommand(waitTime))
            .andThen(PrintCommand("Done Shooting!"))


    val spinStorage = shooter.ForceRun(-0.1).alongWith(storage.ForceRun(0.45))


    val config: TrajectoryConfig = TrajectoryConfig(Constants.forwardMaxVel,
            Constants.forwardMaxAcc) // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_driveSubsystem.kDriveKinematics)




    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a GenericHID or one of its subclasses, and then passing it to a
     * [edu.wpi.first.wpilibj2.command.button.JoystickButton].
     */
    private fun configureButtonBindings() {
        var twoPressed = false

        //@TODO CHANGE KEYBINDS

        //Begins to Shoot Balls
        JoystickButton(driverJoystick, 1).whenPressed(PrintCommand("Shooting!"))
                .whenPressed(shooter.ForceRun(power).alongWith(shoot)).whenInactive(shooter.Stop().alongWith(storage.Stop()))



        //Run Storage
        JoystickButton(driverJoystick, 2).whenPressed(PrintCommand("Running Storage!")).whenPressed(spinStorage).whenReleased(shooter.Stop().alongWith(storage.Stop()))

        //Run Intake
        JoystickButton(driverJoystick, 3).or(JoystickButton(driverJoystick, 4)).or(JoystickButton(driverJoystick, 11))
                .whileActiveOnce(Bar.moveDown().andThen(WaitCommand(0.5).andThen(Intake.Run())).alongWith(PrintCommand("Running Intake!")))
                .whenInactive(Intake.Stop().andThen(WaitCommand(0.2).andThen(Bar.moveUp())))

            //Reverse Intake
        JoystickButton(driverJoystick, 5).or(JoystickButton(driverJoystick, 6)).or(JoystickButton(driverJoystick, 9))
                .whileActiveOnce(Bar.moveDown().andThen(WaitCommand(0.5).andThen(Intake.Reverse()).alongWith(PrintCommand("Reversing Intake!"))))
                .whenInactive(Intake.Stop().andThen(WaitCommand(0.2).andThen(Bar.moveUp())))

        JoystickButton(driverJoystick, 8).whenPressed(Runnable {
            println("" + (power + 0.002 * (-driverJoystick.throttle + 1))) })

        /** ABORT */
        JoystickButton(driverJoystick, 12).whenPressed(shooter.ForceRun(-1.0).alongWith(storage.ForceRun(-0.8)))
                .whenReleased(shooter.Stop().alongWith(storage.Stop() ))

          //Rotate to VisionPort
        Button(BooleanSupplier { driverJoystick.getPOV() == 0 })
                .whileActiveContinuous(rotateToPowerPort)
                .whenInactive(getCartesianDrive());



        /** MECHANICAL JOYSTICK **/
        //Run Winch
        JoystickButton(alternateJoystick, 1).whenPressed(Winch.Run()).whenReleased(Winch.Stop())

        //Run SPEED Winch
        JoystickButton(alternateJoystick, 9).whenPressed(Winch.RunFast()).whenReleased(Winch.Stop())

        //Toggle Spinner Height
//        JoystickButton(alternateJoystick, 2).whenPressed(Runnable {
//            if(twoPressed) {
//                println("Extending!")
//                twoPressed = false
//                actuator.ExtendActuator().schedule()
//            } else {
//                println("Retracting!")
//                twoPressed = true
//                actuator.RetractActuator().schedule()
//            }
//        })

        //Lower Lifter
        JoystickButton(alternateJoystick, 3).or(JoystickButton(alternateJoystick, 5)).whileActiveOnce(Lifter.Lower()).whenInactive(Lifter.Stop())

        //Raise Lifter
        JoystickButton(alternateJoystick, 4).or(JoystickButton(alternateJoystick, 6)).whileActiveOnce(Lifter.Raise()).whenInactive(Lifter.Stop())

        //Spin 4 Rotations
//        JoystickButton(alternateJoystick, 11).whenPressed(spin4).whenReleased(Runnable { spin4.cancel() })

        //Spin to Target Color
//        JoystickButton(alternateJoystick, 12).whenPressed(spinToColor).whenReleased(Runnable { spinToColor.cancel() })
    }

    /**
     * Use this to pass the autonomous command to the main [Robot] class.
     *
     * @return the command to run in autonomous
     */

    fun getAutonomousCommand(): Command {
//        return autoPathSimple()
//        return rotateUntillPowerPort.withTimeout(3.0).andThen(driveToPowerPort.withTimeout(5.0))
//                .andThen(shoot.withTimeout(7.0)).andThen(ForceDrive(m_driveSubsystem, -0.5, 0.0, 0.0)).andThen(WaitCommand(2.0))
//                .andThen(ForceDrive(m_driveSubsystem, 0.0, 0.0, 0.0))

        // Drive to the port and then shoot the balls while keeping the moter safty fed
        return Auto(m_driveSubsystem, m_visionSubsystem, 1.0, true)//.andThen(shooter.ForceRun(power).alongWith(shoot).alongWith(nullDrive))
    }




    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */


    init {

        driverJoystick = m_driverJoystickSubsystem.joystick
        alternateJoystick = m_alternateJoystickSubsystem.joystick
        m_visionSubsystem.startUp()

        // Configure the button bindings
        configureButtonBindings()

//        m_defaultDrive.initialize()
        Bar.initialize()

    }


    fun getCartesianDrive():Command {
        return m_defaultDrive
    }

    fun getRunBar(): Command {
        return Bar
    }

    fun getVision() : VisionSubsystem {
        return m_visionSubsystem
    }

    var exampleTrajectory: Trajectory = TrajectoryGenerator.generateTrajectory( // Start at the origin facing the +X direction
            Pose2d(0.0, 0.0, Rotation2d(0.0)),  // Pass through these two interior waypoints, making an 's' curve path
            listOf(
                    Translation2d(1.0, 1.0),
                    Translation2d(2.0, -1.0)
            ),  // End 3 meters straight ahead of where we started, facing forward
            Pose2d(3.0, 0.0, Rotation2d(0.0)),
            config
    )


    fun generatePathfindingCommand(trajectory: Trajectory): Command {
        return MecanumControllerCommand(
                exampleTrajectory,
                Supplier { m_driveSubsystem.getMPose() },
                SimpleMotorFeedforward(Constants.ks, Constants.kv, Constants.ka),
                m_driveSubsystem.kDriveKinematics,
                PIDController(Constants.xP, Constants.xI, Constants.xD),
                PIDController(Constants.yP, Constants.yI, Constants.yD),
                ProfiledPIDController(Constants.tP, Constants.tI, Constants.tD, TrapezoidProfile.Constraints(Constants.maxRotVel, Constants.maxRotAcc)),//@TODO FIX THIS
                5.0,  //@TODO FIX THIS
                PIDController(0.00239, 0.0, 0.0),
                PIDController(0.00239, 0.0, 0.0),
                PIDController(0.00239, 0.0, 0.0),
                PIDController(0.00239, 0.0, 0.0),
                Supplier { m_driveSubsystem.getWheelSpeeds() },
                Consumer { output -> m_driveSubsystem.setSpeedVoltage(output)},
                m_driveSubsystem
        )
    }


    fun autoPathSimple():Command {
//        return DriveDistance(m_driveSubsystem, 0.5, 1.5).andThen(ForceDrive(m_driveSubsystem, 0.0, 0.0, 0.0).perpetually())
        return ForceDrive(m_driveSubsystem, 0.5, 0.0, 0.0, 2.0)

    }


}
