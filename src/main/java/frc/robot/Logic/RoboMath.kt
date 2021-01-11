package frc.robot.Logic

import frc.robot.Constants
import java.lang.Math

object RoboMath {

    fun targetX(Angle: Double, Distance: Double): Double {
        return Math.cos(Angle)*Distance
    }

    fun targetY(Angle: Double, Distance: Double): Double {
        return Math.sin(Angle)*Distance
    }

    fun shooterPower(distance: Double): Double {
        val factor = 1.0
        return distance * factor
    }


}