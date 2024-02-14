@file:Suppress("unused")
package org.firstinspires.ftc.teamcode.robot

import kotlin.math.roundToInt

class QOL {
    companion object {
        fun inchesToTicks(inches: Number): Int {
            return (inches.toDouble() * MotorConstants.GoBilda312.TICKS_PER_INCH).roundToInt()
        }
        fun ticksToInches(ticks: Number): Double {
            return ticks.toDouble() / MotorConstants.GoBilda312.TICKS_PER_INCH
        }


        fun radToDeg(radians: Float): Double {
            return radians * 180 / Math.PI
        }
        fun degToRad(degrees: Int): Double {
            return degrees * Math.PI / 180
        }
        fun rED(current: Boolean, previous: Boolean): Boolean { // Rising Edge Detector
            return current && !previous
        }
        fun lerp(p0: Double, p1: Double, t: Double) : Double {
            return p0 * (1.0 - t) + (p1 * t)
        }
    }
}

enum class Side {
    LEFT, RIGHT, BOTH
}

enum class Direction {
    FORWARD, BACKWARD, LEFT, RIGHT
}


enum class LiftControlType {
    MANUAL, PID
}

enum class MotorConstants(val MAX_RPM: Double, val TICKS_PER_REV: Double, val WHEEL_DIAMETER: Double, val TICKS_PER_INCH: Double) {
    GoBilda312(312.0, 537.7, 96.0 / 25.4, 537.7 / ((96.0 / 25.4) * Math.PI))
}

//@Config()
//object DriveConstants{
//
//}

// create an enum class where each value is a double that represents the strength of the rumble
enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}