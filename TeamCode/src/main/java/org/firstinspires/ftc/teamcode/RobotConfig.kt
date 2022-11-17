package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.robotcore.hardware.*
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs


enum class Side {
    LEFT, RIGHT, BOTH
}

enum class Direction {
    FORWARD, BACKWARD, LEFT, RIGHT
}

// create an enum class where each value is a double that represents the strength of the rumble

enum class RumbleStrength(val strength: Double) {
    LOW(0.25),
    MEDIUM(0.5),
    HIGH(0.75),
    MAX(1.0)
}

class RobotConfig(hwMap: HardwareMap?) {
    var FL: DcMotorEx
    var FR: DcMotorEx
    var BL: DcMotorEx
    var BR: DcMotorEx

    var TURRET: DcMotorEx
    var SLIDES: DcMotorEx
    var ARM: DcMotorEx

    var CLAW: Servo
    var CLAW_ROTATE: CRServo

    val TICKS_PER_REV_312 = ((((1+(46/17))) * (1+(46/11))) * 28)
    val TICKS_PER_REV_223 = ((((1+(46/11))) * (1+(46/11))) * 28)


    private var hardwareMap: HardwareMap? = null

    fun funnyDrive(drive: Double, turn: Double){
        FL.power = drive + turn
        FR.power = drive - turn
        BL.power = drive + turn
        BR.power = drive - turn
    }

    fun drive(drive: Double, strafe: Double, turn: Double) {
        var max: Double;
        var leftFrontPower: Double = drive + strafe + turn
        var rightFrontPower: Double = drive - strafe - turn
        var leftBackPower: Double = drive - strafe + turn
        var rightBackPower: Double = drive + strafe - turn

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower))
        max = Math.max(max, Math.abs(leftBackPower))
        max = Math.max(max, Math.abs(rightBackPower))

        if (max > 1.0) {
            leftFrontPower /= max
            rightFrontPower /= max
            leftBackPower /= max
            rightBackPower /= max
        }
        FL.setPower(leftFrontPower)
        FR.setPower(rightFrontPower)
        BL.setPower(leftBackPower)
        BR.setPower(rightBackPower)

    }

    fun gamepadDrive(controller: Gamepad, multiplier: Double) {
        drive(
            -controller.left_stick_y.toDouble() * multiplier,
            controller.left_stick_x.toDouble() * multiplier,
            controller.right_stick_x.toDouble() * multiplier
        )
    }

    fun stop() {
        drive(0.0, 0.0, 0.0)
    }

    fun pidDrive(distanceInM: Double, direction: Direction, Kp: Double, Ki: Double, Kd: Double){
        var dashboard: FtcDashboard = FtcDashboard.getInstance()
        val WHEEL_DIAMETER = 96.0 / 1000.0

        val initialWheelPosition = FR.currentPosition

        //convert ticks per rev to ticks per meter
        val TICKS_PER_METER = TICKS_PER_REV_312 / (WHEEL_DIAMETER * Math.PI)


        val target = distanceInM * TICKS_PER_METER
        var lastReference = target
        var integralSum = 0.0
        var lastError = 0.0

        val maxIntegralSum = 0.5


        val timer: ElapsedTime = ElapsedTime()



        while (abs(FR.currentPosition - initialWheelPosition) < abs(target)) {
            var packet = TelemetryPacket()
            val error = target - (FR.currentPosition - initialWheelPosition)

            val errorChange = error - lastError

            val derivative = errorChange / timer.seconds()

            integralSum += (error * timer.seconds())

            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum
            } else if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum
            }

            if (target != lastReference) {
                integralSum = 0.0
            }

            val output = (Kp * error) + (Ki * integralSum) + (Kd * derivative)

            when (direction) {
                Direction.FORWARD -> funnyDrive(output, 0.0)
                Direction.BACKWARD -> funnyDrive(-output, 0.0)
                else -> {
                    // do nothing
                }
            }



            lastError = error
            lastReference = target
            timer.reset()
        }
        stop()
    }



    fun lerp(p0: Double, p1: Double, t: Double) : Double {
        return (1 - t) * p0 + p1 * t;
    }

    fun rumble(controller: Gamepad, side: Side, power: RumbleStrength, duration: Int = 100) {
        val pwr = power.strength
        when (side) {
            Side.LEFT -> {
                controller.rumble(pwr, 0.0, duration)
            }
            Side.RIGHT -> {
                controller.rumble(0.0, pwr, duration)
            }
            Side.BOTH -> {
                controller.rumble(pwr / 2, pwr / 2, duration)
            }
        }
    }
    init {
        hardwareMap = hwMap


        FL = hardwareMap!!.get(DcMotorEx::class.java, "fl")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "fr")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "bl")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "br")

        TURRET = hardwareMap!!.get(DcMotorEx::class.java, "turret")
        SLIDES = hardwareMap!!.get(DcMotorEx::class.java, "slides")
        ARM = hardwareMap!!.get(DcMotorEx::class.java, "arm")

        CLAW = hardwareMap!!.get(Servo::class.java, "claw")
        CLAW_ROTATE = hardwareMap!!.get(CRServo::class.java, "clawAngle")




        FL.direction = DcMotorSimple.Direction.REVERSE
        BL.direction = DcMotorSimple.Direction.REVERSE

        FR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        ARM.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}
