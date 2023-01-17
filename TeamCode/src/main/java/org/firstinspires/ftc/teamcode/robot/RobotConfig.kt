@file:Suppress("unused", "NAME_SHADOWING")
package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.robomatic.util.PIDController
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.utilities.AutoMode.*
import org.firstinspires.ftc.teamcode.utilities.DriveConstants
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.AutoDriveTolerance
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.AutoTurnTolerance
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.drive_kP
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.drive_kI
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.drive_kD
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.heading_kD
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.heading_kI
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.heading_kP
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.strafeMultiplier
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.turn_kD
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.turn_kI
import org.firstinspires.ftc.teamcode.utilities.DriveConstants.turn_kP
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.radToDeg
import org.firstinspires.ftc.teamcode.utilities.QOL.Companion.ticksToInches
import org.firstinspires.ftc.teamcode.utilities.RumbleStrength
import org.firstinspires.ftc.teamcode.utilities.Side
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin


class RobotConfig(hwMap: HardwareMap?) {
    var FL: DcMotorEx
    var FR: DcMotorEx
    var BL: DcMotorEx
    var BR: DcMotorEx

    var driveMotors: Array<DcMotorEx>

    var SLIDES: Slides

    var CLAW: Servo

    var CONE_SENSOR: Rev2mDistanceSensor

    var IMU: BNO055IMU

    private var distanceTarget: Double = 0.0
    private var distanceError: Double = 0.0

    private var angleTarget: Double = 0.0
    private var errorAngle: Double = 0.0

    private var headingError: Double = 0.0
    private var headingTarget: Double = 0.0

    private var correction: Double = 0.0

    private var headingCorrection: Double = 0.0

    private var lastAngle: Double = botHeading

    private var globalAngle: Double = botHeading

    private var hasBeenRun = true

    var autoMode = UNKNOWN

    val currentPosition: Int
        get() {
            return (FL.currentPosition + FR.currentPosition + BL.currentPosition + BR.currentPosition) / 4
        }

    val isBusy: Boolean
        get() {
            for (motor in driveMotors) {
                if (motor.isBusy) {
                    return true
                }
            }
            if (SLIDES.state == Slides.State.MOVING) {
                return true
            }
            return false
        }

    val botHeading: Double
        get() {
            val currentAngle: Double = radToDeg(IMU.angularOrientation.firstAngle.toFloat())

            var deltaAngle = currentAngle - lastAngle

            if (deltaAngle < -180) {
                deltaAngle += 360
            } else if (deltaAngle > 180) {
                deltaAngle -= 360
            }

            globalAngle += deltaAngle

            lastAngle = currentAngle

            return globalAngle
        }

    var zeroPowerBehavior: DcMotor.ZeroPowerBehavior
        get() {
            return FL.zeroPowerBehavior
        }
        set(value) {
            FL.zeroPowerBehavior = value
            FR.zeroPowerBehavior = value
            BL.zeroPowerBehavior = value
            BR.zeroPowerBehavior = value
        }

    var mode: DcMotor.RunMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        set(value) {
            FL.mode = value
            FR.mode = value
            BL.mode = value
            BR.mode = value
            field = value
        }

    val coneDetected: Boolean
        get() {
            return CONE_SENSOR.getDistance(DistanceUnit.INCH) <= 2.3 && abs(SLIDES.currentPosition) < 500
        }



    private var hardwareMap: HardwareMap? = null

    var headingPIDController = PIDController(heading_kP, heading_kI, heading_kD)
    var turnPIDController = PIDController(turn_kP, turn_kI, turn_kD)
    var drivePIDController = PIDController(drive_kP, drive_kI, drive_kD)

    fun funnyDrive(drive: Double, turn: Double){
        FL.power = drive + turn
        FR.power = drive - turn
        BL.power = drive + turn
        BR.power = drive - turn
    }

    fun RCDrive(y: Double, x: Double, rx: Double) {
        val x = x * strafeMultiplier

        //val denominator = max(abs(y) + abs(x) + abs(rx), 1.0).toDouble()

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        val leftFrontPower: Double = y + x + rx
        val rightFrontPower: Double = y - x - rx
        val leftBackPower: Double = y - x + rx
        val rightBackPower: Double = y + x - rx

        FL.power = leftFrontPower
        BL.power = leftBackPower
        FR.power = rightFrontPower
        BR.power = rightBackPower
    }

    fun FCDrive(y: Double, x: Double, turn: Double) {
        val x = x * strafeMultiplier
        val rotX = x * cos(-botHeading) - y * sin(-botHeading)
        val rotY = y * sin(-botHeading) + x * cos(-botHeading)

        val denominator = max(abs(y) + abs(x) + abs(turn), 1.0)

        FL.power = (rotY + rotX + turn) / denominator
        BL.power = (rotY - rotX + turn) / denominator
        FR.power = (rotY - rotX - turn) / denominator
        BR.power = (rotY + rotX - turn) / denominator
    }

    fun gamepadDrive(controller: Gamepad, multiplier: Double) {
        RCDrive(
            -controller.left_stick_y.toDouble() * multiplier,
            controller.left_stick_x.toDouble() * multiplier,
            controller.right_stick_x.toDouble() * multiplier
        )
    }

    fun update() {
        when (autoMode) {
            UNKNOWN -> {
                stop()
            }
            TURN -> {
                while (abs(errorAngle) > AutoTurnTolerance) {
                    errorAngle = angleTarget - botHeading

                    correction = turnPIDController.update(errorAngle)

                    funnyDrive(0.0, correction)
                }
                stop()
            }
            STRAIGHT -> {
                while (abs(distanceError) > AutoDriveTolerance) {
                    headingError = headingTarget - botHeading

                    headingCorrection = headingPIDController.update(headingError)

                    distanceError = distanceTarget - ticksToInches(currentPosition)

                    correction = drivePIDController.update(distanceError)

                    funnyDrive(correction, headingCorrection)
                }
                stop()
            }
        }
    }

    fun resetHeading(){
        globalAngle = 0.0
    }

    fun stop() {
        RCDrive(0.0, 0.0, 0.0)
        autoMode = UNKNOWN
    }

    fun prepareMotors(){
        for (motor in driveMotors){
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }
    fun straight(inches: Double){
        hasBeenRun = false
        prepareMotors()

        distanceTarget = inches

        distanceError = distanceTarget

        autoMode = STRAIGHT

        headingTarget = botHeading

        update()
    }

    fun turnRight(angle: Int = -90){
        turnPIDController.reset()

        autoMode = TURN

        resetHeading()

        angleTarget = -angle.toDouble()

        prepareMotors()

        update()
    }

    fun turnLeft(angle: Int = 90){
        turnPIDController.reset()

        autoMode = TURN

        resetHeading()

        angleTarget = angle.toDouble()

        prepareMotors()

        update()
    }
    fun closeClaw(claw: Servo = CLAW) {
        claw.position = DriveConstants.ClawClose
    }
    fun openClaw(claw: Servo = CLAW) {
        claw.position = DriveConstants.ClawOpen
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


        FL = hardwareMap!!.get(DcMotorEx::class.java, "FL")
        FR = hardwareMap!!.get(DcMotorEx::class.java, "FR")
        BL = hardwareMap!!.get(DcMotorEx::class.java, "BL")
        BR = hardwareMap!!.get(DcMotorEx::class.java, "BR")

        driveMotors = arrayOf(FL, FR, BL, BR)

        CLAW = hardwareMap!!.get(Servo::class.java, "CLAW")

        SLIDES = Slides(hardwareMap!!.get(DcMotorEx::class.java, "SLIDES"))

        CONE_SENSOR = hardwareMap!!.get(Rev2mDistanceSensor::class.java, "CONE_SENSOR")

        headingPIDController.setOutputBounds(-0.1, 0.1)
        turnPIDController.setOutputBounds(-1.0, 1.0)
        drivePIDController.setOutputBounds(-1.0, 1.0)


        FL.direction = DcMotorSimple.Direction.REVERSE
        BL.direction = DcMotorSimple.Direction.REVERSE

        //CLAW.direction = Servo.Direction.REVERSE

        FR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        FL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BR.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        BL.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        FR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        FL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        BR.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        BL.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        SLIDES.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        SLIDES.targetPosition = 0
        SLIDES.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        SLIDES.mode = DcMotor.RunMode.RUN_USING_ENCODER

        IMU = hardwareMap!!.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        IMU.initialize(parameters)
    }
}