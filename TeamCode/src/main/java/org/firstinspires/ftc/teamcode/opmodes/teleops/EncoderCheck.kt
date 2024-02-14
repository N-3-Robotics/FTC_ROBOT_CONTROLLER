package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.robot.Robot


@Disabled
@TeleOp(name = "CLAW CHECK")
class EncoderCheck: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        ROBOT.LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ROBOT.LIFT.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        waitForStart()
        while (opModeIsActive()) {

            ROBOT.LIFT.power = -gamepad1.right_stick_y.toDouble()

            telemetry.addData("LEFT", hardwareMap.get(DcMotorEx::class.java, "LIFT").currentPosition)
            telemetry.addData("RIGHT", hardwareMap.get(DcMotorEx::class.java, "RO").currentPosition)
            telemetry.update()
        }
    }
}

@TeleOp(name = "DIS CHECK")
class DisCheck: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("DISTANCE", ROBOT.DIS.getDistance(DistanceUnit.INCH))
            telemetry.update()
            sleep(500)
        }
    }
}