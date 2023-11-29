package org.firstinspires.ftc.teamcode.opmodes.teleops

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.robot.Robot

@TeleOp
class EncoderCheck: LinearOpMode() {

    override fun runOpMode() {
        val ROBOT = Robot(hardwareMap)

        waitForStart()
        while (opModeIsActive()) {
            telemetry.addData("LEFT", hardwareMap.get(DcMotorEx::class.java, "LIFT").currentPosition)
            telemetry.addData("RIGHT", hardwareMap.get(DcMotorEx::class.java, "RO").currentPosition)
            telemetry.update()
        }
    }
}