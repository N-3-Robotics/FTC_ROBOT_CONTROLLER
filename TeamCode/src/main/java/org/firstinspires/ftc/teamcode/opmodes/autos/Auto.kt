package org.firstinspires.ftc.teamcode.opmodes.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.autos.Alliance.BLUE
import org.firstinspires.ftc.teamcode.opmodes.autos.Alliance.RED
import org.firstinspires.ftc.teamcode.opmodes.autos.Position.LEFT
import org.firstinspires.ftc.teamcode.opmodes.autos.Position.RIGHT
import org.firstinspires.ftc.teamcode.robot.Camera

private enum class Alliance {
    RED, BLUE
}
private enum class Position {
    LEFT, RIGHT
}

class Auto: LinearOpMode() {

    override fun runOpMode() {
        val ALLIANCE = BLUE
        val POSITION = RIGHT

        val CAMERA = Camera(hardwareMap);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d = Pose2d(12.0, 72.0, Math.toRadians(-90.0))



        /* CONFIGURE STARTING POSE*/
        if (ALLIANCE == BLUE && POSITION == LEFT) {
            StartPose = Pose2d(12.0, 72.0, Math.toRadians(-90.0))
        }
        else if (ALLIANCE == BLUE && POSITION == RIGHT) {
            StartPose = Pose2d(-36.0, 72.0, Math.toRadians(-90.0))
        }
        else if (ALLIANCE == RED && POSITION == LEFT) {
            StartPose = Pose2d(-36.0, 72.0, Math.toRadians(90.0))
        }
        else if (ALLIANCE == RED && POSITION == RIGHT) {
            StartPose = Pose2d(-36.0, 72.0, Math.toRadians(90.0))
        }
        /* END CONFIGURE STARTING POSE */



        /* SEQUENCES */
        val mkL = DRIVE.trajectoryBuilder(StartPose)
                .splineTo(Vector2d(0.0, 0.0), 0.0)
                .build()
        /* END SEQUENCES */

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
