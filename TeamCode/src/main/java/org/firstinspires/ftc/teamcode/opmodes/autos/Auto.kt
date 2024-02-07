package org.firstinspires.ftc.teamcode.opmodes.autos

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.opmodes.autos.Alliance.*
import org.firstinspires.ftc.teamcode.opmodes.autos.Position.*
import org.firstinspires.ftc.teamcode.robot.Camera
//import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder
import java.util.LinkedList
import java.util.Queue

enum class Alliance {
    RED, BLUE
}
enum class Position {
    CLOSE, FAR
}


// ALL PATHS ARE BASED ON THIS STARTING POSE
@Autonomous(name = "BLUE FAR")
class AutoBlueFar: LinearOpMode() {

    override fun runOpMode() {

        val LIFT_HEIGHT = 1800

        val RETURN_POS = 62.0

        val ALLIANCE = BLUE
        val POSITION = FAR

        val color = if (ALLIANCE == BLUE) {
            1
        } else {
            -1
        }

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);

        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        var Mult: Double = 2.0

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE
        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1
        WRIST.position = TestVars.WristLevelPos



        /* CONFIGURE STARTING POSE*/
        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == BLUE && POSITION == FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == RED && POSITION == FAR) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == RED && POSITION == CLOSE) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        }
        else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        }
        DRIVE.poseEstimate = StartPose
        DRIVE.updatePoseEstimate()
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(38 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(90.0))
                .forward(5.6)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(-90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(start.end())
                .forward(3.0)
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    RG.position = 0.0
                }
                .waitSeconds(1.0)
                .build()




        /* END SEQUENCES */

        while (!isStarted) {
            telemetry.addData("Cube Location", CAMERA.cubeLocation)
            telemetry.update()
        }

        waitForStart();

        pathToFollow.add(start)

        var after: TrajectorySequence = center
        when (CAMERA.cubeLocation) {
            "CENTER" -> {
                after = center
                pathToFollow.add(center)
                Mult = 1.80
            }
            "LEFT" -> {
                after = left
                pathToFollow.add(left)
                Mult = 0.77
            }
            "RIGHT" -> {
                after = right
                pathToFollow.add(right)
                Mult = 3.1
            }
        }

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(after.end())
                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
                .build()

        pathToFollow.add(returnToStartPos)
        val toBoard = if (POSITION == CLOSE) {
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(38.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                        telemetry.update()
                    }
                    .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS - (color * Mult * 6) - (color * 12.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        } else {

            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(38.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                    }
                    .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS -(color * Mult * 6) - (color * 12.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        }
        pathToFollow.add(toBoard)



        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }

        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
                .addDisplacementMarker(){
                    LG.position = 0.1
                }
                .back(14.0)
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    LG.position = 0.0
                    WRIST.position = TestVars.WristLevelPos
                    LIFT.targetPosition = 0
                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                    LIFT.velocity = 1000.0
                }
                .lineTo(Vector2d(42.0, color*RETURN_POS + 1))
                .UNSTABLE_addTemporalMarkerOffset(1.0){
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                }
                .forward(20.0)
                .build()

        DRIVE.followTrajectorySequence(park)

        stop()
    }
}

@Autonomous(name = "BLUE CLOSE")
class AutoBlueClose: LinearOpMode() {

    override fun runOpMode() {

        val LIFT_HEIGHT = 1700

        val RETURN_POS = 62.0

        val ALLIANCE = BLUE
        val POSITION = CLOSE

        val color = if (ALLIANCE == BLUE) {
            1
        } else {
            -1
        }

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        var Mult: Double = 2.0

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE
        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1
        WRIST.position = TestVars.WristLevelPos



        /* CONFIGURE STARTING POSE*/
        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == BLUE && POSITION == FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == RED && POSITION == FAR) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == RED && POSITION == CLOSE) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        }
        else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        }
        DRIVE.poseEstimate = StartPose
        DRIVE.updatePoseEstimate()
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(38 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(90.0))
                .forward(5.5)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(-90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(start.end())
                .forward(3.0)
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    RG.position = 0.0
                }
                .waitSeconds(1.0)
                .build()




        /* END SEQUENCES */
        /* FINISH TUNING NEXT TIME */
        while (!isStarted) {
            telemetry.addData("Cube Location", CAMERA.cubeLocation)
            telemetry.update()
        }

        waitForStart();

        pathToFollow.add(start)

        var after: TrajectorySequence = center
        when (CAMERA.cubeLocation) {
            "CENTER" -> {
                after = center
                pathToFollow.add(center)
                Mult = 1.80
            }
            "LEFT" -> {
                after = left
                pathToFollow.add(left)
                Mult = 0.70 // NEEDS FURTHER TUNING
            }
            "RIGHT" -> {
                after = right
                pathToFollow.add(right)
                Mult = 3.1
            }
        }

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
                .build()

        pathToFollow.add(returnToStartPos)
        val toBoard = if (POSITION == CLOSE) {
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(38.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                        telemetry.update()
                    }
                    .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        } else {

            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(38.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                    }
                    .lineToLinearHeading(Pose2d(38.0, color * RETURN_POS -(color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        }
        pathToFollow.add(toBoard)



        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }

        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
                .addDisplacementMarker(){
                    LG.position = 0.1
                }
                .back(14.0)
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    LG.position = 0.0
                    WRIST.position = TestVars.WristLevelPos
                    LIFT.targetPosition = 0
                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                    LIFT.velocity = 1000.0
                }
                .lineTo(Vector2d(42.0, color*RETURN_POS + 1))
                .UNSTABLE_addTemporalMarkerOffset(1.0){
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                }
                .forward(24.0)
                .build()

        DRIVE.followTrajectorySequence(park)

        stop()
    }
}

@Autonomous(name = "RED FAR")
class AutoRedFar: LinearOpMode() {

    override fun runOpMode() {

        val LIFT_HEIGHT = 1600

        val RETURN_POS = 60.0

        val ALLIANCE = RED
        val POSITION = FAR

        val color = if (ALLIANCE == BLUE) {
            1
        } else {
            -1
        }

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        var Mult: Int = 2

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE
        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1
        WRIST.position = TestVars.WristLevelPos



        /* CONFIGURE STARTING POSE*/
        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == BLUE && POSITION == FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == RED && POSITION == FAR) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == RED && POSITION == CLOSE) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        }
        else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        }
        DRIVE.poseEstimate = StartPose
        DRIVE.updatePoseEstimate()
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(38 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(-90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(start.end())
                .forward(3.0)
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    RG.position = 0.0
                }
                .waitSeconds(1.0)
                .build()




        /* END SEQUENCES */

        while (!isStarted) {
            telemetry.addData("Cube Location", CAMERA.cubeLocation)
            telemetry.update()
        }

        waitForStart();

        pathToFollow.add(start)

        var after: TrajectorySequence = center
        when (CAMERA.cubeLocation) {
            "CENTER" -> {
                after = center
                pathToFollow.add(center)
                Mult = 2
            }
            "LEFT" -> {
                after = left
                pathToFollow.add(left)
                Mult = 3
            }
            "RIGHT" -> {
                after = right
                pathToFollow.add(right)
                Mult = 1
            }
        }

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
                .build()

        pathToFollow.add(returnToStartPos)
        val toBoard = if (POSITION == CLOSE) {
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(40.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                        telemetry.update()
                    }
                    .lineToLinearHeading(Pose2d(36.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(20.0)
                    .build()
        } else {

            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(40.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                    }
                    .lineToLinearHeading(Pose2d(40.0, color * RETURN_POS -(color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        }
        pathToFollow.add(toBoard)



        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }

        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
                .addDisplacementMarker(){
                    LG.position = 0.1
                }
                .back(14.0)
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    LG.position = 0.0
                    WRIST.position = TestVars.WristLevelPos
                    LIFT.targetPosition = 0
                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                    LIFT.velocity = 800.0
                }
                .lineTo(Vector2d(42.0, color*RETURN_POS))
                .UNSTABLE_addTemporalMarkerOffset(1.0){
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                }
                .forward(24.0)
                .build()

        DRIVE.followTrajectorySequence(park)

        stop()
    }
}

@Autonomous(name = "RED CLOSE")
class AutoRedClose: LinearOpMode() {

    override fun runOpMode() {

        val LIFT_HEIGHT = 1600

        val RETURN_POS = 62.0

        val ALLIANCE = RED
        val POSITION = CLOSE

        val color = if (ALLIANCE == BLUE) {
            1
        } else {
            -1
        }

        val CAMERA = Camera(hardwareMap, telemetry, ALLIANCE);

        val DRIVE = SampleMecanumDrive(hardwareMap);
        var StartPose: Pose2d
        var pathToFollow: Queue<TrajectorySequence> = LinkedList()

        var LIFT: DcMotorEx
        var WRIST: Servo
        var LG: Servo
        var RG: Servo

        var Mult: Int = 2

        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
        LIFT.direction = DcMotorSimple.Direction.REVERSE
        LIFT.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
        LIFT.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
        LG = hardwareMap!!.get(Servo::class.java, "LG")
        RG = hardwareMap!!.get(Servo::class.java, "RG")

        LG.position = 0.0
        RG.position = 0.1
        WRIST.position = TestVars.WristLevelPos



        /* CONFIGURE STARTING POSE*/
        StartPose = if (ALLIANCE == BLUE && POSITION == CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == BLUE && POSITION == FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == RED && POSITION == FAR) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else if (ALLIANCE == RED && POSITION == CLOSE) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        }
        else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        }
        DRIVE.poseEstimate = StartPose
        DRIVE.updatePoseEstimate()
        /* END CONFIGURE STARTING POSE */


        /* SEQUENCES */
        val start = DRIVE.trajectorySequenceBuilder(StartPose)
                .forward(38 - (17.0 / 2.0))
                .build()

        val left = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val right = DRIVE.trajectorySequenceBuilder(start.end())
                .turn(Math.toRadians(-90.0))
                .forward(6.0)
                .addDisplacementMarker {
                    RG.position = 0.0
                }
                .back(8.0)
                .lineToLinearHeading(Pose2d(StartPose.x, StartPose.y - (color*(42-(17.0/2.0))), Math.toRadians(color*-90.0)))
                .build()

        val center = DRIVE.trajectorySequenceBuilder(start.end())
                .forward(3.0)
                .waitSeconds(1.0)
                .addDisplacementMarker{
                    RG.position = 0.0
                }
                .waitSeconds(1.0)
                .build()




        /* END SEQUENCES */

        while (!isStarted) {
            telemetry.addData("Cube Location", CAMERA.cubeLocation)
            telemetry.update()
        }

        waitForStart();

        pathToFollow.add(start)

        var after: TrajectorySequence = center
        when (CAMERA.cubeLocation) {
            "CENTER" -> {
                after = center
                pathToFollow.add(center)
                Mult = 2
            }
            "LEFT" -> {
                after = left
                pathToFollow.add(left)
                Mult = 3
            }
            "RIGHT" -> {
                after = right
                pathToFollow.add(right)
                Mult = 1
            }
        }

        val returnToStartPos = DRIVE.trajectorySequenceBuilder(center.end())
                .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))
                .build()

        pathToFollow.add(returnToStartPos)
        val toBoard = if (POSITION == CLOSE) {
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(40.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                        telemetry.update()
                    }
                    .lineToLinearHeading(Pose2d(40.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
        } else {
            /* RESIDUAL.  DISREGARD THIS SECTION */
            DRIVE.trajectorySequenceBuilder(returnToStartPos.end())
                    .lineTo(Vector2d(40.0, color*RETURN_POS))
                    .addDisplacementMarker {
                        RG.position = 0.1
                        LIFT.targetPosition = LIFT_HEIGHT
                        LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                        LIFT.velocity = 800.0
                    }
                    .lineToLinearHeading(Pose2d(36.0, color * RETURN_POS - (color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                    .addDisplacementMarker{
                        WRIST.position = TestVars.WristTop
                    }
                    .forward(16.0)
                    .build()
            /*---------------------------------*/
        }
        pathToFollow.add(toBoard)



        telemetry.addData("Status", "Running");
        telemetry.update();


        while (!pathToFollow.isEmpty()) {
            DRIVE.followTrajectorySequence(pathToFollow.poll())
        }

        val park = DRIVE.trajectorySequenceBuilder(toBoard.end())
                .addDisplacementMarker(){
                    LG.position = 0.1
                }
                .back(14.0)
                .UNSTABLE_addTemporalMarkerOffset(1.0) {
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                    LG.position = 0.0
                    WRIST.position = TestVars.WristLevelPos
                    LIFT.targetPosition = 0
                    LIFT.mode = DcMotor.RunMode.RUN_TO_POSITION
                    LIFT.velocity = 800.0
                }
                .lineTo(Vector2d(42.0, color*RETURN_POS))
                .UNSTABLE_addTemporalMarkerOffset(1.0){
                    LIFT.velocity = 0.0
                    LIFT.mode = DcMotor.RunMode.RUN_USING_ENCODER
                }
                .forward(24.0)
                .build()

        DRIVE.followTrajectorySequence(park)

        stop()
    }
}


//@Autonomous(name = "Arm Test")
//class armTest: LinearOpMode() {
//    override fun runOpMode() {
//        var LIFT: DcMotorEx
//        var WRIST: Servo
//        var LG: Servo
//        var RG: Servo
//
//        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
//        LIFT.direction = DcMotorSimple.Direction.REVERSE
//
//        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
//        LG = hardwareMap!!.get(Servo::class.java, "LG")
//        RG = hardwareMap!!.get(Servo::class.java, "RG")
//
//
//        LG.position = 0.0
//        RG.position = 0.1
//        waitForStart()
//
//        LIFT.power = 1.0
//        sleep(700)
//        LIFT.power = 0.05
//        WRIST.position = TestVars.WristTop
//        sleep(1500)
//        LG.position = 0.1
//        sleep(1000)
//        LG.position = 0.0
//        WRIST.position = TestVars.WristLevelPos
//        sleep(1000)
//        LIFT.power = -0.8
//        sleep(700)
//
//    }
//}

//}
//@Autonomous(name = "RoadRunner Arm Test")
//class RoadArmTest: LinearOpMode() {
//    override fun runOpMode() {
//        var LIFT: DcMotorEx
//        var WRIST: Servo
//        var LG: Servo
//        var RG: Servo
//        var StartPose: Pose2d
//        val DRIVE = SampleMecanumDrive(hardwareMap);
//        StartPose = Pose2d(0.0,0.0,Math.toRadians(0.0))
//
//        var pathToFollow: Queue<TrajectorySequence> = LinkedList()
//
//        LIFT = hardwareMap!!.get(DcMotorEx::class.java, "LIFT")
//        LIFT.direction = DcMotorSimple.Direction.REVERSE
//
//        WRIST = hardwareMap!!.get(Servo::class.java, "WRIST")
//        LG = hardwareMap!!.get(Servo::class.java, "LG")
//        RG = hardwareMap!!.get(Servo::class.java, "RG")
//
//        LG.position = 0.0
//        RG.position = 0.1
//
//        val backTen = DRIVE.trajectorySequenceBuilder(StartPose)
//                .back(10.0)
//                .build()
//        fun liftToPosition(){
//            LIFT.power = 1.0
//            sleep(700)
//            LIFT.power = 0.05
//            WRIST.position = TestVars.WristTop
//            sleep(1500)
//        }
//
//        val forwardTen = DRIVE.trajectorySequenceBuilder(StartPose)
//                .forward(10.0)
//                .build()
//
//        fun releasePixel() {
//            LG.position = 0.1
//            sleep(1000)
//        }
//
//        fun lowerArm() {
//            LG.position = 0.0
//            WRIST.position = TestVars.WristLevelPos
//            sleep(1500)
//            LIFT.power = -0.8
//            sleep(700)
//            LIFT.power = 0.0
//
//        }
//        val sequence = DRIVE.trajectorySequenceBuilder(StartPose)
//                .back(10.0)
//                .addDisplacementMarker{
//                    LIFT.power = 1.0
//                    sleep(700)
//                }
//                .addDisplacementMarker{
//                    LIFT.power = 0.05
//                    WRIST.position = TestVars.WristTop
//                    sleep(1500)
//                }
//                .forward(10.0)
//                .addDisplacementMarker{
//                    LG.position = 0.1
//                    sleep(1000)
//                }
//                .forward(-10.0)
//                .addDisplacementMarker{
//                    LG.position = 0.0
//                    WRIST.position = TestVars.WristLevelPos
//                    sleep(1000)
//                }
//                .addDisplacementMarker{
//                    LIFT.power = -0.8
//                    sleep(800)
//                }
//                .addDisplacementMarker{
//                    LIFT.power = 0.0
//                }
//                .build()
//        val liftAbovePixel = DRIVE.trajectorySequenceBuilder(StartPose)
//                .addDisplacementMarker{
//                    LIFT.power = 1.0
//                    sleep(100)
//                }
//                .waitSeconds(0.1)
//                .addDisplacementMarker{
//                    LIFT.power = 0.0
//                }
//                .build()
//        waitForStart()
//        DRIVE.followTrajectorySequence(liftAbovePixel)
//        /*
//        DRIVE.followTrajectorySequence(backTen)
//        liftToPosition()
//        DRIVE.followTrajectorySequence(forwardTen)
//        releasePixel()
//        DRIVE.followTrajectorySequence(backTen)
//        lowerArm()
//         */
//
//
//    }
//}