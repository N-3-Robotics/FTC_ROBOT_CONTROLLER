package org.firstinspires.ftc.teamcode.opmodes.autos


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.robot.Camera

@Autonomous(name = "AUTO")
class AutoBlueFar: LinearOpMode() {


    override fun runOpMode() {

        // Initialize your Robot.  Any functions called on this object will be executed on the robot.
        // NOTE: This SampleMecanumDrive is different from the ROBOT class in the TeleOp example.
        val DRIVE = SampleMecanumDrive(hardwareMap)

        // Initialize the camera.  Be sure your camera is plugged in and configured, and that your pipelines are set up.
        val CAMERA = Camera(hardwareMap, telemetry);

        /*
        Make sure you update the robots starting position so that the localizer knows where the robot is on the field.
        For example:
            val startPose = Pose2d(-63.0, 24.0, Math.toRadians(180.0))
            DRIVE.poseEstimate = startPose
            DRIVE.updatePoseEstimate()


         */


        while (!isStarted) {
            // TODO: Display any telemetry information you want.
            // Will loop until you press the Start button.
            telemetry.update()
        }

        waitForStart();

        /*
        This is where you will put your autonomous code.
        You can use the DRIVE object to build trajectories for your robot.

        Depending on how you want to structure your code, you can either
        build your sequences (trajectories) and then immediately send them to the robot,

            DRIVE.followTrajectorySequence(DRIVE.trajectoryBuilder(DRIVE.poseEstimate)
                .splineTo(Vector2d(0.0, 0.0), 0.0)
                .build()
            )

        Or you can build a queue of sequences and then send them to the robot one at a time.

                var pathToFollow: Queue<TrajectorySequence> = LinkedList()
                val start = DRIVE.trajectorySequenceBuilder(StartPose)
                            .forward(38 - (17.0 / 2.0))
                            .build()

                pathToFollow.add(start)

                val center = DRIVE.trajectorySequenceBuilder(start.end())
                            .forward(3.0)
                            .waitSeconds(1.0)
                            .addDisplacementMarker{
                                RG.position = 0.0
                            }
                            .waitSeconds(1.0)
                            .build()

                pathToFollow.add(center)

                while (!pathToFollow.isEmpty()) {
                    DRIVE.followTrajectorySequence(pathToFollow.poll())
                }
        */
        stop() // Not necessary, but it's good practice to stop the opmode when you're done.
    }
}