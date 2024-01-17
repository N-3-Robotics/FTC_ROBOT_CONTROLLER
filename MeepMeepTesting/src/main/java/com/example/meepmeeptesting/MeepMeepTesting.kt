package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim


internal enum class Alliance {
    RED,
    BLUE
}

internal enum class Position {
    CLOSE,
    FAR
}

object MeepMeepTesting {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(650)
        val ALLIANCE = Alliance.BLUE
        val POSITION = Position.FAR


        /* CONFIGURE STARTING POSE*/
        val StartPose: Pose2d = if (ALLIANCE == Alliance.BLUE && POSITION == Position.CLOSE) {
            Pose2d(12.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == Alliance.BLUE && POSITION == Position.FAR) {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        } else if (ALLIANCE == Alliance.RED && POSITION == Position.CLOSE) {
            Pose2d(12.0, -72.0 + 17 / 2, Math.toRadians(90.0))

        } else if (ALLIANCE == Alliance.RED && POSITION == Position.FAR) {
            Pose2d(-35.0, -72.0 + 17 / 2, Math.toRadians(90.0))
        } else {
            Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(90.0))
        }

        val color = if (ALLIANCE == Alliance.BLUE) 1 else -1
        val Mult = 2

        val myBot = DefaultBotBuilder(meepMeep) // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0) // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(ColorSchemeRedDark())
                .followTrajectorySequence { drive: DriveShim ->
                    drive.trajectorySequenceBuilder(StartPose)
                            .forward(48 - (17.0 / 2.0))
                            .lineTo(Vector2d(StartPose.x, 60.0))
                            .strafeLeft(3*24.0)


                            .lineToLinearHeading(Pose2d(36.0, 60.0 - (color*6) - 12.0, Math.toRadians(0.0)))
                            .forward(6.0)
                            .addDisplacementMarker {
                               println("whoop")
                            }
                            .back(6.0)

                            .lineTo(Vector2d(36.0, 60.0))
                            .forward(24.0)

                            .build()
                }

        meepMeep.setBackground(Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start()
    }
}