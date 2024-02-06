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

        val RETURN_POS = 62.0

        val StartPose = Pose2d(-35.0, 72.0 - 17 / 2, Math.toRadians(-90.0))
        val color = 1
        val Mult = 2

        val myBot = DefaultBotBuilder(meepMeep) // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(47.49731874210789 * 0.6, 30.0, Math.toRadians(134.45), Math.toRadians(134.45), 15.0) // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(ColorSchemeRedDark())
                .followTrajectorySequence { drive: DriveShim ->
                    drive.trajectorySequenceBuilder(StartPose)
                            .forward(38 - (17.0 / 2.0))

                            .forward(3.0)
                            .waitSeconds(1.0)
                            .waitSeconds(1.0)

                            .lineToLinearHeading(Pose2d(StartPose.x, color*RETURN_POS, StartPose.heading))

                            .lineTo(Vector2d(40.0, color*RETURN_POS))
                            .lineToLinearHeading(Pose2d(40.0, color * RETURN_POS -(color * Mult * 6) - (color * 14.0), Math.toRadians(0.0)))
                            .forward(16.0)

                            .back(14.0)
                            .lineTo(Vector2d(42.0, color*RETURN_POS))
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