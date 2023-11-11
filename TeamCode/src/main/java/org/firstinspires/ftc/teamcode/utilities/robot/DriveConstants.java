package org.firstinspires.ftc.teamcode.utilities.robot;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

@Config
public class DriveConstants {


    public static final double TICKS_PER_REV = 537.7;

    // assuming all of this is right, what could cause the measured distance to be less that the actual distance travelled?
    public static final double WHEEL_SIZE = 48 / 25.4;

    public static final double INCHES_PER_REVOLUTION = 2 * WHEEL_SIZE * Math.PI;

    public static double getEncoderTicksFromInches(double inches) {
        return (inches / DriveConstants.INCHES_PER_REVOLUTION) * DriveConstants.TICKS_PER_REV;
    }

    public static double getInchesFromEncoderTicks(double ticks) {
        return (ticks / DriveConstants.TICKS_PER_REV) * DriveConstants.INCHES_PER_REVOLUTION;
    }

    public static double BANG_BANG_POWER = -0.3;
    public static double TICK_THRESHOLD = 50;
    public static double ANGLE_AT_TIME = 0;
    public static double MAX_TURN_TIME = 1.75;

    public static double TURN_THRESHOLD = Math.toRadians(2);
    public static double ANGULAR_VELOCITY_THRESHOLD = Math.toRadians(10);
    public static double ANGULAR_VELOCITY_THRESHOLD_MIN = Math.toRadians(1);

    public static Pose2d POSITION_THRESHOLD = new Pose2d(0.25, 0.25, TURN_THRESHOLD);
    public static double MAX_CORRECTION_TIME = 0.75;

    public static double MAX_VELOCITY = 55; // If doing 1
    public static double MAX_ACCELERATION = 35;

    public static double MAX_ANGULAR_VELOCITY = Math.toRadians(270);
    public static double trackWidth = 15.5;

}
