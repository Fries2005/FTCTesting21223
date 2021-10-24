package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class Storage {
    public enum Alliance {
        RED(-1),
        BLUE(1);

        public final int value;

        Alliance(int value) {
            this.value = value;
        }
    }

    public static Alliance ALLIANCE = Alliance.BLUE;
    public static Pose2d CURRENT_POSE = new Pose2d(-60, ALLIANCE.value * 36);

    public static final Pose2d HIGH_GOAL_POSE = new Pose2d(-8, ALLIANCE.value * 36, Math.toRadians(180));


    public static final double LEFT_DISTANCE_OFFSET = 8.5;
    public static final double RIGHT_DISTANCE_OFFSET = 8.5;
    public static final double BACK_DISTANCE_OFFSET = 8.5;

    public static int ARM_DRIVE_POSITION = 0;
    public static final int ARM_DRIVE_TICKS = 5264;
    public static final double ARM_SERVO_CLOSE = 0;
    public static final double ARM_SERVO_OPEN = 1;
    public static boolean isArmClosed = false;
    public static ElapsedTime isArmClosedTime = new ElapsedTime();

    public static final double LAUNCHER_DRIVE_TICKS = 145.6;
    public static final double LAUNCHER_SERVO_PUSH = 0.9;
    public static final double LAUNCHER_SERVO_IDLE = 0;
    public static boolean isLauncherOn = false;
    public static ElapsedTime isLauncherOnTime = new ElapsedTime();
    public static boolean isLauncherPushed = false;
    public static ElapsedTime isLauncherPushedTime = new ElapsedTime();
    public static boolean isPowerShot = false;

    public static final double INTAKE_SERVO_CLOSE = 0.9;
    public static final double INTAKE_SERVO_OPEN = 0.1;
    public static boolean isIntakeOn = false;
    public static ElapsedTime isIntakeOnTime = new ElapsedTime();
    public static boolean isIntakeClosed = false;
    public static ElapsedTime isIntakeClosedTime = new ElapsedTime();

    public static void reset() {
        ARM_DRIVE_POSITION = 0;
        isArmClosed = false;
        isArmClosedTime = new ElapsedTime();

        isLauncherOn = false;
        isLauncherOnTime = new ElapsedTime();
        isLauncherPushed = false;
        isLauncherPushedTime = new ElapsedTime();

        isIntakeOn = false;
        isIntakeOnTime = new ElapsedTime();
    }
}