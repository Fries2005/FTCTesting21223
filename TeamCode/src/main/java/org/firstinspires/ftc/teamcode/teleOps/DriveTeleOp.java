package org.firstinspires.ftc.teamcode.teleOps;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PresidentialDrive;
import org.firstinspires.ftc.teamcode.PresidentialSystems;
import org.firstinspires.ftc.teamcode.Storage;

@TeleOp(group = "drive")
public class DriveTeleOp extends LinearOpMode {
    // Define 2 states, drive control or automatic control
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    double forwardHeading = Math.toRadians(180);

//    Pose2d leftPowershotPose = new Pose2d(1, Storage.ALLIANCE.value * 33, forwardHeading);
//    Pose2d midPowershotPose = new Pose2d(1, Storage.ALLIANCE.value * 25, forwardHeading);
//    Pose2d rightPowershotPose = new Pose2d(1, Storage.ALLIANCE.value * 17, forwardHeading);
//    Pose2d towerPose = new Pose2d(-14, Storage.ALLIANCE.value * 36, forwardHeading);

    @Override
    public void runOpMode() throws InterruptedException {
        PresidentialDrive drive = new PresidentialDrive(hardwareMap);
        PresidentialSystems systems = new PresidentialSystems(hardwareMap);

        //drive.setPoseEstimate(Storage.CURRENT_POSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            //drive.update();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
//            telemetry.addData("x", drive.getPoseEstimate().getX());
//            telemetry.addData("y", drive.getPoseEstimate().getY());
//            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addData("powershot mode:", Storage.isPowerShot);
            telemetry.update();

            // We follow different logic based on whether we are in manual driver control or switch
            // control to the automatic mode
            switch (currentMode) {
                case DRIVER_CONTROL:
                    float[] leftStick = {gamepad1.left_stick_x, gamepad1.left_stick_y};

                    drive.teleControls(gamepad1.right_trigger, gamepad1.left_trigger, leftStick, gamepad1.right_bumper, gamepad1.left_bumper);

                    systems.teleControls(gamepad2);

//                    if (gamepad1.b) {
//                        drive.setPoseEstimate(new Pose2d(
//                                -1 * (72.0 - Storage.BACK_DISTANCE_OFFSET),
//                                Storage.ALLIANCE.value * (72.0 - Storage.RIGHT_DISTANCE_OFFSET),
//                                Math.toRadians(270)));
//                    } else if (gamepad1.a) {
//                        Trajectory towerTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(towerPose)
//                                .build();
//
//                        drive.followTrajectory(towerTrajectory);
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.dpad_left) {
//                        Trajectory leftPowershotTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(leftPowershotPose)
//                                .build();
//
//                        drive.followTrajectory(leftPowershotTrajectory);
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.dpad_down) {
//                        Trajectory midPowershotTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(midPowershotPose)
//                                .build();
//
//                        drive.followTrajectory(midPowershotTrajectory);
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    } else if (gamepad1.dpad_right) {
//                        Trajectory rightPowershotTrajectory = drive.trajectoryBuilder(drive.getPoseEstimate())
//                                .lineToSplineHeading(rightPowershotPose)
//                                .build();
//
//                        drive.followTrajectory(rightPowershotTrajectory);
//                        currentMode = Mode.AUTOMATIC_CONTROL;
//                    }
//                    break;
//
//                case AUTOMATIC_CONTROL:
//                    // If X is pressed, we break out of the automatic following
//                    if (gamepad1.x) {
//                        drive.cancelFollowing();
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }
//
//                    // If drive finishes its task, cede control to the driver
//                    if (!drive.isBusy()) {
//                        currentMode = Mode.DRIVER_CONTROL;
//                    }
//                    break;
            }
        }
    }
}
