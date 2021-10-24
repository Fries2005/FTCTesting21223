package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Storage;

public class PresidentialSystems {
    public DcMotorEx launcherDriveLeft;
    public DcMotorEx launcherDriveRight;
    public DcMotorEx intakeDrive;
    public DcMotorEx armDrive;

    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    public DistanceSensor backDistanceSensor;

    public Servo armServo;
    public Servo launcherServo;
    public Servo intakeServo;
    public Servo cameraServo;

    HardwareMap map;

    /***
     * Initializes all of the robots motors, servos, and sensors
     * @param map hardwareMap
     * @author Neil Agrawal
     */
    public PresidentialSystems(HardwareMap map) {
        this.map = map;

        launcherDriveLeft = map.get(DcMotorEx.class, "launcherDriveLeft");
        launcherDriveRight = map.get(DcMotorEx.class, "launcherDriveRight");
        intakeDrive = map.get(DcMotorEx.class, "intakeDrive");
        armDrive = map.get(DcMotorEx.class, "armDrive");

        armServo = map.servo.get("armServo");
        launcherServo = map.servo.get("launcherServo");
        intakeServo = map.servo.get("intakeServo");
        cameraServo = map.servo.get("cameraServo");

        rightDistanceSensor = map.get(DistanceSensor.class, "rightDistanceSensor");
        leftDistanceSensor = map.get(DistanceSensor.class, "leftDistanceSensor");
        backDistanceSensor = map.get(DistanceSensor.class, "backDistanceSensor");

        //set mode
        launcherDriveLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcherDriveRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //set zero power mode
        launcherDriveLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        launcherDriveRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        armDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //set direction
        launcherDriveLeft.setDirection(DcMotorEx.Direction.FORWARD);
        launcherDriveRight.setDirection(DcMotorEx.Direction.REVERSE);
        intakeDrive.setDirection(DcMotorEx.Direction.FORWARD);
        armDrive.setDirection(DcMotorEx.Direction.REVERSE);

        //armDrive.setTargetPositionTolerance(0);
    }

    public void reset() {
        armServo.setPosition(Storage.ARM_SERVO_OPEN);
        launcherServo.setPosition(Storage.LAUNCHER_SERVO_IDLE);
        intakeServo.setPosition(Storage.INTAKE_SERVO_OPEN);
        //cameraServo.setPosition();
        armDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void teleControls(Gamepad gamepad2) {
        //Arm
        armDrive.setPower(-gamepad2.right_stick_y);

        if (gamepad2.a && Storage.isArmClosedTime.milliseconds() > 500) {
            Storage.isArmClosed = !Storage.isArmClosed;
            Storage.isArmClosedTime.reset();
        }

        if (Storage.isArmClosed) {
            armServo.setPosition(Storage.INTAKE_SERVO_OPEN);
        }
        if (!Storage.isArmClosed) {
            armServo.setPosition(Storage.INTAKE_SERVO_CLOSE);
        }
//
//        if (gamepad2.right_bumper && Storage.isLauncherPushedTime.milliseconds() > 500) {
//            Storage.isLauncherPushed = !Storage.isLauncherPushed;
//            Storage.isLauncherPushedTime.reset();
//        }
//
//        if (Storage.isLauncherPushed) {
//            armServo.setPosition(Storage.LAUNCHER_SERVO_PUSH);
//        }
//        if (!Storage.isLauncherPushed) {
//            armServo.setPosition(Storage.LAUNCHER_SERVO_IDLE);
//        }

        if (gamepad2.right_bumper && Storage.isLauncherPushedTime.milliseconds() > 600) {
            Storage.isLauncherPushed = true;
            Storage.isLauncherPushedTime.reset();
        }

        if (Storage.isLauncherPushed && Storage.isLauncherOn) {
            launcherServo.setPosition(Storage.LAUNCHER_SERVO_PUSH);
            Storage.isLauncherPushed = false;
        }
        if (Storage.isLauncherPushedTime.milliseconds() > 300)
            launcherServo.setPosition(Storage.LAUNCHER_SERVO_IDLE);

        //Intake
        intakeDrive.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        //Launcher
        if (gamepad2.left_bumper && Storage.isLauncherOnTime.milliseconds() > 250) {
            Storage.isLauncherOn = !Storage.isLauncherOn;
            Storage.isLauncherOnTime.reset();
        }

        if (gamepad2.y && Storage.isLauncherOnTime.milliseconds() > 250) {
            Storage.isPowerShot = !Storage.isPowerShot;
            Storage.isLauncherOnTime.reset();
        }

        if (Storage.isLauncherOn) {
            if (!Storage.isPowerShot) {
                launcherDriveLeft.setVelocity((1125.0 / 60.0) * Storage.LAUNCHER_DRIVE_TICKS);
                launcherDriveRight.setVelocity((1125.0 / 60.0) * Storage.LAUNCHER_DRIVE_TICKS);
            } else {
                launcherDriveLeft.setVelocity((1050.0 / 60.0) * Storage.LAUNCHER_DRIVE_TICKS);
                launcherDriveRight.setVelocity((1050.0 / 60.0) * Storage.LAUNCHER_DRIVE_TICKS);
            }
        }

        if (!Storage.isLauncherOn) {
            launcherDriveLeft.setVelocity(0);
            launcherDriveRight.setVelocity(0);
        }


    }
}
