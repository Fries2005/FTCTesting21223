package org.firstinspires.ftc.teamcode.teleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "BasicTeleOp", group = "basic")

public class BasicTeleOp extends OpMode {
    Robot robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    @Override
    public void init(){
        robot.init(hardwareMap);

    }

    @Override
    public void loop(){

        float[] leftStick = {gamepad1.left_stick_x, gamepad1.left_stick_y};

        robot.teleControls(gamepad1.right_trigger, gamepad1.left_trigger, leftStick, gamepad1.right_bumper, gamepad1.left_bumper);

        telemetry.addData("Top Left Pow", robot.frontLeftDrive.getPower());
        telemetry.addData("Bottom Left Pow", robot.backLeftDrive.getPower());
        telemetry.addData("Top Right Pow", robot.frontRightDrive.getPower());
        telemetry.addData("Bottom Right Pow", robot.backRightDrive.getPower());
        telemetry.update();

    }
}
