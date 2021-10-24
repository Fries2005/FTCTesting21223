
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/***
 * The Robot class
 * @author Neil Agrawal and Kaushik Siruvuri
 */

public class Robot {
//    public BNO055IMU imu;

    public DcMotorEx frontLeftDrive;
    public DcMotorEx backLeftDrive;
    public DcMotorEx backRightDrive;
    public DcMotorEx frontRightDrive;

    public DcMotorEx launcherDriveLeft;
    public DcMotorEx launcherDriveRight;
    public DcMotorEx intakeDrive;
    public DcMotorEx armDrive;

    public DistanceSensor rightDistanceSensor;
    public DistanceSensor leftDistanceSensor;
    //public DistanceSensor frontDistanceSensor;

    public Servo armServo;
    public Servo launcherServo;
    public Servo cameraServo;
    public Servo intakeServo;

    //private DcMotor.RunMode initialMode;

    HardwareMap map;


    /****
     * Sets the drive mode
     * @param enteredMode drive mode
     * @author Neil Agrawal
     */
    public Robot(DcMotor.RunMode enteredMode) {
        //initialMode = enteredMode;
    }

    /***
     * Initializes all of the robots motors, servos, and sensors
     * @param aMap hardwareMap
     * @author Neil Agrawal
     */
    public void init(HardwareMap aMap) {
        map = aMap;

//        imu = map.get(BNO055IMU.class, "imu");

//        //Initialize IMU parameters
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu.initialize(parameters);

        frontLeftDrive = map.get(DcMotorEx.class, "frontLeftDrive");
        backLeftDrive = map.get(DcMotorEx.class, "backLeftDrive");
        backRightDrive = map.get(DcMotorEx.class, "backRightDrive");
        frontRightDrive = map.get(DcMotorEx.class, "frontRightDrive");

        launcherDriveLeft = map.get(DcMotorEx.class, "launcherDriveLeft");
        launcherDriveRight = map.get(DcMotorEx.class, "launcherDriveRight");
        intakeDrive = map.get(DcMotorEx.class, "intakeDrive");
        armDrive = map.get(DcMotorEx.class, "armDrive");

        armServo = map.servo.get("armServo");
        launcherServo = map.servo.get("launcherServo");
        cameraServo = map.servo.get("cameraServo");
        intakeServo = map.servo.get("intakeServo");

        //rightDistanceSensor = map.get(DistanceSensor.class, "rightDistanceSensor");
        //leftDistanceSensor = map.get(DistanceSensor.class, "leftDistanceSensor");
        //frontDistanceSensor = map.get(DistanceSensor.class, "frontDistanceSensor");

//        //Encoders:
//        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        launcherDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        launcherDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set mode
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        launcherDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //set zero power mode
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherDriveLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherDriveRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set direction
        frontLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        launcherDriveLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        launcherDriveRight.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        armDrive.setDirection(DcMotorSimple.Direction.REVERSE);
    }

//    /**
//     * Gets the orientation of the robot using the REV IMU
//     *
//     * @return the angle of the robot
//     * @author Anand Vinod
//     */
//    public double getZAngle() {
//        return (-imu.getAngularOrientation().firstAngle);
//    }

    /***
     * Stops the robot by setting power to zero.
     * @author Neil Agrawal
     */
    public void stop() {
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        frontRightDrive.setPower(0);
    }

    /***
     * Sets the power for all motors to one value.
     * @param power power
     * @author Neil Agrawal
     */
    public void move(double power) {
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        backRightDrive.setPower(power);
        frontRightDrive.setPower(power);
    }

    /***
     * Sets the power for the left side and the right sie of the robot individually to produce turns.
     * @param leftPower Left-side power
     * @param rightPower Right-side power
     * @author Neil Agrawal
     */
    public void move(double leftPower, double rightPower) {
        frontLeftDrive.setPower(leftPower);
        backLeftDrive.setPower(leftPower);
        backRightDrive.setPower(rightPower);
        frontRightDrive.setPower(rightPower);
    }

    /***
     * Sets the power for all four motors individually to produce desired movement.
     * @param fr Front-right power
     * @param fl Front-left power
     * @param bl Back-left power
     * @param br Back-right power
     * @author Neil Agrawal
     */
    public void move(double fl, double bl, double br, double fr) {
        frontLeftDrive.setPower(fl);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br);
        frontRightDrive.setPower(fr);
    }

    /***
     * Makes the robot strafe left at the desired power
     * @param power power
     * @author Neil Agrawal
     */
    public void strafeLeft(double power) {
        backLeftDrive.setPower(power);
        backRightDrive.setPower(-power);
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
    }

    /***
     * Makes the robot strafe right at the desired power
     * @param power power
     * @author Neil Agrawal
     */
    public void strafeRight(double power) {
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
        frontLeftDrive.setPower(power);
        frontRightDrive.setPower(-power);
    }

    /***
     * Makes the robot strafe diagonally in the desired direction at the desired power
     * @param direction direction
     * @param power power
     * @author Neil Agrawal
     */
    public void strafeDiagonal(String direction, float power) {
        switch (direction) {
            case "NE":
                backLeftDrive.setPower(power);
                backRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(power);
                break;

            case "SE":
                backLeftDrive.setPower(0);
                backRightDrive.setPower(-power);
                frontLeftDrive.setPower(-power);
                frontRightDrive.setPower(0);
                break;

            case "SW":
                backLeftDrive.setPower(-power);
                backRightDrive.setPower(0);
                frontLeftDrive.setPower(0);
                frontRightDrive.setPower(-power);
                break;

            case "NW":
                backLeftDrive.setPower(0);
                backRightDrive.setPower(power);
                frontLeftDrive.setPower(power);
                frontRightDrive.setPower(0);
                break;
        }
    }

    /***
     * Complete TeleOpContorols
     * @param rightTrigger rightTrigger
     * @param leftTrigger leftTrigger
     * @param leftStick leftStick
     * @param rightBumper rightBumper
     * @param leftBumper leftBumper
     * @author Kaushik Siruvuri
     */
    public void teleControls(float rightTrigger, float leftTrigger, float[] leftStick, boolean rightBumper, boolean leftBumper/*, boolean yButton, boolean bButton*/) {
        float frontLeft = rightTrigger - leftTrigger;
        float backLeft = rightTrigger - leftTrigger;
        float frontRight = rightTrigger - leftTrigger;
        float backRight = rightTrigger - leftTrigger;
        if (rightTrigger == 0 && leftTrigger == 0 && !rightBumper && !leftBumper && leftStick[0] == 0 && leftStick[1] == 0) {
            stop();
        }

        if (rightTrigger != 0 && rightBumper) {
            strafeDiagonal("NE", 1);
            return;
        } else if (rightTrigger != 0 && leftBumper) {
            strafeDiagonal("NW", 1);
            return;
        } else if (leftTrigger != 0 && rightBumper) {
            strafeDiagonal("SE", 1);
            return;
        } else if (leftTrigger != 0 && leftBumper) {
            strafeDiagonal("SW", 1);
            return;
        } else if (rightBumper) {
            strafeRight(1);
            return;
        } else if (leftBumper) {
            strafeLeft(1);
            return;
        }

        if (leftStick[0] > 0) {
            if (frontLeft != 0 && backLeft != 0) {
                frontRight *= (1 - leftStick[0]);
                backRight *= (1 - leftStick[0]);
            } else {
                // Pivot
                frontLeft = leftStick[0];
                backLeft = leftStick[0];
                frontRight = -leftStick[0];
                backRight = -leftStick[0];
            }
        } else if (leftStick[0] < 0) {
            if (frontRight != 0 && backRight != 0) {
                frontLeft *= (1 + leftStick[0]);
                backLeft *= (1 + leftStick[0]);
            } else {
                // Pivot
                // Note that leftStick[0] is currently positive
                frontLeft = leftStick[0];
                backLeft = leftStick[0];
                frontRight = -leftStick[0];
                backRight = -leftStick[0];
            }
        }

        // Trigger controls y power
        // Left stick degree controls x movement power
        backLeftDrive.setPower(backLeft);
        backRightDrive.setPower(backRight);
        frontLeftDrive.setPower(frontLeft);
        frontRightDrive.setPower(frontRight);
    }
}