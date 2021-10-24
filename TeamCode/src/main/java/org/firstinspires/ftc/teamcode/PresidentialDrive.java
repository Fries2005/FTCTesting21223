package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

//import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;
//import org.firstinspires.ftc.teamcode.roadrunner.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import androidx.annotation.NonNull;

import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.DriveConstants.kV;

@Config
public class PresidentialDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(7, 0, 0);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(7, 0, 0);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double OMEGA_WEIGHT = 1;

    public static int POSE_HISTORY_LIMIT = 100;

    public enum Mode {
        IDLE,
        TURN,
        FOLLOW_TRAJECTORY
    }

//    private FtcDashboard dashboard;
    private NanoClock clock;

    private Mode mode;

    private PIDFController turnController;
    private MotionProfile turnProfile;
    private double turnStart;

    private TrajectoryVelocityConstraint velConstraint;
    private TrajectoryAccelerationConstraint accelConstraint;
    private TrajectoryFollower follower;

    private LinkedList<Pose2d> poseHistory;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private BNO055IMU imu;

    private VoltageSensor batteryVoltageSensor;

    private Pose2d lastPoseOnTurn;

    public PresidentialDrive(HardwareMap hardwareMap) {
        //super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

//        dashboard = FtcDashboard.getInstance();
//        dashboard.setTelemetryTransmissionInterval(25);

        clock = NanoClock.system();

        mode = Mode.IDLE;

        turnController = new PIDFController(HEADING_PID);
        turnController.setInputBounds(0, 2 * Math.PI);

        velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
        accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        poseHistory = new LinkedList<>();

//        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

//        // TODO: adjust the names of the following hardware devices to match your configuration
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
//        imu.initialize(parameters);
//
//        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
//        // upward (normal to the floor) using a command like the following:
//        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
//        BNO055IMUUtil.remapAxes(imu, AxesOrder.ZYX, AxesSigns.NPN);

        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        leftRear = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        rightRear = hardwareMap.get(DcMotorEx.class, "backRightDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightDrive");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
       // setLocalizer(new PresidentialLocalizer(hardwareMap));
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, velConstraint, accelConstraint);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, velConstraint, accelConstraint);
    }

//    public void turnAsync(double angle) {
//        double heading = getPoseEstimate().getHeading();
//
//        lastPoseOnTurn = getPoseEstimate();
//
//        turnProfile = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(heading, 0, 0, 0),
//                new MotionState(heading + angle, 0, 0, 0),
//                MAX_ANG_VEL,
//                MAX_ANG_ACCEL
//        );
//
//        turnStart = clock.seconds();
//        mode = Mode.TURN;
//    }

//    public void turn(double angle) {
//        turnAsync(angle);
//        waitForIdle();
//    }

//    public void followTrajectoryAsync(Trajectory trajectory) {
//        follower.followTrajectory(trajectory);
//        mode = Mode.FOLLOW_TRAJECTORY;
//    }

//    public void followTrajectory(Trajectory trajectory) {
//        followTrajectoryAsync(trajectory);
//        waitForIdle();
//    }

    public void cancelFollowing() {
        mode = Mode.IDLE;
    }

    public Pose2d getLastError() {
        switch (mode) {
            case FOLLOW_TRAJECTORY:
                return follower.getLastError();
            case TURN:
                return new Pose2d(0, 0, turnController.getLastError());
            case IDLE:
                return new Pose2d();
        }
        throw new AssertionError();
    }

//    public void update() {
//        updatePoseEstimate();
//
//        Pose2d currentPose = getPoseEstimate();
//        Pose2d lastError = getLastError();
//
//        poseHistory.add(currentPose);
//
//        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
//            poseHistory.removeFirst();
//        }
//
//        TelemetryPacket packet = new TelemetryPacket();
//        Canvas fieldOverlay = packet.fieldOverlay();
//
//        packet.put("mode", mode);
//
//        packet.put("x", currentPose.getX());
//        packet.put("y", currentPose.getY());
//        packet.put("heading (deg)", Math.toDegrees(currentPose.getHeading()));
//
//        packet.put("xError", lastError.getX());
//        packet.put("yError", lastError.getY());
//        packet.put("headingError (deg)", Math.toDegrees(lastError.getHeading()));
//
//        switch (mode) {
//            case IDLE:
//                // do nothing
//                break;
//            case TURN: {
//                double t = clock.seconds() - turnStart;
//
//                MotionState targetState = turnProfile.get(t);
//
//                turnController.setTargetPosition(targetState.getX());
//
//                double correction = turnController.update(currentPose.getHeading());
//
//                double targetOmega = targetState.getV();
//                double targetAlpha = targetState.getA();
//                setDriveSignal(new DriveSignal(new Pose2d(
//                        0, 0, targetOmega + correction
//                ), new Pose2d(
//                        0, 0, targetAlpha
//                )));
//
//                Pose2d newPose = lastPoseOnTurn.copy(lastPoseOnTurn.getX(), lastPoseOnTurn.getY(), targetState.getX());
//
//                fieldOverlay.setStroke("#4CAF50");
//                DashboardUtil.drawRobot(fieldOverlay, newPose);
//
//                if (t >= turnProfile.duration()) {
//                    mode = Mode.IDLE;
//                    setDriveSignal(new DriveSignal());
//                }
//
//                break;
//            }
//            case FOLLOW_TRAJECTORY: {
//                setDriveSignal(follower.update(currentPose, getPoseVelocity()));
//
//                Trajectory trajectory = follower.getTrajectory();
//
//                fieldOverlay.setStrokeWidth(1);
//                fieldOverlay.setStroke("#4CAF50");
//                DashboardUtil.drawSampledPath(fieldOverlay, trajectory.getPath());
//                double t = follower.elapsedTime();
//                DashboardUtil.drawRobot(fieldOverlay, trajectory.get(t));
//
//                fieldOverlay.setStroke("#3F51B5");
//                DashboardUtil.drawPoseHistory(fieldOverlay, poseHistory);
//
//                if (!follower.isFollowing()) {
//                    mode = Mode.IDLE;
//                    setDriveSignal(new DriveSignal());
//                }
//
//                break;
//            }
//        }
//
//        fieldOverlay.setStroke("#3F51B5");
//        DashboardUtil.drawRobot(fieldOverlay, currentPose);
//
//        dashboard.sendTelemetryPacket(packet);
//    }

//    public void waitForIdle() {
//        while (!Thread.currentThread().isInterrupted() && isBusy()) {
//            update();
//        }
//    }

    public boolean isBusy() {
        return mode != Mode.IDLE;
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

//    public void setWeightedDrivePower(Pose2d drivePower) {
//        Pose2d vel = drivePower;
//
//        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
//                + Math.abs(drivePower.getHeading()) > 1) {
//            // re-normalize the powers according to the weights
//            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
//                    + VY_WEIGHT * Math.abs(drivePower.getY())
//                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());
//
//            vel = new Pose2d(
//                    VX_WEIGHT * drivePower.getX(),
//                    VY_WEIGHT * drivePower.getY(),
//                    OMEGA_WEIGHT * drivePower.getHeading()
//            ).div(denom);
//        }
//
//        setDrivePower(vel);
//    }
//
//    @NonNull
//    @Override
//    public List<Double> getWheelPositions() {
//        List<Double> wheelPositions = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
//        }
//        return wheelPositions;
//    }
//
//    @Override
//    public List<Double> getWheelVelocities() {
//        List<Double> wheelVelocities = new ArrayList<>();
//        for (DcMotorEx motor : motors) {
//            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
//        }
//        return wheelVelocities;
//    }
//
//    @Override
//    public void setMotorPowers(double v, double v1, double v2, double v3) {
//        leftFront.setPower(v);
//        leftRear.setPower(v1);
//        rightRear.setPower(v2);
//        rightFront.setPower(v3);
//    }

//    @Override
//    public double getRawExternalHeading() {
//        return imu.getAngularOrientation().firstAngle;
//    }
//

    // Robot stuff

    /***
     * Stops the robot by setting power to zero.
     * @author Neil Agrawal
     */
    public void stop() {
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    /***
     * Sets the power for all motors to one value.
     * @param power power
     * @author Neil Agrawal
     */
    public void move(double power) {
        leftFront.setPower(power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        rightFront.setPower(power);
    }

    /***
     * Sets the power for the left side and the right sie of the robot individually to produce turns.
     * @param leftPower Left-side power
     * @param rightPower Right-side power
     * @author Neil Agrawal
     */
    public void move(double leftPower, double rightPower) {
        leftFront.setPower(leftPower);
        leftRear.setPower(leftPower);
        rightRear.setPower(rightPower);
        rightFront.setPower(rightPower);
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
        leftFront.setPower(fl);
        leftRear.setPower(bl);
        rightRear.setPower(br);
        rightFront.setPower(fr);
    }

    /***
     * Makes the robot strafe left at the desired power
     * @param power power
     * @author Neil Agrawal
     */
    public void strafeLeft(double power) {
        leftRear.setPower(power);
        rightRear.setPower(-power);
        leftFront.setPower(-power);
        rightFront.setPower(power);
    }

    /***
     * Makes the robot strafe right at the desired power
     * @param power power
     * @author Neil Agrawal
     */
    public void strafeRight(double power) {
        leftRear.setPower(-power);
        rightRear.setPower(power);
        leftFront.setPower(power);
        rightFront.setPower(-power);
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
                leftRear.setPower(power);
                rightRear.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(power);
                break;

            case "SE":
                leftRear.setPower(0);
                rightRear.setPower(-power);
                leftFront.setPower(-power);
                rightFront.setPower(0);
                break;

            case "SW":
                leftRear.setPower(-power);
                rightRear.setPower(0);
                leftFront.setPower(0);
                rightFront.setPower(-power);
                break;

            case "NW":
                leftRear.setPower(0);
                rightRear.setPower(power);
                leftFront.setPower(power);
                rightFront.setPower(0);
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

        if (rightTrigger != 0 && rightBumper) { //
            strafeDiagonal("NW", 1);
            return;
        } else if (rightTrigger != 0 && leftBumper) {
            strafeDiagonal("NE", 1);
            return;
        } else if (leftTrigger != 0 && rightBumper) {
            strafeDiagonal("SW", 1);
            return;
        } else if (leftTrigger != 0 && leftBumper) {
            strafeDiagonal("SE", 1);
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
        leftRear.setPower(backLeft);
        rightRear.setPower(backRight);
        leftFront.setPower(frontLeft);
        rightFront.setPower(frontRight);
    }
}
