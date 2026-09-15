package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Autonomous(name = "AutoFarBoth_NotLaunch",group="Robot")
public class AutoFarBoth_NotLaunch extends LinearOpMode {

    final double FEED_TIME_SECONDS = 1; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;

    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

//    public DcMotor arm = null;
//    public CRServo intake = null;
//    public Servo wrist = null;

    double COUNTS_PER_INCH;
    ElapsedTime runtime;

    double backLeftTarget;
    double backRightTarget;
    double frontLeftTarget;
    double frontRightTarget;

    final double WRIST_FOLDED_IN = 0.35;
    final double WRIST_FOLDED_OUT = 0.925;//folds out to grab

    ElapsedTime feederTimer = new ElapsedTime();

    private DcMotorEx launch = null;

    final double TICKS_PER_REV = 537.7;
    final double MAX_RPM = 312;
    final double LAUNCHER_TARGET_RPM_ClOSE = 230;   // 目标转速
    final double LAUNCHER_MIN_RPM_ClOSE = 225;    // 判定可发射阈值
    final double LAUNCHER_TARGET_RPM_FAR = 210;
    final double LAUNCHER_MIN_RPM_FAR = 190;

    double CURRENT_LAUNCHER_TARGET_RPM;
    double CURRENT_LAUNCHER_MIN_RPM;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        READY_TO_FIRE,
        LAUNCH,
        LAUNCHING,
        ENDING,
    }

    private AutoFarBoth_NotLaunch.LaunchState launchState;

    @Override
    public void runOpMode() {
        int COUNTS_PER_MOTOR_REV;
        double DRIVE_GEAR_REDUCTION;
        int WHEEL_DIAMETER_INCHES;
        double DRIVE_SPEED;
        double TURN_SPEED;

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        launch = hardwareMap.get(DcMotorEx.class, "launch");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");



//        arm = hardwareMap.get(DcMotor.class, "arm");
//        extend = hardwareMap.get(DcMotor.class, "extend");
//        wrist  = hardwareMap.get(Servo.class, "wrist");

        COUNTS_PER_MOTOR_REV = 538;
        DRIVE_GEAR_REDUCTION = 1; //CPR=(PPR*4)(GEAR_RATIO)=(28*4)(19.2/1)=537.7, including 19.2/1 ratio
        WHEEL_DIAMETER_INCHES = 4;
        COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
        DRIVE_SPEED = 0.6;
        TURN_SPEED = 0.5;
        runtime = new ElapsedTime();

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        frontLeft.setDirection(DcMotor.Direction.REVERSE);
//        frontRight.setDirection(DcMotor.Direction.FORWARD);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);

//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setTargetPosition(200);

//        while (!(arm.getCurrentPosition() == arm.getTargetPosition())) {
//            arm.setPower(0.5);
//        }

        telemetry.addData("Status", "Initialized");
        // wristAction(WRIST_FOLDED_IN);
        telemetry.addData("Starting at", JavaUtil.formatNumber(backLeft.getCurrentPosition(), 7, 0) + " : " + JavaUtil.formatNumber(backRight.getCurrentPosition(), 7, 0));

        telemetry.update();

        waitForStart();


        encoderDrive(DRIVE_SPEED, 25, 25, 20);
//        try {
//            Thread.sleep(500);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        autoShoot(3,true,false);
//        encoderDrive(DRIVE_SPEED, 10, 10,20);
//        encoderDrive(TURN_SPEED, 15, -15, 20 );
//        encoderDrive(DRIVE_SPEED, -10, -10, 20);
        // armPosition(250);
        // intakeAction(1);
        // wristAction(WRIST_FOLDED_OUT);


        telemetry.update();
    }

    private void encoderDrive(double speed, int leftInches, int rightInches, int timeoutS) {
        if (opModeIsActive()) {
            backLeftTarget = (backLeft.getCurrentPosition() + Math.floor(leftInches * COUNTS_PER_INCH));
            backRightTarget = (backRight.getCurrentPosition() + Math.floor(rightInches * COUNTS_PER_INCH));
            frontLeftTarget = (frontLeft.getCurrentPosition() + Math.floor(leftInches * COUNTS_PER_INCH));
            frontRightTarget = (frontRight.getCurrentPosition() + Math.floor(rightInches * COUNTS_PER_INCH));

            backLeft.setTargetPosition((int) backLeftTarget);
            backRight.setTargetPosition((int) backRightTarget);
            frontLeft.setTargetPosition((int) frontLeftTarget);
            frontRight.setTargetPosition((int) frontRightTarget);


            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            backLeft.setPower(speed);
            backRight.setPower(speed);
            frontLeft.setPower(speed);
            frontRight.setPower(speed);

            while (opModeIsActive() && runtime.seconds() < timeoutS && backLeft.isBusy() && backRight.isBusy() && frontLeft.isBusy() && frontRight.isBusy()) {
                telemetry.addData("Running to",
                        String.format("%d : %d : %d : %d",
                                (int) backLeftTarget,
                                (int) backRightTarget,
                                (int) frontLeftTarget,
                                (int) frontRightTarget));
                telemetry.addData("Currently at",
                        String.format("%d : %d : %d : %d",
                                backLeft.getCurrentPosition(),
                                backRight.getCurrentPosition(),
                                frontLeft.getCurrentPosition(),
                                frontRight.getCurrentPosition()));
                telemetry.update();
            }

            backLeft.setPower(0);
            backRight.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);
        }
    }

    //    private void armPosition(int ticks){
//        if (opModeIsActive()){
//            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            arm.setTargetPosition(ticks);
//            while (!(arm.getCurrentPosition() == arm.getTargetPosition())) {
//                arm.setPower(0.5);
//            }
//        }
//    }
//    private void intakeAction(int intakeState){
//        if (opModeIsActive()){
//            int INTAKE_TIME;
//            for (INTAKE_TIME = 1; INTAKE_TIME <= 3; INTAKE_TIME++) {
//                intake.setPower(intakeState);
//            }
//            intake.setPower(0);
//        }
//    }
//    private void wristAction(double wristState){
//        if (opModeIsActive()){
//            wrist.setPosition(wristState);
//        }
//    }

    private void autoShoot(int shots, boolean fireClose, boolean fireFar) {

        if (fireFar) {
            CURRENT_LAUNCHER_TARGET_RPM = LAUNCHER_TARGET_RPM_FAR;
            CURRENT_LAUNCHER_MIN_RPM    = LAUNCHER_MIN_RPM_FAR;
        } else if (fireClose) {
            CURRENT_LAUNCHER_TARGET_RPM = LAUNCHER_TARGET_RPM_ClOSE;
            CURRENT_LAUNCHER_MIN_RPM    = LAUNCHER_MIN_RPM_ClOSE;
        }


        double targetTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_TARGET_RPM) / 60.0;
        double minTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_MIN_RPM) / 60.0;

        launch.setVelocity(targetTicksPerSec);
        sleep(360);
        for (int i = 0; i < shots && opModeIsActive(); i++) {
            while (opModeIsActive() &&
                    Math.abs(launch.getVelocity()) < minTicksPerSec) {
                telemetry.addData("launchSpeed", launch.getVelocity());
                telemetry.update();
            }

            telemetry.addData("launchSpeed", launch.getVelocity());
            telemetry.update();

            leftFeeder.setPower(FULL_SPEED);
            rightFeeder.setPower(FULL_SPEED);

            sleep((long)(FEED_TIME_SECONDS * 1000));

            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);

            sleep(360);
            telemetry.addData("launchSpeed", launch.getVelocity());
            telemetry.update();
        }

        telemetry.addData("launchSpeed", launch.getVelocity());
        telemetry.update();
        launch.setPower(0);
        telemetry.addData("launchSpeed", launch.getVelocity());
        telemetry.update();
    }

}
