package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Hajimi", group="Robot")
public class Hajimi extends LinearOpMode {

    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotorEx launch = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private Hajimi.LaunchState launchState;

    ElapsedTime feederTimer = new ElapsedTime();

    final double FEED_TIME_SECONDS = 0.6; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;


    final double TICKS_PER_REV = 537.7;
    final double MAX_RPM = 312;
    final double LAUNCHER_TARGET_RPM_ClOSE = 270;   // 目标转速 170
    final double LAUNCHER_MIN_RPM_ClOSE = 260;      // 判定可发射阈值 190
    final double LAUNCHER_TARGET_RPM_FAR = 280;
    final double LAUNCHER_MIN_RPM_FAR =270;

    double CURRENT_LAUNCHER_TARGET_RPM;
    double CURRENT_LAUNCHER_MIN_RPM;

    double targetTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_TARGET_RPM) / 60.0;
    double minTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_MIN_RPM) / 60.0;



//    final double WRIST_FOLDED_IN = 0.35;
//    final double WRIST_FOLDED_OUT = 0.925;
//    final double ARM_TICKS_PER_DEGREE = 19.79;

    //28 // number of encoder ticks per rotation of the bare motor
    //      * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
    //  * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
    //* 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    //Manual Driving
    private float y;
    private double x;
    private float rx;
    private double denominator;

    //
    private double Cam_x;
    private double Cam_y;
    private double Cam_z;
    private double Cam_range;
    private double Cam_bearing;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        READY_TO_FIRE,
        LAUNCH,
        LAUNCHING,
        ENDING,
    }

//    private static final boolean USE_WEBCAM = false;
//    private AprilTagProcessor aprilTag;
//    private VisionPortal visionPortal;


    //autoDrive for launchAdjust
//    private final double COUNTS_PER_MOTOR_REV = 538;
//    private final double DRIVE_GEAR_REDUCTION = 19.2;
//    private final double WHEEL_DIAMETER_CM = 4;
//    private final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * Math.PI);
//    private final double DRIVE_SPEED = 0.6;
//    private final double TURN_SPEED = 0.5;

    public void initialization() {
        launchState = Hajimi.LaunchState.IDLE;

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        arm = hardwareMap.get(DcMotor.class, "arm");
        launch = hardwareMap.get(DcMotorEx.class, "launch");

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        //initialization
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


//        ((DcMotorEx) arm).setCurrentAlert(5, CurrentUnit.AMPS);


//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        wrist = hardwareMap.get(Servo.class, "wrist");
//        wrist.setPosition(WRIST_FOLDED_IN);
        launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(BRAKE);
        launch.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(DcMotorSimple.Direction.FORWARD);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Initialization Done");
        telemetry.update();
    }

//    private void initAprilTag() {
//        WebcamName Camera = hardwareMap.get(WebcamName.class, "Webcam 1");
//        // Create the AprilTag processor.
//        aprilTag = new AprilTagProcessor.Builder()
//
//                // The following default settings are available to un-comment and edit as needed.
//                //.setDrawAxes(false)
//                //.setDrawCubeProjection(false)
//                //.setDrawTagOutline(true)
//                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
//                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
//                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//
//                // == CAMERA CALIBRATION ==
//                // If you do not manually specify calibration parameters, the SDK will attempt
//                // to load a predefined calibration for your camera.
//                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
//                // ... these parameters are fx, fy, cx, cy.
//
//
//                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11) // Tag family
//                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
//                .setDrawAxes(true)                                   //
//                .setDrawTagOutline(true)                             //
//                .setDrawCubeProjection(false)                        //
//                .setDrawTagID(true)                                  // ID
//                .build();
//
//        // Adjust Image Decimation to trade-off detection-range for detection-rate.
//        // eg: Some typical detection data using a Logitech C920 WebCam
//        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
//        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
//        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
//        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
//        // Note: Decimation can be changed on-the-fly to adapt during a match.
//        //aprilTag.setDecimation(3);
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        builder.enableLiveView(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(aprilTag);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Disable or re-enable the aprilTag processor at any time.
//        //visionPortal.setProcessorEnabled(aprilTag, true);
//
//    }   // end method initAprilTag()

    int shotCount = 0;
    final int MAX_SHOTS = 3;
    boolean lastShotButton = false; // 防止长按连发

    void launch(boolean shotRequested) {
        boolean newPress = shotRequested && !lastShotButton;
        lastShotButton = shotRequested;

        switch (launchState) {
            case IDLE:
                shotCount = 0;
                if (newPress) {
                    launchState = Hajimi.LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launch.setVelocity(targetTicksPerSec);
                if (Math.abs(launch.getVelocity()) > minTicksPerSec) {
                    launchState = Hajimi.LaunchState.READY_TO_FIRE;
                }
                break;

            case READY_TO_FIRE:
                if (newPress && shotCount < MAX_SHOTS) {
                    if (Math.abs(launch.getVelocity()) > minTicksPerSec) {
                        launchState = Hajimi.LaunchState.LAUNCH;
                    }
                } else if (shotCount >= MAX_SHOTS) { //因为是在发射过程中给count++，因此发射max次以后回到ready_to_fire后shotCount大于等于了MAX_SHOTS会被送到ending
                    launchState = Hajimi.LaunchState.ENDING;
                }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                shotCount++;
                launchState = Hajimi.LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = Hajimi.LaunchState.READY_TO_FIRE;
                }
                break;

            case ENDING:
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                launch.setPower(0);
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                shotCount = 0;
                launchState = Hajimi.LaunchState.IDLE;
                break;
        }
    }


    @Override
    public void runOpMode() {
        initialization();
//        initAprilTag();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x * 1.1;
                rx = -gamepad1.right_stick_x;
                denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
                frontLeft.setPower((y - x + rx)/ denominator);
                backLeft.setPower((y + x + rx) / denominator);
                frontRight.setPower((y + x - rx) / denominator);
                backRight.setPower((y - x - rx) / denominator);
//
//                frontLeft.setPower((y + x + rx)/ denominator);
//                backLeft.setPower((y - x + rx) / denominator);
//                frontRight.setPower((y + x - rx) / denominator);
//                backRight.setPower((y - x - rx) / denominator);

                boolean fireClose = gamepad1.right_bumper;
                boolean fireFar   = gamepad1.left_bumper;

                // 在调用 launch() 之前，先决定用哪一套参数
                if (fireFar) {
                    CURRENT_LAUNCHER_TARGET_RPM = LAUNCHER_TARGET_RPM_FAR;
                    CURRENT_LAUNCHER_MIN_RPM    = LAUNCHER_MIN_RPM_FAR;
                } else if (fireClose) {
                    CURRENT_LAUNCHER_TARGET_RPM = LAUNCHER_TARGET_RPM_ClOSE;
                    CURRENT_LAUNCHER_MIN_RPM    = LAUNCHER_MIN_RPM_ClOSE;
                }
                targetTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_TARGET_RPM) / 60.0;
                minTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_MIN_RPM) / 60.0;
                launch(fireFar || fireClose);


//                armPositionFudgeFactor = FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

//                arm.setTargetPosition((int) (armPosition + armPositionFudgeFactor));

//                ((DcMotorEx) arm).setVelocity(800);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* TECH TIP: Encoders, integers, and doubles
            Encoders report when the motor has moved a specified angle. They send out pulses which
            only occur at specific intervals (see our ARM_TICKS_PER_DEGREE). This means that the
            position our arm is currently at can be expressed as a whole number of encoder "ticks".
            The encoder will never report a partial number of ticks. So we can store the position in
            an integer (or int).
            A lot of the variables we use in FTC are doubles. These can capture fractions of whole
            numbers. Which is great when we want our arm to move to 122.5°, or we want to set our
            servo power to 0.5.

            setTargetPosition is expecting a number of encoder ticks to drive to. Since encoder
            ticks are always whole numbers, it expects an int. But we want to think about our
            arm position in degrees. And we'd like to be able to set it to fractions of a degree.
            So we make our arm positions Doubles. This allows us to precisely multiply together
            armPosition and our armPositionFudgeFactor. But once we're done multiplying these
            variables. We can decide which exact encoder tick we want our motor to go to. We do
            this by "typecasting" our double, into an int. This takes our fractional double and
            rounds it to the nearest whole number.
            */

            /* Check to see if our arm is over the current limit, and report via telemetry. *


            /* send telemetry to the driver of the arm's current position and target position */
//                telemetry.addData("armTarget: ", arm.getTargetPosition());
//                telemetry.addData("arm Encoder: ", arm.getCurrentPosition());


                telemetry.addLine("Locomotion encoder values:");
                telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
                telemetry.addData("frontRight", frontRight.getCurrentPosition());
                telemetry.addData("backLeft", backLeft.getCurrentPosition());
                telemetry.addData("backRight", backRight.getCurrentPosition());
                telemetry.addLine(" ");
                telemetry.addLine("Launch related values:");
                telemetry.addData("launchState", launchState);
                telemetry.addData("launchSpeed", launch.getVelocity());
                telemetry.update();


            }
        }
    }

//    private void launchAdjust() {
//
//        class AutoDrive {
//            void encoderDrive(double speed, double leftCM, double rightCM) {
//
//                    double backLeftTarget = backLeft.getCurrentPosition() + Math.floor(leftCM * COUNTS_PER_CM);
//                    double backRightTarget = backRight.getCurrentPosition() + Math.floor(rightCM * COUNTS_PER_CM);
//                    double frontLeftTarget = backLeft.getCurrentPosition() + Math.floor(leftCM * COUNTS_PER_CM);
//                    double frontRightTarget = backRight.getCurrentPosition() + Math.floor(rightCM * COUNTS_PER_CM);
//
//                    backLeft.setTargetPosition((int) backLeftTarget);
//                    backRight.setTargetPosition((int) backRightTarget);
//                    frontLeft.setTargetPosition((int) frontLeftTarget);
//                    frontRight.setTargetPosition((int) frontRightTarget);
//
//                    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//                    backLeft.setPower(Math.abs(speed));
//                    backRight.setPower(Math.abs(speed));
//                    frontLeft.setPower(Math.abs(speed));
//                    frontRight.setPower(Math.abs(speed));
//
//                    backLeft.setPower(0);
//                    backRight.setPower(0);
//                    frontLeft.setPower(0);
//                    frontRight.setPower(0);
//
//                    backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                    frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//        }
//        AutoDrive move = new AutoDrive();
//
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                double Cam_x = detection.ftcPose.x;
//                double Cam_y = detection.ftcPose.y;
//                double Cam_z = detection.ftcPose.z;
//                double Cam_range = detection.ftcPose.range;
//
//                double Cam_bearing = detection.ftcPose.bearing;
//                /*
//                x>0 ==> Tag is right x unit to the camera
//                y>0 ==> Tag is y unit in front of the camera
//                z>0 ==> Tag is z unit higher than the camera
//                Range = straight line distance
//                Bearing>0 ==> Bearing degree to the right
//                Elevation>0 ==> Elevation degree above the camera level
//                PRY is for navigation, much more advance than what we will need for this season
//                 */
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
////        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
////        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
////        telemetry.addLine("RBE = Range, Bearing & Elevation");
//
//        double designated_distance = 50; //in cm
//        double adjust_distance = Cam_range-designated_distance;
//        if (adjust_distance>0){
//            move.encoderDrive(DRIVE_SPEED, adjust_distance, adjust_distance);
//        } else if (adjust_distance<0) {
//            move.encoderDrive(DRIVE_SPEED, -adjust_distance, -adjust_distance);
//        }
//
//
//        double tolerance_x = 2.0;
//        double tolerance_angle = 2.0;
//
//        if (Math.abs(Cam_x) > tolerance_x) {
//            double turnDistance = Math.abs(Cam_x) * 0.5;
//            if (Cam_x > 0) {
//                move.encoderDrive(TURN_SPEED, turnDistance, -turnDistance);
//            } else {
//                move.encoderDrive(TURN_SPEED, -turnDistance, turnDistance);
//            }
//        } else if (Math.abs(Cam_bearing) > tolerance_angle) {
//            double turnDistance = Cam_bearing * 0.5;
//            if (Cam_bearing > 0) {
//                move.encoderDrive(TURN_SPEED, turnDistance, -turnDistance);
//            } else {
//                move.encoderDrive(TURN_SPEED, -turnDistance, turnDistance);
//            }
//        }
//
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        double LAUNCH_POWER = 1.0;
//        int LAUNCH_TIME_MS = 2000;
//
//        launch.setPower(LAUNCH_POWER);
//        sleep(LAUNCH_TIME_MS);
//        launch.setPower(0);
//    }


}

