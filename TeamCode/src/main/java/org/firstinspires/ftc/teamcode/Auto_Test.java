package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Autonomous(name = "Auto_Test", group = "Sensor")
public class Auto_Test extends LinearOpMode{

    double targetX = 20;   // cm
    double targetY = 20;   // cm
    double targetHeading = 90; // degrees

    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
//    private DcMotor arm = null;
    private DcMotor launch = null;
    //    private CRServo intake = null;
    //private Servo wrist = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    private GoBildaPinpointDriver pinpoint;

    public void initialization() {

        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        launch = hardwareMap.get(DcMotor.class, "launch");

//      arm = hardwareMap.get(DcMotor.class, "arm");
        configurePinpoint();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0));


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
//        ((DcMotorEx) arm).setCurrentAlert(5, CurrentUnit.AMPS);


//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        /* Make sure that the intake is off, and the wrist is folded in. */
//        wrist = hardwareMap.get(Servo.class, "wrist");
//        wrist.setPosition(WRIST_FOLDED_IN);


        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();
    }



    @Override
    public void runOpMode() {

        initialization();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                 class Drive {
                    void drive(double forward, double strafe, double turn) {
                        double denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(forward), Math.abs(strafe), Math.abs(turn))), 1));
                        double fl = (forward - strafe + turn);
                        double bl = forward + strafe + turn;
                        double fr = forward - strafe - turn;
                        double br = forward + strafe - turn;



                        frontLeft.setPower(fl / denominator);
                        backLeft.setPower(bl / denominator);
                        frontRight.setPower(fr / denominator);
                        backRight.setPower(br / denominator);
                    }
                }
                Drive move = new Drive();

                pinpoint.update();
                Pose2D pose = pinpoint.getPosition();

                // Current Position
                double x = pose.getX(DistanceUnit.CM);
                double y = pose.getY(DistanceUnit.CM);
                double heading = pose.getHeading(AngleUnit.DEGREES);

                telemetry.addData("X coordinate (CM)", pose.getX(DistanceUnit.CM));
                telemetry.addData("Y coordinate (Cm)", pose.getY(DistanceUnit.CM));
                telemetry.addData("Heading angle (DEGREES)", pose.getHeading(AngleUnit.DEGREES));

                // Calculate Difference in World Coordinate
                double dx = targetX - x;
                double dy = targetY - y;

                // Distance and Angle Difference from Target Position
                double distance = Math.hypot(dx, dy);
                double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

                // Calculate World Coordinate Angle Error
                double angleError = targetAngle - heading;
                angleError = AngleUnit.normalizeDegrees(angleError);

                // Convert World Coordinate into Mecanum Wheel Coordinate
//                double driveX = Math.cos(Math.toRadians(angleError)) * distance;
//                double driveY = Math.sin(Math.toRadians(angleError)) * distance;
                double driveX =  dx*Math.cos(Math.toRadians(heading)) + dy*Math.sin(Math.toRadians(heading));
                double driveY = -dx*Math.sin(Math.toRadians(heading)) + dy*Math.cos(Math.toRadians(heading));
   
                // Calculate Final Heading Error for final adjustment after reaching Target Position
                double headingError = AngleUnit.normalizeDegrees(targetHeading - heading);

                // PID
                double kP_drive = 0.01;    
                double kP_turn = 0.01;

                double forwardPower = kP_drive * driveY;   // Forward
                double strafePower = kP_drive * driveX;   // Left or Right
                double turnPower = kP_turn * headingError;


                forwardPower = Range.clip(forwardPower, -0.5, 0.5);
                strafePower = Range.clip(strafePower, -0.5, 0.5);
                turnPower = Range.clip(turnPower, -0.3, 0.3);

                // Drive
                move.drive(forwardPower, strafePower, turnPower);


                if (distance < 2 && Math.abs(headingError) < 5) {
                    break;
                }
            }
        }
    }
    public void configurePinpoint() {

        pinpoint.setOffsets(
                /* Left/Right offset */ -168.0,
                /* Forward/Backward offset */ -84.0,
                DistanceUnit.MM);
        //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }
}
