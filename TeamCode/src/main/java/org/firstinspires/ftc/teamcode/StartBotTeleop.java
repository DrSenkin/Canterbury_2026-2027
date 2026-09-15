

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleop", group = "StarterBot")
//@Disabled
public class StartBotTeleop extends OpMode {
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */

    // Declare OpMode members.
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotorEx launch = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    ElapsedTime feederTimer = new ElapsedTime();

    final double TICKS_PER_REV = 537.7;
    final double MAX_RPM = 312;
    final double LAUNCHER_TARGET_RPM_ClOSE = 170;   // 目标转速
    final double LAUNCHER_MIN_RPM_ClOSE = 150;      // 判定可发射阈值
    final double LAUNCHER_TARGET_RPM_FAR = 210;
    final double LAUNCHER_MIN_RPM_FAR = 190;

    double CURRENT_LAUNCHER_TARGET_RPM;
    double CURRENT_LAUNCHER_MIN_RPM;

    double targetTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_TARGET_RPM) / 60.0;
    double minTicksPerSec = (TICKS_PER_REV * CURRENT_LAUNCHER_MIN_RPM) / 60.0;

    // 在初始化或发射逻辑中：


    private float y;
    private double x;
    private float rx;
    private double denominator;


    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        READY_TO_FIRE,
        LAUNCH,
        LAUNCHING,
        ENDING,
    }

    private LaunchState launchState;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");

        launch = hardwareMap.get(DcMotorEx.class, "launch");

        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);



//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.REVERSE);
//        backRight.setDirection(DcMotor.Direction.FORWARD);


        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        launch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        launch.setZeroPowerBehavior(BRAKE);

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launch.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x * 1.1;
        rx = gamepad1.right_stick_x;
        denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

//        frontLeft.setPower((y - x + rx) / denominator);
//        backLeft.setPower((y + x + rx) / denominator);
//        frontRight.setPower((y - x - rx) / denominator);
//        backRight.setPower((y + x - rx) / denominator);

        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);




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


        telemetry.addLine("Locomotion encoder values:");
        telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
        telemetry.addData("frontRight", frontRight.getCurrentPosition());
        telemetry.addData("backLeft", backLeft.getCurrentPosition());
        telemetry.addData("backRight", backRight.getCurrentPosition());
        telemetry.addLine(" ");
        telemetry.addLine("Launch related values:");
        telemetry.addData("launchState", launchState);
        telemetry.addData("launchSpeed", launch.getVelocity());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



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
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                launch.setVelocity(targetTicksPerSec);
                if (Math.abs(launch.getVelocity()) > minTicksPerSec) {
                    launchState = LaunchState.READY_TO_FIRE;
                }
                break;

            case READY_TO_FIRE:
                if(newPress && shotCount < MAX_SHOTS) {
                    if (Math.abs(launch.getVelocity()) > minTicksPerSec) {
                        launchState = LaunchState.LAUNCH;
                    }
                }
                else if (shotCount >= MAX_SHOTS){ //因为是在发射过程中给count++，因此发射max次以后回到ready_to_fire后shotCount大于等于了MAX_SHOTS会被送到ending
                    launchState = LaunchState.ENDING;
                    }
                break;

            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                shotCount++;
                launchState = LaunchState.LAUNCHING;
                break;

            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.READY_TO_FIRE;
                }
                break;

            case ENDING:
                try{
                    Thread.sleep(1000);
                }catch (InterruptedException e){
                    e.printStackTrace();
                }
                launch.setPower(0);
                leftFeeder.setPower(0);
                rightFeeder.setPower(0);
                shotCount = 0;
                launchState = LaunchState.IDLE;
                break;
        }
    }

}
