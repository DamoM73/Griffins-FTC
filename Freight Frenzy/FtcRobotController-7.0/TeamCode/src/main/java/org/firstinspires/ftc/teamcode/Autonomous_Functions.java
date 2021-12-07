package org.firstinspires.ftc.compcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous

public class Autonomous_Functions extends LinearOpMode {
    // Create objects for this robot
    private Blinker expansion_Hub_2;
    //private Gyroscope imu;
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;
    //private DcMotorEx arm_motor;

    // convert count per revolution to counts per cm
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_CIRCUMFERENCE_MM = 10 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM = DRIVE_COUNTS_PER_MM * 1;

    private ElapsedTime
            runtime = new ElapsedTime();


    public float motor1_power(float jY, float jX, float rX) {
        return ((jX + jY)/2 - rX /2);
    }

    public float motor2_power(float jY, float jX, float rX) {
        return ((jY - jX)/2 + rX /2);
    }

    public float motor3_power(float jY, float jX, float rX) {
        return ((jY - jX)/2 - rX /2);
    }

    public float motor4_power(float jY, float jX, float rX) {
        return ((jX + jY)/2 + rX /2);
    }

    public void motorFwdTargetPositions (float cmDistance) {
        // set target positions when travelling forward (all +)
        int motor1Target = (int)motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);
        int motor2Target = (int)motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);
        int motor3Target = (int)motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);
        int motor4Target = (int)motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);



        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        telemetry.addData("FR", motor_front_right.getTargetPosition());
        telemetry.addData("FL", motor_front_left.getTargetPosition());
        telemetry.addData("BR", motor_back_right.getTargetPosition());
        telemetry.addData("BL", motor_back_left.getTargetPosition());
        telemetry.update();
    }

    public void motorBwdTargetPositions (float cmDistance) {
        // set target positions when travelling backward
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }

    public void motorRgtTargetPositions (float cmDistance) {
        // set target positions when driving right (fr -, bl -)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }

    public void motorLftTargetPositions (float cmDistance) {
        // set target positions when driving left (fl -, br -)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }

    public void motorFwdRgtTargetPositions (float cmDistance) {
        // set target positions when driving diagonally right and forward (fr 0, bl 0)
        int motor1Target = (int)motor_front_right.getCurrentPosition();
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor4Target = (int)motor_back_left.getCurrentPosition();


        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }

    public void motorFwdLftTargetPositions (float cmDistance) {
        // set target positions when driving diagonally left forward (fl 0, br 0)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor2Target = (int)motor_front_left.getCurrentPosition();
        int motor3Target = (int)motor_back_right.getCurrentPosition();
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }

    public void motorBwdRgtTargetPositions (float cmDistance) {
        // set target positions when driving diagonally right backward (fr -1, bl -1, fl 0, br 0)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)motor_front_left.getCurrentPosition();
        int motor3Target = (int)motor_back_right.getCurrentPosition();
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }

    public void motorBwdLftTargetPositions (float cmDistance) {
        // set target positions when driving diagonally left backward (fr 0, bl 0, fl -1, br -1)
        int motor1Target = (int)motor_front_right.getCurrentPosition();
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)motor_back_left.getCurrentPosition();

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
    }


    public void runMotors(double speed) {
        // sets power of motors so that they will run to position
        motor_front_right.setPower(speed);
        motor_back_right.setPower(speed);
        motor_front_left.setPower(speed);
        motor_back_left.setPower(speed);

        // wait while motors are running
        while (opModeIsActive() && (motor_front_right.isBusy() || motor_front_left.isBusy() || motor_back_right.isBusy() || motor_back_left.isBusy())) {
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }


    @Override
    public void runOpMode() {
        /*
        SETUP
        Initialises all the required variables and objects and initialises them
        ready for the start();
        */


        // initialise objects for expansion hub components
        //expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");
        //arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        //arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);

        motor_front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorBwdRgtTargetPositions(10);
        //changes the motor modes to RUN_TO_POSITION
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run during autonomous
        if (opModeIsActive()) {

            runMotors(0.05);



            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                /*
                RUNNING
                This section loops endlessly until the program has been stopped
                */

                // sets the test_motor to test_motor_power
                motor_front_right.setPower(motor2_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
                motor_back_right.setPower(motor4_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
                motor_front_left.setPower(motor1_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));
                motor_back_left.setPower(motor3_power(gamepad1.right_stick_y*-1, gamepad1.right_stick_x, -gamepad1.left_stick_x));

                /*if(gamepad1.a == true) {
                    arm_motor.setTargetPosition(0);
                    arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm_motor.setVelocity(50);
                }

                if(gamepad1.b == true) {
                    arm_motor.setTargetPosition(-38);
                    arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm_motor.setVelocity(200);
                }*/

                //display current motor power to the driver station
                telemetry.update();
            }
        }
    }
}


