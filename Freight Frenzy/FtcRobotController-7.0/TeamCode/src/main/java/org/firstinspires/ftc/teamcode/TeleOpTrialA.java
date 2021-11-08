package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp

public class Tank_Drive extends LinearOpMode {
    private Blinker expansion_Hub_2;
    private Gyroscope imu;
    private DcMotor motorDown;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor motorUp;
    private Servo servoClasp;

    public float motor1_power(float jY, float jX, float rX) { return ((jX+jY)/2-rX/2);}
    public float motor2_power(float jY, float jX, float rX) { return ((jX-jY)/2+rX/2);}
    public float motor3_power(float jY, float jX, float rX) { return ((jX-jY)/2-rX/2);}
    public float motor4_power(float jY, float jX, float rX) { return ((jX+jY)/2+rX/2);}


    @Override
    public void runOpMode() {
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu = hardwareMap.get(Gyroscope.class, "imu");

        // Motor
        motor_front_right = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor_back_right = hardwareMap.get(DcMotor.class, "motor_back_right");
        motor_front_left = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor_back_left = hardwareMap.get(DcMotor.class, "motor_back_left");
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_right.setDirection(DcMotor.Direction.REVERSE);
        motor_front_left.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);

        servoClasp = hardwareMap.get(Servo.class, "servoClasp");
        telemetry.addData("Motor Status", "Initialized");
        telemetry.update()

        // Intake
        motor_intake = hardwareMap.get(DcMotor.class, "motor_intake");
        motor_expel = hardwareMap.get(DcMotor.class, "motor_expel");
        motor_outtake = hardwareMap.get(DcMotor.class, "motor_outtake");
        telemetry.addData("Intake Status", "Initialized");

        // Turntable
        motor_turntable = hardwareMap.get(DcMotor.class, "motor_turntable");
        telemetry.addData("Intake Status", "Initialized");

        // Cap stone
        motor_capstone_claw = hardwareMap.get(DcMotor.class, "motor_capstone_claw");
        motor_capstone_arm = hardwareMap.get(DcMotor.class, "motor_capstone_arm");

        waitForStart();
        // Motor
        double leftPower = 0;
        double rightPower = 0;

        //Intake
        int intake_level = 1;
        int intake_level_0 = -20;
        int intake_level_1 = 0;
        int intake_level_2 = 20;

        // Cap stone
        boolean arm_raised = false;
        boolean claw_gripping = false;

        while (opModeIsActive()) {
            // Motor
            this.gamepad1.left_stick_y-this.gamepad1.left_stick_x;
            this.gamepad1.left_stick_y+this.gamepad1.left_stick_x;
            motor_front_right.setPower(motor2_power(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,-gamepad1.left_stick_x));
            motor_back_right.setPower(motor4_power(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,-gamepad1.left_stick_x));
            motor_front_left.setPower(motor1_power(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,-gamepad1.left_stick_x));
            motor_back_left.setPower(motor3_power(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,-gamepad1.left_stick_x));
            telemetry.addData("Motor Status", "Running");

            //Intake
            if this.gamepad1.dpad.up {
                if intake_level == 1 {
                    // servo_target = intake_level_2
                    intake_level = 2;
                }
                if intake_level == 0 {
                    // servo_target = intake_level_1

                    intake_level = 1;
                }
            }
            if this.gamepad1.dpad.down {
                if intake_level == 1 {
                    // servo_target = intake_level_0
                    intake_level = 0;
                }
                if intake_level == 2 {
                    // servo_target = intake_level_1

                    intake_level = 1;
                }
            }
            // Expel and intake
            if this.gamepad1.right_trigger > 10 {
                motor_expel.setPower(100);
            }
            if this.gamepad1.left_trigger > 10 {
                motor_intake.setPower(100);
            }

            if this.gamepad1.a == true {
                motor_turntable.setPower(100);
            }

            //cap stone
            if this.gamepad1.x {
                if arm_raised == false {
                    // motor_capstone_arm set to raised
                }
                else {
                    // motor_capstone_arm set to bottom
                }
            }
            if this.gamepad1.y {
                if claw_gripping == false {
                    // motor_capstone_claw set to gripping
                }
                else {
                    // motor_capstone_claw set to open
                }
            }

            telemetry.addData("Status", "Running");
            telemetry.addData("Right", rightPower);
            telemetry.addData("Left", leftPower);
            telemetry.update();
        }
    }
}