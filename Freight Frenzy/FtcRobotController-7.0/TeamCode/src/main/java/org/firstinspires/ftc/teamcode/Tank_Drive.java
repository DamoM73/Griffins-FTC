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

    @Override
    public void runOpMode() {
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        motor_front_right = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor_back_right = hardwareMap.get(DcMotor.class, "motor_back_right");
        motor_front_left = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor_back_left = hardwareMap.get(DcMotor.class, "motor_back_left");
        servoClasp = hardwareMap.get(Servo.class, "servoClasp");

        telemetry.addData("Status", "Initialized");
        telemetry.update()

        waitForStart();

        double leftPower = 0;
        double rightPower = 0;

        while (opModeIsActive()) {
            leftPower = this.gamepad1.left_stick_y-this.gamepad1.left_stick_x;
            rightPower = this.gamepad1.left_stick_y+this.gamepad1.left_stick_x;

            motor_front_right.setPower(rightPower);
            motor_back_right.setPower(rightPower);
            motor_front_left.setPower(leftPower);
            motor_back_left.setPower(leftPower);

            telemetry.addData("Status", "Running");
            telemetry.addData("Right", rightPower);
            telemetry.addData("Left", leftPower);
            telemetry.update();
        }
    }
}