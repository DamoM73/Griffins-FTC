package org.firstinspires.ftc.compcode.CentreStage;

// Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


// Class for Driver Controls
@TeleOp(name = "Test Motors")
public class eachMotor extends OpMode {
    public BNO055IMU imu;
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;

    public Servo leftIntakeServo;
    public Servo rightIntakeServo;

    private ColorSensor colour;
    private DistanceSensor distance;
    
    // Initialise interfaces with other modules
    Motion drivetrain;

    @Override
    public void init() {
        // Create expansion hub
        
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");

        drivetrain = new Motion(motor_front_right,motor_back_left,motor_front_left,motor_back_right,imu);
        

        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Fun message to drivers!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        telemetry.addData("Have Fun","Drivers");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.update();
        if (gamepad1.y) {
            motor_front_left.setPower(0.1);
        }
        else {
            motor_front_left.setPower(0);
        }
        if (gamepad1.b) {
            motor_front_right.setPower(0.1);
        }
        else {
            motor_front_right.setPower(0);
        }
        if (gamepad1.x) {
            motor_back_left.setPower(0.1);
        }
        else {
            motor_back_left.setPower(0);
        }
        if (gamepad1.a) {
            motor_back_right.setPower(0.1);
        }
        else {
            motor_back_right.setPower(0);
        }
        
        telemetry.addData("Front Left", "Y");
        telemetry.addData("Front Right", "B");
        telemetry.addData("Back Left", "X");
        telemetry.addData("Back Right", "A");

        telemetry.update();
    }
}
