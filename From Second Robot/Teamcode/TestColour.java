
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;


@TeleOp

public class TestColour extends OpMode {
    /* Declare OpMode members. */
    private ColorSensor colour;
    private DistanceSensor distance;
    private Blinker expansion_Hub_2;
    
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        colour = hardwareMap.get(ColorSensor.class, "left_colour");
        distance = hardwareMap.get(DistanceSensor.class, "left_colour");
        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        motor_front_right.setPower(0.0);
        motor_front_left.setPower(0.0);
        motor_back_right.setPower(0.0);
        motor_back_left.setPower(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (distance.getDistance(DistanceUnit.CM)<20) {
            telemetry.addData("InRange: ","True");
            motor_front_right.setPower(0.1);
            motor_front_left.setPower(0.1);
            motor_back_right.setPower(0.1);
            motor_back_left.setPower(0.1);
        }else{
            telemetry.addData("InRange: ","False");
            motor_front_right.setPower(0.0);
            motor_front_left.setPower(0.0);
            motor_back_right.setPower(0.0);
            motor_back_left.setPower(0.0);
        }
        telemetry.addData("Distance: ",distance.getDistance(DistanceUnit.CM) );
        telemetry.update();
        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");
        

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}
