package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous

public class ColourTest extends LinearOpMode{
    // Create objects for this robot
    private Blinker expansion_Hub_1;
    private Blinker expansion_Hub_2;
    //private Gyroscope imu;
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;
    private DcMotorEx arm_motor;
    private ColorSensor colour;
    private DistanceSensor distance;

    @Override
    public void runOpMode() {

        // initialise objects for expansion hub components
        expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotor.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotor.class, "motor_back_left");
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
        arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        // initialise object for the sensors
        colour = hardwareMap.get(ColorSensor.class, "colour");
        distance = hardwareMap.get(DistanceSensor.class, "colour");

        // initialise the direction of the motor
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Red", colour.red());
            telemetry.addData("Blue", colour.blue());
            telemetry.addData("Green", colour.green());
            telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }

}
