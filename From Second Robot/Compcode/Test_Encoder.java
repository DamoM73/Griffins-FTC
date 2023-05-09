package org.firstinspires.ftc.compcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp

public class Test_Encoder extends LinearOpMode{
    private DcMotor motor_front_right;
    private Blinker expansion_Hub_2;
    
    @Override
    public void runOpMode() {
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_front_right.setTargetPosition(200);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        
        
        motor_front_right.setPower(0.1);
        
        while (opModeIsActive()){
            
            telemetry.addData("Encoder",motor_front_right.getCurrentPosition());
            telemetry.update();
            
        
        }
    }
}