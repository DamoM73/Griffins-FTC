package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class TankDrive extends OpMode {
    //Object defintions
    private Blinker Control_Hub;
    private DcMotor motor_front_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_right;
    private DcMotor motor_back_left;


    //functions
    public float power_motor_front_right (float jY, float jx, float rX) {return((jY+jx)/2 + rX/2);}
    public float power_motor_front_left (float jY, float jx, float rX) {return((jY-jx)/2 - rX/2);}
    public float power_motor_back_right(float jY, float jx, float rX) {return((jY-jx)/2 + rX/2);}
    public float power_motor_back_left(float jY, float jx, float rX) {return((jY+jx)/2 - rX/2);}
    
    @Override
    public void init() {
        Control_Hub=hardwareMap.get(Blinker.class, "Control Hub");
        motor_front_right=hardwareMap.get(DcMotor.class, "motor_front_right");
        motor_front_left=hardwareMap.get(DcMotor.class, "motor_front_left");
        motor_back_right=hardwareMap.get(DcMotor.class, "motor_back_right");
        motor_back_left=hardwareMap.get(DcMotor.class, "motor_back_left");
        
        //initialise the motor direction
        //motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        //motor_front_left.setDirection(DcMotor.Direction.REVERESE);
        //motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        //motor_back_left.setDirection(DcMotor.Direction.REVERESE);
  
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Initialised");    
        telemetry.update();
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        motor_front_right.setPower(power_motor_front_right(gamepad1.left_stick_y*-1,gamepad1.left_stick_x,gamepad1.right_stick_x*-1));
        motor_front_left.setPower(power_motor_front_left(gamepad1.left_stick_y*-1,gamepad1.left_stick_x,gamepad1.right_stick_x*-1));
        motor_back_left.setPower(power_motor_back_left(gamepad1.left_stick_y*-1,gamepad1.left_stick_x,gamepad1.right_stick_x*-1));
        motor_back_right.setPower(power_motor_back_right(gamepad1.left_stick_y*-1,gamepad1.left_stick_x,gamepad1.right_stick_x*-1));
        
        // Send telemetry message to signify robot running;
        telemetry.addData("motors",  "driving");
        telemetry.addData("status",  "running");
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
