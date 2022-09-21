package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TankDrive extends OpMode {
    //Object defintions
    private Blinker Control_Hub;
    private Blinker Expansion_Hub;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private DcMotor armlift;
    private DcMotor intake;
    private Servo armextend;
    private Servo claw;
    private DcMotor DDS;


    //functions
    public float power_motor1(float jY, float jX, float rX) {
        return ((jX + jY)/2 + rX /2);
    }

    public float power_motor2(float jY, float jX, float rX) {
        return ((jY - jX)/2 - rX /2);
    }

    public float power_motor3(float jY, float jX, float rX) {
        return ((jY - jX)/2 + rX /2);
    }

    public float power_motor4(float jY, float jX, float rX) {
        return ((jX + jY)/2 - rX /2);
    }
    
     public double extensionPos = 1;
    
    @Override
    public void init() {
        Control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        Expansion_Hub = hardwareMap.get(Blinker.class, "Expansion Hub");
        motor1 = hardwareMap.get(DcMotor.class, "motor_front_left");
        motor2 = hardwareMap.get(DcMotor.class, "motor_front_right");
        motor3 = hardwareMap.get(DcMotor.class, "motor_back_left");
        motor4 = hardwareMap.get(DcMotor.class, "motor_back_right");
        armlift = hardwareMap.get(DcMotor.class, "motor_liftArm");
        intake = hardwareMap.get(DcMotor.class, "motor_spinny");
        armextend = hardwareMap.get(Servo.class, "Servo_ArmExtension");
        claw = hardwareMap.get(Servo.class, "ServoClaw");
        DDS = hardwareMap.get(DcMotor.class, "motor_DDS");
        
        //initialise the motor direction
        motor1.setDirection(DcMotor.Direction.REVERSE);
        motor2.setDirection(DcMotor.Direction.FORWARD);
        motor3.setDirection(DcMotor.Direction.REVERSE);
        motor4.setDirection(DcMotor.Direction.FORWARD);
        
  
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
        motor1.setPower(power_motor1(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,gamepad1.left_stick_x));
        motor2.setPower(power_motor2(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,gamepad1.left_stick_x));
        motor3.setPower(power_motor3(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,gamepad1.left_stick_x));
        motor4.setPower(power_motor4(gamepad1.right_stick_y*-1,gamepad1.right_stick_x,gamepad1.left_stick_x));
        
        // Send telemetry message to signify robot running;
        telemetry.addData("motors",  "driving");
        telemetry.addData("status",  "running");
        
        telemetry.addData("arm lift pos:",  armlift.getCurrentPosition());
        
        // allows left stick to control height and left trigger to maintain position
        armlift.setPower(-0.9 * gamepad2.left_stick_y); // 0.75
        armlift.setPower(0.45 * gamepad2.left_trigger); // 0.3
        /**if (armlift.getCurrentPosition() > 342) {
            armlift.setPower(-0.75 * gamepad2.left_stick_y);
            armlift.setPower(0.3 * gamepad2.left_trigger);
        }
        else {
            armlift.setPower(-0.75 * gamepad2.left_stick_y);
            armlift.setPower(0.4 * gamepad2.left_trigger); 
        }**/
        
        // sets the intake power of the right
        if (gamepad1.a == true):
            intake.setPower(0.5);
        
        /** commented for future reference NOT TO BE USED
        // sets the height of arm to be first level of shipping hub (7.62 cm)
            if(gamepad2.a == true) {
                armlift.setTargetPosition(172);
                armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armlift.setPower(1);
                armlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
        // sets the height of arm to be second level of shipping hub (21.59 cm)
            if(gamepad2.x == true) {
                armlift.setTargetPosition(342);
                armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armlift.setPower(1);
                armlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        
        // sets the height of arm to be third level of shipping hub (37.46 cm)
            if(gamepad2.b == true) {
                armlift.setTargetPosition(494);
                armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armlift.setPower(1);
                armlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            
        // sets the height of arm to be high enough for capping
            if(gamepad2.y == true) {
                armlift.setTargetPosition(626);
                armlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armlift.setPower(1);
                armlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        **/
        
        // extends the arm up
        if (gamepad2.y) {
            if (extensionPos != 1) {
                extensionPos = extensionPos + 0.001;
            }
        }
        
        // extends the arm down
        if (gamepad2.a) {
            if (extensionPos != 0) {
                extensionPos = extensionPos - 0.001;
            }
        }
        
        // run the DDS
        if (gamepad2.right_trigger > 0) {
            DDS.setPower(0.4);
        }
        else {
            DDS.setPower(0);
        }
        
        
        
        telemetry.addData("extension position variable", extensionPos);
        
        // closes claw
        if (gamepad2.x) {
            claw.setPosition(1);
        }
        
        // opens claw
        if (gamepad2.b) {
            claw.setPosition(0);
        }
        
        // sets the position of the arm extension to variable changed when dpad up and down pressed
        armextend.setPosition(extensionPos);
        
        
        
        telemetry.addData("claw", claw.getPosition());
        telemetry.addData("arm", armextend.getPosition());
        telemetry.update();

        
        
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
