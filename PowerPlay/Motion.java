package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



public class Motion {
    // Initialise control hub
    public Blinker expansion_Hub_2;
    public BNO055IMU imu;
    public Orientation lastAngles = new Orientation();
    
    // Initialise motor objects.
    public DcMotor motor_front_right;
    public DcMotor motor_back_right;
    public DcMotor motor_front_left;
    public DcMotor motor_back_left;
    
    // convert count per revolution to counts per cm 
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = ((1+(46/11))*(1+(46/11)));
    static final double WHEEL_CIRCUMFERENCE_MM = 100 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM_STRAIGHT = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM/1.2682;
    static final double DRIVE_COUNTS_PER_CM_STRAIGHT = DRIVE_COUNTS_PER_MM_STRAIGHT * 10;
    static final double DRIVE_COUNTS_PER_MM_SIDEWAYS = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM_SIDEWAYS = DRIVE_COUNTS_PER_MM_SIDEWAYS * 10;

    // Initiliase variables
    private Boolean turning = false;
    private double gap;
    private double target;
    private double power;
    private double leftPower;
    private double rightPower;
    private int degrees;


    private float front_left_power(float jY, float jX, float rX) {
        /**
        Use joystick positions to translate to front left motor power for mechanum drive.
        **/
        return ((jX + jY)/2 - rX /2);
    }

    private float front_right_power(float jY, float jX, float rX) {
        /**
        Use joystick positions to translate to front right motor power for mechanum drive.
        **/
        return ((jY - jX)/2 + rX /2);
    }

    private float back_left_power(float jY, float jX, float rX) {
        /**
        Use joystick positions to translate to back left motor power for mechanum drive.
        **/
        return ((jY - jX)/2 - rX /2);
    }

    private float back_right_powe(float jY, float jX, float rX) {
        /**
        Use joystick positions to translate to back right motor power for mechanum drive.
        **/
        return ((jX + jY)/2 + rX /2);
    }
    
    Motion (DcMotor motor_front_rightN,DcMotor motor_back_leftN, DcMotor motor_front_leftN, DcMotor motor_back_rightN,BNO055IMU imuN) {
        /**
        Constructor method for motors.
        **/
        this.motor_front_right = motor_front_rightN;
        this.motor_back_left = motor_back_leftN;
        this.motor_front_left = motor_front_leftN;
        this.motor_back_right = motor_back_rightN;
        this.imu = imuN;

    }
    
    public void JoystickMoving (float left_x,float right_x,float right_y) {
        /**
        Uses joystick controls to move the robot.
        **/
        if (turning) {
            gap = absolute(getAngle() - target);
            if (gap > 360){gap-=360;}
            
            if (gap <=15){
                double scaleFactor = gap/15;
                if (scaleFactor*power<0.35){
                    scaleFactor=0.35/power;
                }
                
                if (degrees < 0) {   // turn right.
                    leftPower = power*scaleFactor;
                    rightPower = -power*scaleFactor;
                }
                else if (degrees > 0) {   // turn left.
                    leftPower = -power*scaleFactor;
                    rightPower = power*scaleFactor;
                }
                    
                motor_front_right.setPower(rightPower);
                motor_back_right.setPower(rightPower);
                motor_front_left.setPower(leftPower);
                motor_back_left.setPower(leftPower);
            }
            if (gap < 1) {
                // turn the motors off.
                motor_front_right.setPower(0);
                motor_back_right.setPower(0);
                motor_front_left.setPower(0);
                motor_back_left.setPower(0);
                turning = false;
            }
        } else {
        motor_front_right.setPower(front_right_power(right_y*-1, right_x*-1, -left_x));
        motor_back_right.setPower(back_right_power(right_y*-1, right_x*-1, -left_x));
        motor_front_left.setPower(front_left_power(right_y*-1, right_x*-1, -left_x));;
        motor_back_left.setPower(back_left_power(right_y*-1, right_x*-1, -left_x));
        }
    }
    
    public void motorFwdTargetPositions (float cmDistance, double speed) {
        /**
        Calculates motor encoder values and applies them for moving forward
        Inputs: distance, speed
        **/
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        // set target positions when travelling forward (all +)
        int motor1Target = (int)motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);
        int motor2Target = (int)motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);
        int motor3Target = (int)motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);
        int motor4Target = (int)motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT);

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_front_left.setPower(speed);
        motor_back_right.setPower(speed);
        motor_back_left.setPower(speed);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    public void motorBwdTargetPositions (float cmDistance, double speed) {
        /**
        Calculates motor encoder values and applies them for moving backward
        Inputs: distance, speed
        **/
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        // set target positions when travelling backward
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_STRAIGHT)));

        // set motors to drive to position
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        // sets power of motors
        motor_front_right.setPower(-speed);
        motor_front_left.setPower(-speed);
        motor_back_right.setPower(-speed);
        motor_back_left.setPower(-speed);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    public void motorRgtTargetPositions (float cmDistance, double speed) {
        /**
        Calculates motor encoder values and applies them for moving right
        Inputs: distance, speed
        **/
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        
        // set target positions when driving right (fr -, bl -)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        // sets power of motors
        motor_front_right.setPower(-speed);
        motor_front_left.setPower(speed);
        motor_back_right.setPower(speed);
        motor_back_left.setPower(-speed);
                
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    public void motorLftTargetPositions (float cmDistance, double speed) {
        /**
        Calculates motor encoder values and applies them for moving left
        Inputs: distance, speed
        **/
        motor_front_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_front_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_right.setMode(DcMotor.RunMode.RESET_ENCODERS);
        motor_back_left.setMode(DcMotor.RunMode.RESET_ENCODERS);
                
        // set target positions when driving left (fl -, br -)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS)));
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM_SIDEWAYS));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_front_left.setPower(-speed);
        motor_back_right.setPower(-speed);
        motor_back_left.setPower(speed);

        // Starts motors
        motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        //Waits until motors finished
        while (motor_front_left.isBusy() || motor_front_right.isBusy() || motor_back_left.isBusy() || motor_back_right.isBusy()){
            try {
                Thread.sleep(7000);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }
    
    private double getAngle() {
        /**
        Gets the robots angle from gyroscope
        Returns angles
        **/
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }
    
    private double absolute(double number){
        /**
        Returns the absolute value of a number
        Returns number
        **/
        if (number>0){return number;}
        else {return number*-1;}
    }
    
    public void rotate(int degrees, double power){
        /**
        Rotates the robot around.
        Clockwise degrees are negative
        Inputs: degrees to turn, power
        **/
        double  leftPower, rightPower;
        
        // restart imu movement tracking.
        target = degrees + getAngle();
        double closeToNinety = target/90;
        target = Math.round(closeToNinety)*90;
        
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motor_front_right.setPower(rightPower);
        motor_back_right.setPower(rightPower);
        motor_front_left.setPower(leftPower);
        motor_back_left.setPower(leftPower);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // rotate until turn is completed.
        double gap = absolute(getAngle() - target);

        while (gap>1) {
            gap = absolute(getAngle() - target);
            if (gap > 360){gap-=360;}
            
            if (gap <=15){
                double scaleFactor = gap/15;
                if (scaleFactor*power<0.35){
                    scaleFactor=0.35/power;
                }
                
                if (degrees < 0) {   // turn right.
                    leftPower = power*scaleFactor;
                    rightPower = -power*scaleFactor;
                }
                else if (degrees > 0) {   // turn left.
                    leftPower = -power*scaleFactor;
                    rightPower = power*scaleFactor;
                }
                
                motor_front_right.setPower(rightPower);
                motor_back_right.setPower(rightPower);
                motor_front_left.setPower(leftPower);
                motor_back_left.setPower(leftPower);
            }
        }

        // turn the motors off.
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }

    public void rotateAuto(int degrees, double power){
        // restart imu movement tracking.
        double target = degrees + getAngle();
        double closeToNinety = target/90;
        target = Math.round(closeToNinety)*90;
        
        // getAngle() returns + when rotating anticlockwise (left) and - when rotating clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        motor_front_right.setPower(rightPower);
        motor_back_right.setPower(rightPower);
        motor_front_left.setPower(leftPower);
        motor_back_left.setPower(leftPower);
        
        motor_front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // rotate until turn is completed.
        gap = absolute(getAngle() - target);

        turning = true;
    }
}
