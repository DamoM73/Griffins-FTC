package org.firstinspires.ftc.compcode.CentreStage;
// Imports
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

// Class for control of vertical lift
public class Lift {
    // Create variables
    public Blinker expansion_Hub_2;

    private DcMotor liftRotateMotor;
    private DcMotor liftExtendMotor;
    private Servo wristServo;
    private Servo hookServo;

    private float wristPosition;
    private double armExtendModifier = 0.5;
    private double wristSpeedModifier = 0.05;
    private double armRotateModifier = 0.1;

    private double wristPickupAngle = 0.5;
    private int liftPickupExtend = 0;
    private int armPickupAngle = -200;

    private float wristBaseAngle = 1;
    private int liftBaseExtend = 0;
    private int armBaseAngle = -50;

    private float wristCompactAngle = 0;
    private int liftCompactExtend = 0;
    private int armCompactAngle = 0;

    Lift (DcMotor liftRotateMotor, DcMotor liftExtendMotor, Servo wristServo, Servo hook) {
        // Create lift object with all powers
        this.liftRotateMotor = liftRotateMotor;
        this.liftExtendMotor = liftExtendMotor;
        this.wristServo = wristServo;
        this.hookServo = hook;

        liftRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftExtendMotor.setDirection(DcMotor.Direction.REVERSE);
        liftExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void extendArm(double speed){
        // Positive for up, negative for down
        if (speed > -0.1 && speed < 0.1) {
            liftExtendMotor.setPower(0);
            if (liftExtendMotor.getCurrentPosition() < 5) {
                // Near bottom
                hookServo.setPosition(1); // Turn on
            }
        }
        // if joystick is moved
        else {
            // if max
            if (liftExtendMotor.getCurrentPosition()>1750) {
                liftExtendMotor.setTargetPosition(1750);
                liftExtendMotor.setPower(0.2);
                liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            // if min
            else if (liftExtendMotor.getCurrentPosition()<0) {
                liftExtendMotor.setTargetPosition(0);
                liftExtendMotor.setPower(0.2);
                liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else {
                hookServo.setPosition(0); // Turn off
                liftExtendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
                liftExtendMotor.setPower(speed*armExtendModifier);
            }
        }
    }

    public void rotateArm(double speed) {
        if (-0.1 < speed && speed < 0.1) {
            liftRotateMotor.setPower(0);
        }
        else {
            liftRotateMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            liftRotateMotor.setPower(speed*armRotateModifier);
        }
    }

    public void rotateWrist(double speed) {
       
        ///????? IDK, it is a servo which means you cnat use set power and stuff so i am seeing if this works, it gets the position and then modifys it so it can rotate or maybe it is a contin
        //wristServo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //wristServo.setPower(speed);

        wristServo.setPosition(wristServo.getPosition() + speed*wristSpeedModifier);
    }

    public void pickUpPosition() {
        /**Move to pick up Position */
        wristServo.setPosition(wristPickupAngle);
        liftExtendMotor.setTargetPosition(liftPickupExtend);
        liftRotateMotor.setTargetPosition(armPickupAngle);
        liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setPower(1);
        liftRotateMotor.setPower(1);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex){
            ex.printStackTrace();
        }
        hookServo.setPosition(1); // Turn on
    }

    public void moveToBasePosition(){
        /**Move to position to place on bottom position */
        hookServo.setPosition(1); // Turn off
        wristServo.setPosition(wristBaseAngle);
        liftExtendMotor.setTargetPosition(liftBaseExtend);
        liftRotateMotor.setTargetPosition(armBaseAngle);
        liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setPower(1);
        liftRotateMotor.setPower(1);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex){
            ex.printStackTrace();
        }
    }

    public void compact() {
        /**Move to compacted position */
        wristServo.setPosition(wristCompactAngle);
        liftExtendMotor.setTargetPosition(liftCompactExtend);
        liftRotateMotor.setTargetPosition(armCompactAngle);
        liftRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftExtendMotor.setPower(1);
        liftRotateMotor.setPower(1);
        try {
            Thread.sleep(1000);
        }
        catch(InterruptedException ex){
            ex.printStackTrace();
        }
        hookServo.setPosition(1); // Turn on
    }
}