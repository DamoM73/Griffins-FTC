package org.firstinspires.ftc.compcode.PowerPlay;
// Imports
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;

// Class for control of vertical lift
public class Lift {
    // Create variables
    public Blinker expansion_Hub_2;

    private DcMotor liftRotateMotor;
    private DcMotor liftExtendMotor;
    private Servo wristServo;

    Lift (DcMotor liftRotateMotor, DcMotor liftExtendMotor, Servo wristServo) {
        // Create lift object with all powers
        this.liftRotateMotor = liftRotateMotor;
        this.liftMotor = liftMotor;
        this.liftExtendMotor = liftExtendMotor;

        liftRotateMotor.setDirection(DcMotor.Direction.REVERSE);
        liftRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftExtendMotor.setDirection(DcMotor.Direction.REVERSE);
        liftExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public void extendArm(double speed){
        // Positive for up, negative for down
        if (-0.1 < speed && speed < 0.1) {
            liftExtendMotor.setPower(0);
        }
        else {
            liftExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftExtendMotor.setPower(speed*0.5);
        }
    }

    public void rotateArm(double speed) {
        if (-0.1 < speed && speed < 0.1) {
            liftRotateMotor.setPower(0);
        }
        else {
            liftRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftRotateMotor.setPower(speed*0.5);
        }
    
    }
    
    public void MoveToPosition(int position) {
        // Starts the motor moving 
        right_lift_motor.setTargetPosition(position);
        left_lift_motor.setTargetPosition(position);
            
        right_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        right_lift_motor.setPower(0.7);
        left_lift_motor.setPower(0.7);
        
        // Waits for motors to reach objective
        while (right_lift_motor.isBusy() || left_lift_motor.isBusy()){
            try {
                Thread.sleep(100);   
            }
            catch(InterruptedException ex){
                ex.printStackTrace();
            }
        }
        // Stop motors
        right_lift_motor.setPower(0.2);
        left_lift_motor.setPower(0.2);
        
    }
    
    public void MoveToPositionString(String position) {
        int TargPosition = StrPos.indexOf(position);

        MoveToPosition(TargPosition);
    }

    public void MoveInPositionList(int direction) {
        UpdateLiftPosition();
        int position_of_lower=0;
        int distance_from_lower=0;
        int position_of_upper=0;
        int distance_from_upper=0;
        int position_of_proper=0;
        for (int i = 0; i < positions.length; i++) {
            if (positions[i] < currentPosition) {
                position_of_lower = i;
                distance_from_lower = currentPosition - positions[i];
            } else if ((positions[i] > currentPosition) && (position_of_upper == 0)) {
                position_of_upper = i;
                distance_from_upper = positions[i] - currentPosition;
            } else if (positions[i] == currentPosition) {
                position_of_proper = i;
            }
        }
        // Check if on position 
        if (position_of_proper != 0){
            position_of_proper = position_of_proper;
        // Get which lift position lift is closest to
        } else if (distance_from_upper > distance_from_lower){
            // Closer to lower
            position_of_proper = position_of_lower;
        } else {
            // Closer to upper
            position_of_proper = position_of_upper;
        }
        int changedPos = position_of_proper + direction;
        if (changedPos > positions.length) {
            changedPos = positions.length;
        } else if (changedPos < 0){
            changedPos = 0;
        }
        MoveToPosition(changedPos);
    }
    
    
    public int UpdateLiftPosition() {
        // Returns the lift position
        currentPosition = (int)(right_lift_motor.getCurrentPosition() + left_lift_motor.getCurrentPosition())/2;
        return currentPosition;
    }
}