package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;


public class Lift {
    public Blinker expansion_Hub_2;

    private DcMotor left_lift_motor;
    private DcMotor right_lift_motor;
    
    public int currentPosition;
    public int[] positions = {0,2,7,11,14,};
    ArrayList<String> StrPos = new ArrayList<String>();

    Lift (DcMotor left_lift_motor,DcMotor right_lift_motor) {
        this.left_lift_motor = left_lift_motor;
        this.right_lift_motor = right_lift_motor;
        right_lift_motor.setDirection(DcMotor.Direction.REVERSE);
        
        StrPos.add("bottom");
        StrPos.add("round");
        StrPos.add("low");
        StrPos.add("medium");
        StrPos.add("high");
        
    }
    
    public void SetMoveSpeed(double speed){
        // Positive for up, negative for down
        if (-0.1 < speed && speed < 0.1) {
            right_lift_motor.setTargetPosition(right_lift_motor.getCurrentPosition());
            left_lift_motor.setTargetPosition(right_lift_motor.getCurrentPosition());
            
            right_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
            right_lift_motor.setPower(0.1);
            left_lift_motor.setPower(0.1);
        }
        else {
            right_lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
            right_lift_motor.setPower(speed*0.5);
            left_lift_motor.setPower(speed*0.5);
        }
    }
    
    public void MoveToPosition(int position) {
        right_lift_motor.setTargetPosition(position);
        left_lift_motor.setTargetPosition(position);
            
        right_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        right_lift_motor.setPower(0.7);
        left_lift_motor.setPower(0.7);
    }
    
    public void MoveToPositionString(String position) {
        int TargPosition = StrPos.indexOf(position);

        right_lift_motor.setTargetPosition(TargPosition);
        left_lift_motor.setTargetPosition(TargPosition);
            
        right_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        right_lift_motor.setPower(0.7);
        left_lift_motor.setPower(0.7);
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
        currentPosition = (int)(right_lift_motor.getCurrentPosition() + left_lift_motor.getCurrentPosition())/2;
        return currentPosition;
    }
}




