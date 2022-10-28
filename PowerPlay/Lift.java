package org.firstinspires.ftc.compcode.PowerPlay;

import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;


public class Lift {
    public Blinker expansion_Hub_2;

    private DcMotor left_lift_motor;
    private DcMotor right_lift_motor;
    
    public int currentPosition;
    public int[] positions = {0,-20,-150,-300,-480};
    ArrayList<String> StrPos = new ArrayList<String>();
    
    int TargPos = 0;
    float speed = 1;

    Lift (DcMotor left_lift_motor,DcMotor right_lift_motor) {
        this.left_lift_motor = left_lift_motor;
        this.right_lift_motor = right_lift_motor;
        right_lift_motor.setDirection(DcMotor.Direction.REVERSE);
        
        StrPos.add("bottom");
        StrPos.add("round");
        StrPos.add("low");
        StrPos.add("medium");
        StrPos.add("high");
        
        right_lift_motor.setTargetPosition(TargPos);
        left_lift_motor.setTargetPosition(TargPos);
        
        right_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_lift_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            
        right_lift_motor.setPower(speed);
        left_lift_motor.setPower(speed);
        
    }
    
    public int getLeftPosition(){
        return left_lift_motor.getCurrentPosition();
    }
    
    public int getRightPosition(){
        return right_lift_motor.getCurrentPosition();
    }
    public int getTargetPosition(){
        return TargPos;
    }

    public float getSpeed(){
        return speed;
    }
    
    
    public void SetMoveSpeed(double distance, float changeSpeed){
        // Positive for up, negative for down
        if (-0.1 < distance && distance < 0.1) {
        }
        else {
            TargPos += distance;
        }
        speed += changeSpeed/-100;
        
        right_lift_motor.setTargetPosition(TargPos);
        left_lift_motor.setTargetPosition(TargPos);
        right_lift_motor.setPower(speed);
        left_lift_motor.setPower(speed);
    }
    
    public void MoveToPosition(int position) {
        TargPos = position;
    }
    
    public int MoveToPositionString(String position) {
        TargPos = positions[StrPos.indexOf(position)];
        return TargPos;
    }

    public void MoveInPositionList(int direction) {
        int currentPosition = (int)(right_lift_motor.getCurrentPosition() + left_lift_motor.getCurrentPosition())/2;
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
}




