import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Blue_Depot", group="Linear Opmode")

public class Blue_Depot extends LinearOpMode {
    // Create objects for this robot
    private Blinker expansion_Hub_2;
    //private Gyroscope imu;
    private DcMotor motor_front_right;
    private DcMotor motor_back_right;
    private DcMotor motor_front_left;
    private DcMotor motor_back_left;
    private DcMotorEx arm_motor;
    private DcMotorEx input_output_motor;
    private DcMotor DDS_motor;
    private ColorSensor colour;
    private DistanceSensor distance;



    // convert count per revolution to counts per cm
    static final double HD_COUNTS_PER_REV = 28;
    static final double DRIVE_GEAR_REDUCTION = 19.2;
    static final double WHEEL_CIRCUMFERENCE_MM = 10 * Math.PI;
    static final double DRIVE_COUNTS_PER_MM = (HD_COUNTS_PER_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE_MM;
    static final double DRIVE_COUNTS_PER_CM = DRIVE_COUNTS_PER_MM * 1;
    static final double DRIVE_COUNTS_PER_DEGREE = 20;
    static final double CM_PER_SECOND_PER_POWER = 20;
    static final double DEGREES_PER_SECOND_PER_POWER = 20;

    private ElapsedTime
    runtime = new ElapsedTime();
    public void motorFwdTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when travelling forward (all +)
        int motor1Target = (int)motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);
        int motor2Target = (int)motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);
        int motor3Target = (int)motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);
        int motor4Target = (int)motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM);

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_front_left.setPower(speed);
        motor_back_right.setPower(speed);
        motor_back_left.setPower(speed);
        runtime.reset();
        
        //Caluculates time required
        time = DEGREES_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorBwdTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when travelling backward
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        motor_front_right.setPower(-speed);
        motor_front_left.setPower(-speed);
        motor_back_right.setPower(-speed);
        motor_back_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = DEGREES_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorRgtTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when driving right (fr -, bl -)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        motor_front_right.setPower(-speed);
        motor_front_left.setPower(speed);
        motor_back_right.setPower(speed);
        motor_back_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = DEGREES_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorLftTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when driving left (fl -, br -)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_front_left.setPower(-speed);
        motor_back_right.setPower(-speed);
        motor_back_left.setPower(speed);
        runtime.reset();
        
        //Caluculates time required
        time = DEGREES_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorFwdRgtTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when driving diagonally right and forward (fr 0, bl 0)
        int motor1Target = (int)motor_front_right.getCurrentPosition();
        int motor2Target = (int)(motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor3Target = (int)(motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor4Target = (int)motor_back_left.getCurrentPosition();


        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        
        motor_back_right.setPower(speed);
        motor_front_left.setPower(speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        
    }

    public void motorFwdLftTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when driving diagonally left forward (fl 0, br 0)
        int motor1Target = (int)(motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));
        int motor2Target = (int)motor_front_left.getCurrentPosition();
        int motor3Target = (int)motor_back_right.getCurrentPosition();
        int motor4Target = (int)(motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        
        motor_front_right.setPower(speed);
        motor_back_left.setPower(speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorBwdRgtTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when driving diagonally right backward (fr -1, bl -1, fl 0, br 0)
        int motor1Target = (int)(-1 * (motor_front_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor2Target = (int)motor_front_left.getCurrentPosition();
        int motor3Target = (int)motor_back_right.getCurrentPosition();
        int motor4Target = (int)(-1 * (motor_back_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        
        motor_front_right.setPower(-speed);
        motor_back_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_left.setPower(0);
    }

    public void motorBwdLftTargetPositions (float cmDistance, double speed) {
        /**
        // set target positions when driving diagonally left backward (fr 0, bl 0, fl -1, br -1)
        int motor1Target = (int)motor_front_right.getCurrentPosition();
        int motor2Target = (int)(-1 * (motor_front_left.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor3Target = (int)(-1 * (motor_back_right.getCurrentPosition() + (int)(cmDistance * DRIVE_COUNTS_PER_CM)));
        int motor4Target = (int)motor_back_left.getCurrentPosition();

        // set motors to drive to position.
        motor_front_right.setTargetPosition(motor1Target);
        motor_front_left.setTargetPosition(motor2Target);
        motor_back_right.setTargetPosition(motor3Target);
        motor_back_left.setTargetPosition(motor4Target);
        **/
        
        // sets power of motors
        
        motor_back_right.setPower(-speed);
        motor_front_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = CM_PER_SECOND_PER_POWER * cmDistance * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        
    }

    public void turnMotors(int angle, double speed) {
        //motor_front_right.setTargetPosition((int)motor_front_right.getCurrentPosition()+(int)(angle*DRIVE_COUNTS_PER_DEGREE));
        //motor_front_left.setTargetPosition((int)motor_front_left.getCurrentPosition()+(int)(-1*angle*DRIVE_COUNTS_PER_DEGREE));
        //motor_back_right.setTargetPosition((int)motor_back_right.getCurrentPosition()+(int)(angle*DRIVE_COUNTS_PER_DEGREE));
        //motor_back_left.setTargetPosition((int)motor_back_left.getCurrentPosition()+(int)(-1*angle*DRIVE_COUNTS_PER_DEGREE));
        
        // sets power of motors
        motor_front_right.setPower(speed);
        motor_back_right.setPower(-speed);
        motor_front_left.setPower(speed);
        motor_back_left.setPower(-speed);
        runtime.reset();
        
        //Caluculates time required
        time = DEGREES_PER_SECOND_PER_POWER * angle * speed;
        while (runtime.seconds() < time) {

        }

        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);
    }

    public void runMotors(double speed, double time) {
        // sets power of motors so that they will run to position
        motor_front_right.setPower(speed);
        motor_back_right.setPower(speed);
        motor_front_left.setPower(speed);
        motor_back_left.setPower(speed);
        runtime.reset();

        /** wait while motors are running
         while (opModeIsActive() && (motor_front_right.isBusy() || motor_front_left.isBusy() || motor_back_right.isBusy() || motor_back_left.isBusy())) {
         }
         **/

        while (runtime.seconds() < time) {

        }


        // stops motors
        motor_front_right.setPower(0);
        motor_back_right.setPower(0);
        motor_front_left.setPower(0);
        motor_back_left.setPower(0);



    }


    @Override
    public void runOpMode() {
        /*
        SETUP
        Initialises all the required variables and objects and initialises them
        ready for the start();
        */


        // initialise objects for expansion hub components
        //expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Control Hub");
        //imu = hardwareMap.get(Gyroscope.class, "imu");

        // initialise object for the dc motor
        motor_front_right = hardwareMap.get(DcMotorEx.class, "motor_front_right");
        motor_front_left = hardwareMap.get(DcMotorEx.class, "motor_front_left");
        motor_back_right = hardwareMap.get(DcMotorEx.class, "motor_back_right");
        motor_back_left = hardwareMap.get(DcMotorEx.class, "motor_back_left");
        
         arm_motor = hardwareMap.get(DcMotorEx.class, "arm_motor");
         arm_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         input_output_motor = hardwareMap.get(DcMotorEx.class, "input_output_motor");
         DDS_motor = hardwareMap.get(DcMotor.class, "DDS_motor");
         
        // initialise sensors
        colour = hardwareMap.get(ColorSensor.class, "left_colour");
        distance = hardwareMap.get(DistanceSensor.class, "left_colour");

        // initialise the directions of the motors
        motor_front_right.setDirection(DcMotor.Direction.FORWARD);
        motor_front_left.setDirection(DcMotor.Direction.REVERSE);
        motor_back_right.setDirection(DcMotor.Direction.FORWARD);
        motor_back_left.setDirection(DcMotor.Direction.REVERSE);

        motor_front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        DDS_motor.setDirection(DcMotor.Direction.FORWARD);

        // set up telemetry to disply on driver station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run during autonomous
        if (opModeIsActive()) {
            //Move to barcode
            motorFwdTargetPositions(35,0.05);
            //changes the motor modes to RUN_TO_POSITION
            motor_front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor_back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            int barcode = 0;

            //slide across barcode checking distance
            for (int i = 0; i < 3 ;i++) {
                if (distance.getDistance(DistanceUnit.CM)<20){
                    barcode = i;
                }

                motorRgtTargetPositions(21,0.05);
                
            }

             //compare distance and set right height for output
             if(barcode == 0) {
             arm_motor.setTargetPosition(0);
             arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             arm_motor.setVelocity(10);
             }else {if(barcode == 1) {
             arm_motor.setTargetPosition(-30);
             arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             arm_motor.setVelocity(10);
             }else {
             arm_motor.setTargetPosition(-60);
             arm_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             arm_motor.setVelocity(10);
             }
             }
             //activate output
             input_output_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
             input_output_motor.setPower(60);

            //Turn towards warehouse
            turnMotors(-90,0.03);

            //Move  to Warehouse
            motorFwdTargetPositions(300,0.03);
        }
    }
}

