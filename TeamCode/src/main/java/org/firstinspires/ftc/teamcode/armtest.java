package org.firstinspires.ftc.teamcode;

/**
 * Created by adars on 11/20/2018.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MecanumDrive", group = "Drive")
public class armtest extends OpMode {
    // instance variables
    // private variables
    // Motors

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    //AdafruitIMU imu;

    double Ch1;
    double Ch3;
    double Ch4 ;
    double accel;
    double speedv = 2;
    int endtime = 0;
    boolean pressed;

    double speedcoef;

    // Servos

    // constructors
    public armtest() {
        // default constructor

    }

    @Override
    public void init() {
        //
        // Initialize everything
        //
        // Motors

        motorFL = hardwareMap.dcMotor.get("m1");
        motorFR = hardwareMap.dcMotor.get("m2");

        // imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));
        // imu.init();

        pressed = false;
        endtime = 0;
        speedcoef = .5;

        accel = 0;

    }

    @Override
    public void start() {
        //imu.start();`
    }

    // loop
    @Override
    public void loop() {
        //accel = Math.sqrt(imu.getAccelX()*imu.getAccelX() + imu.getAccelZ()*imu.getAccelZ() + imu.getAccelY()*imu.getAccelY());

        if (gamepad1.a){
            speedv = 2;                   //fast
        }
        if (gamepad1.b){
            speedv = 1;                   //slow
        }
        Ch1 = gamepad1.right_stick_x;
        Ch3 = gamepad1.left_stick_y;
        Ch4 = gamepad1.left_stick_x;

        motorFR.setPower( speedcoef* -(Ch3 - Ch1 - Ch4));
        motorFL.setPower( speedcoef * (Ch3 + Ch1 + Ch4));


        if (speedv == 1){
            speedcoef = .25;
        }
        if (speedv == 2){
            speedcoef = .5;
        }
       /*if(gamepad1.a){
            speedcoef = .5;
        }else{
           speedcoef = 1;
       }*/

        telemetry.addData("Speed coeff" , speedcoef);
        //telemetry.addData("Acceleration" , accel);
        telemetry.update();

        // Runs the collector
        //



    }

    // functions
    @Override
    public void stop() {

        // set to zero so the power doesn't influence any motion or rotation in the robot

    }

}
