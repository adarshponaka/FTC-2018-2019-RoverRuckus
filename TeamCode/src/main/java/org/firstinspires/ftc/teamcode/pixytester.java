package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.firstinspires.ftc.teamcode.classes.Pixy;

/**
 * Created by adars on 10/7/2018.
 * Aakash can't handle spice
 */

@Autonomous(name = "pixyTester")

public class pixytester extends LinearOpMode{

    I2cDeviceSynch pixy;

    Pixy cam;

    I2cDeviceSynch pixy2;

    Pixy cam2;

    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    //Mecanum
    Mecanum bot = new Mecanum();

    //Gyro Initialize
    AdafruitIMU imu = new AdafruitIMU();

    private static final Double ticks_per_inch = 510 / (3.1415 * 4);
    private static final Double CORRECTION = .04;
    private static final Double THRESHOLD = 2.0;
    Double driveDistance;

    public void runOpMode(){
        pixy = hardwareMap.i2cDeviceSynch.get("pixy");
        pixy2 = hardwareMap.i2cDeviceSynch.get("pixy2");

        cam = new Pixy(pixy);
        cam2 = new Pixy(pixy2);

        //motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        //Mecanum
        bot = new Mecanum(motorFR,motorFL,motorBR,motorBL);

        //IMU
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        driveDistance = 15.0;

        waitForStart();


        imu.start();

        cam.engage();



        while (opModeIsActive()) {
            if ((cam.getX() != 0) && (cam.getY() != 0) && (cam.numobjects() != 0)) {
                encoderDrive(1, "right", 0.3);
            }
            telemetry.addData("Data 1", cam.getX());
            telemetry.addData("Data 2", cam.getY());
            telemetry.addData("Data 0", cam.numobjects());
            telemetry.addData("Data 3", cam2.getX());
            telemetry.addData("Data 4", cam2.getY());
            telemetry.addData("Data 5", cam2.numobjects());
            telemetry.update();
        }

    }
    public void encoderDrive(double inches, String direction , double power ) {
        int encoderval;
        //
        // Sets the encoders
        //
        bot.reset_encoders();
        encoderval = ticks_per_inch.intValue() * (int) inches;
        bot.run_using_encoders();
        //
        // Uses the encoders and motors to set the specific position
        //
        bot.setPosition(encoderval,encoderval,encoderval,encoderval);
        //
        // Sets the power and direction
        //
        bot.setPowerD(power);
        if (direction == "forward"){
            bot.run_forward();
        } else if(direction == "backward"){
            bot.run_backward();
        } else if (direction == "left"){
            bot.run_left();
        } else if (direction == "right"){
            bot.run_right();
        } else if (direction == "diagonal_left_up"){
            bot.run_diagonal_left_up();
        }

        while (bot.testDistance(motorFL) != 1 && opModeIsActive()) {
            telemetry.addData("Pos ", motorFL.getCurrentPosition());
            telemetry.update();
        }

        bot.brake();
        bot.reset_encoders();

    }

}
