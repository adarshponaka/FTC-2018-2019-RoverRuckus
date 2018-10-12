package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.lang.Math;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Sup", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

/**
 * Created by adars on 9/12/2018.
 */

public class sup extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor flMotor;
    private DcMotor frMotor;
    private DcMotor blMotor;
    private DcMotor brMotor;


    double flspeedcoefficient = -.9;
    double blspeedcoefficient = -.9;
    double frspeedcoefficient = .9;
    double brspeedcoefficient = .9;

    ColorSensor sensorColor;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

       /* eg: Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app on the phone).
        */
        // leftMotor  = hardwareMap.dcMotor.get("left_drive");
        // rightMotor = hardwareMap.dcMotor.get("right_drive");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        //  rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // telemetry.addData("Status", "Initialized");

        flMotor = hardwareMap.dcMotor.get("fl");
        frMotor = hardwareMap.dcMotor.get("fr");
        blMotor = hardwareMap.dcMotor.get("bl");
        brMotor = hardwareMap.dcMotor.get("br");

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

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
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.9) {
            telemetry.addData("Red",":" + Integer.toString(sensorColor.red()));
            telemetry.addData("Blue",":" + Integer.toString(sensorColor.blue()));
            telemetry.addData("Green",":" + Integer.toString(sensorColor.green()));            telemetry.addData("Status", "Running: " + runtime.toString());
            double A = .5;
            double L = .5;
            flspeedcoefficient = -.6;
            blspeedcoefficient = -.6;
            frspeedcoefficient = .6;
            brspeedcoefficient = .6;
            //front left motor
            L = getL(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.PI / 4);
            A = getA(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.PI / 4);
            flMotor.setPower(flspeedcoefficient * L * A);
            telemetry.addLine("fl: " + Double.toString(flspeedcoefficient * L * A));
            //back left motor
            L = getL(gamepad1.left_stick_x, gamepad1.left_stick_y, -Math.PI / 4);
            A = getA(gamepad1.left_stick_x, gamepad1.left_stick_y, -Math.PI / 4);
            blMotor.setPower(blspeedcoefficient * L * A);
            telemetry.addLine("bl: " + Double.toString(blspeedcoefficient * L * A));
            //front right
            L = getL(gamepad1.right_stick_x, gamepad1.right_stick_y, -Math.PI / 4);
            A = getA(gamepad1.right_stick_x, gamepad1.right_stick_y, -Math.PI / 4);
            frMotor.setPower(frspeedcoefficient * L * A);
            telemetry.addLine("fr: " + Double.toString(frspeedcoefficient * L * A));
            //back right motor,
            L = getL(gamepad1.right_stick_x, gamepad1.right_stick_y, Math.PI / 4);
            A = getA(gamepad1.right_stick_x, gamepad1.right_stick_y, Math.PI / 4);
            brMotor.setPower(brspeedcoefficient * L * A);
            telemetry.addLine("br: " + Double.toString(brspeedcoefficient * L * A));
        }else if (gamepad1.left_trigger == 0){
            telemetry.addData("Red",":" + Integer.toString(sensorColor.red()));
            telemetry.addData("Blue",":" + Integer.toString(sensorColor.blue()));
            telemetry.addData("Green",":" + Integer.toString(sensorColor.green()));
            telemetry.addData("Status", "Running: " + runtime.toString());
            double A = .5;
            double L = .5;
            flspeedcoefficient = -.9;
            blspeedcoefficient = -.9;
            frspeedcoefficient = .9;
            brspeedcoefficient = .9;
            //front left motor
            L = getL(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.PI / 4);
            A = getA(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.PI / 4);
            flMotor.setPower(flspeedcoefficient * L * A);
            telemetry.addLine("fl: " + Double.toString(flspeedcoefficient * L * A));
            //back left motor
            L = getL(gamepad1.left_stick_x, gamepad1.left_stick_y, -Math.PI / 4);
            A = getA(gamepad1.left_stick_x, gamepad1.left_stick_y, -Math.PI / 4);
            blMotor.setPower(blspeedcoefficient * L * A);
            telemetry.addLine("bl: " + Double.toString(blspeedcoefficient * L * A));
            //front right motor
            L = getL(gamepad1.right_stick_x, gamepad1.right_stick_y, -Math.PI / 4);
            A = getA(gamepad1.right_stick_x, gamepad1.right_stick_y, -Math.PI / 4);
            frMotor.setPower(frspeedcoefficient * L * A);
            telemetry.addLine("fr: " + Double.toString(frspeedcoefficient * L * A));
            //back right motor
            L = getL(gamepad1.right_stick_x, gamepad1.right_stick_y, Math.PI / 4);
            A = getA(gamepad1.right_stick_x, gamepad1.right_stick_y, Math.PI / 4);
            brMotor.setPower(brspeedcoefficient * L * A);
            telemetry.addLine("br: " + Double.toString(brspeedcoefficient * L * A));
        }
    }
    private static double getL(double x, double y, double angle){
        return Math.pow((Math.pow(x,2) + Math.pow(y, 2)),.5);
    }
    private static double getA(double x, double y, double angle){
        double x1 =0;
        x1 = getnewX(-x,y,angle);
        double y1=0;
        y1 = getnewY(-x,y,angle);
        double theta = 0;

        theta = (Math.atan(y1/x1));
        if(x1 < 0) {
            theta *= -1;
        }
        if (theta>0||theta<0){

        }else{
            theta = 0;
        }
        return theta/(Math.PI/2);
    }
    private static double getnewX(double x,double y,double angle){
        return x*Math.cos(angle) - y*Math.sin(angle);
    }
    private static double getnewY(double x,double y,double angle){
        return x*Math.sin(angle) + y*Math.cos(angle);
    }
    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}