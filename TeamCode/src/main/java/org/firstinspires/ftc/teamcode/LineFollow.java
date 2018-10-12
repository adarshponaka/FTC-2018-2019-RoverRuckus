package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.classes.Mecanum;
import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;

/**
 * Created by adars on 9/15/2018.
 */
@Autonomous(name="LineFollow", group="SupBot")

public class LineFollow extends LinearOpMode {
    private DcMotor motorFR;
    private DcMotor motorFL;
    private DcMotor motorBR;
    private DcMotor motorBL;

    //Mecanum
    Mecanum bot = new Mecanum();

    //Gyro Initialize
    AdafruitIMU imu = new AdafruitIMU();

    //Color sensor
    ColorSensor sensorColor;

    private static final Double ticks_per_inch = 510 / (3.1415 * 4);
    private static final Double THRESHOLD = 2.0;

    public void runOpMode() {
        //motors
        motorFL = hardwareMap.dcMotor.get("fl");
        motorFR = hardwareMap.dcMotor.get("fr");
        motorBL = hardwareMap.dcMotor.get("bl");
        motorBR = hardwareMap.dcMotor.get("br");

        //Mecanum
        bot = new Mecanum(motorFR, motorFL, motorBR, motorBL);

        //IMU
        imu = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu"));

        //Color Sensor
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        waitForStart();
        imu.start();

        //STATE ONE: MOVE FORWARD
        int stop = 0;
        /*
        motorFR.setPower(4);
        motorBR.setPower(4);
        motorFL.setPower(4);
        motorBL.setPower(4);
        */
        gyroTurnRight(90, "oof", 0.3);
        telemetry.addData("IMU:",imu.getHeading());

        while(stop == 0 ) {
            while ((sensorColor.blue() < 80) && (sensorColor.red() < 80) && (sensorColor.green() < 80)) {
                telemetry.addData("sup", "sup");
                encoderDrive(1, "forward", 0.2);
            };
            while ((sensorColor.blue() > 350) && (sensorColor.red() > 350) && (sensorColor.green() > 350)) {
                gyroTurnRight(90, "oof", 0.3);
                //bot.turn_right();
                telemetry.addData("sup", "sup");
            };
            while ((sensorColor.blue() < 350) && (sensorColor.red() < 350) && (sensorColor.green() < 350)&&(sensorColor.blue() > 80) && (sensorColor.red() > 80) && (sensorColor.green() > 80)) {
                bot.brake();
                //bot.turn_right();
                stop = 1;
            };
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
    public void gyroTurnRight(double angle, String direction, double power){
        double aheading = imu.getHeading() + angle;
        telemetry.addData("IMU:",imu.getHeading());
        if(aheading>360){
            aheading=aheading-360;
        }
        boolean gua = false;
        bot.run_without_encoders();
        bot.setPowerD(power);

        while(opModeIsActive() && gua==false) {
            //aheading = Math.abs(imu.getHeading()) + angle;
            bot.turn_right();

            telemetry.addData("Heading", imu.getHeading());
            telemetry.addData("Target Angle", aheading);
            telemetry.update();
            if (imu.getHeading() >= (aheading - THRESHOLD) && (imu.getHeading() <= (aheading + THRESHOLD))) {
                bot.brake();
                gua=true;
            }

        }

        bot.brake();

    }
}
