package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.CRServo;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;
        import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.AdafruitIMU;


/**
 * Created by adars on 11/4/2018.
 */

@TeleOp(name="armtester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice


public class armtester extends OpMode {
    private DcMotor motorT; //motor top
    private DcMotor motorB; //motor base
    private Servo EEservo;
    private ElapsedTime runtime = new ElapsedTime();
    private double AB = .5;
    private double LB = .5;
    private double AT = .5;
    private double LT = .5;
    private double powerB = 0;
    private double powerT = 0;
    private double EEservopos = 0.0;
    private AdafruitIMU imu1 = new AdafruitIMU();
    private AdafruitIMU imu2 = new AdafruitIMU();

    //inverse kinematics
    private double x = 20; //in inches
    private double y = -5; //in inches
    private double phi = 180 ; //in degrees
    private double l1 = 14.5; //in inches
    private double l2 = 14.5; //in inches
    private double l3 = 7.398; //in inches
    private double theta1 = 0; //in degrees
    private double theta2 = 0; //in degrees
    private double theta3 = 0; //in degrees

    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        motorB = hardwareMap.dcMotor.get("mb"); //motor base
        motorT = hardwareMap.dcMotor.get("mt"); //motor top
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EEservo = hardwareMap.servo.get("ee"); //end effector
        EEservo.setPosition(EEservopos);
        phi = (phi /180) * Math.PI; //phi to radians
        imu1 = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu1"));
        imu1.init();
        //imu2 = new AdafruitIMU(hardwareMap.get(BNO055IMU.class, "imu2"));

    }
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        imu1.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){

        telemetry.addLine("IMU 1 " + Double.toString(Math.sqrt(imu1.getAccelX()*imu1.getAccelX() + imu1.getAccelZ()*imu1.getAccelZ() + imu1.getAccelY()*imu1.getAccelY())));
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addLine("Motors B encoder " + Integer.toString(motorB.getCurrentPosition()));
        telemetry.addLine("Motors T encoder " + Integer.toString(motorT.getCurrentPosition()));

        //manual arm movement
            LB = getL(0, gamepad1.left_stick_y, Math.PI / 4);
            AB = getA(0, gamepad1.left_stick_y, Math.PI / 4);
            powerB = 0.9*LB*AB;
            motorB.setPower((0.5*powerB));
            motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addLine("mb: " + Double.toString(powerB));

            LT = getL(0, gamepad1.right_stick_y, Math.PI / 4);
            AT = getA(0, gamepad1.right_stick_y, Math.PI / 4);
            powerT = 0.9*LT*AT;
            motorT.setPower((powerT));
            motorT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addLine("mt: " + Double.toString(powerT));

        //manual servo movement
            //x is turning down when end effector is left
            //b is turning down when end effector is right
            if (gamepad1.x){
              EEservopos = EEservopos - 0.001;
            }
            if (gamepad1.b){
                EEservopos = EEservopos + 0.001;
            }
            if(EEservopos >= 1.0){
                EEservopos=1.0;
            }
            if(EEservopos<=0.0){
                EEservopos=0.0;
            }
            EEservo.setPosition(EEservopos);
            telemetry.addLine("Servo Pos " + Double.toString(EEservopos));

        //inverse kinematics: calculating motor angles
            theta1 = gettheta1(x, y, phi, l1, l2, l3);
            theta2 = gettheta2(x, y, phi, l1, l2, l3);
            theta3 = gettheta3(phi, theta1, theta2);
            telemetry.addLine(Double.toString(theta1) + " " + Double.toString(theta2) + " " +  Double.toString(theta3));
            telemetry.addLine(Integer.toString((int)Math.round((theta2/360)*1120)));
        //autonomous arm movement
            //nevrest 40 = 1120 ticks per rotation (motorT)
            //nevrest 20 = 560 ticks per rotation (motorB) //need to recalculate due to gearbox
            if(gamepad1.right_bumper){
                motorT.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorT.setTargetPosition(-1*(int)Math.round((theta2/360)*1120));
                motorB.setTargetPosition(-1*(int)Math.round((theta1/360)*560));
                EEservo.setPosition((int)Math.round((theta1/180)*1));
                motorT.setPower(.5);
                motorB.setPower(.5);
                while(motorT.isBusy()){

                }
                while(motorB.isBusy()){

                }
                motorT.setPower(0);
                motorB.setPower(0);
            }
            if(gamepad1.a){
                motorT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorT.setPower(.4);
                while(motorT.getCurrentPosition()<(900-70)){

                }
                motorT.setPower(0);
            }
            if(gamepad1.y){
                motorT.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorT.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    //inverse kinematics functions
    private static double gettheta1(double xval, double yval, double phival, double l1val, double l2val, double l3val){
        return((((Math.PI/2)-Math.acos((Math.pow((xval-(l3val*Math.sin(phival))),2)+
                Math.pow((yval-(l3val*Math.cos(phival))),2)+Math.pow(l1val, 2)-Math.pow(l2val,2))/
                (2*l1val*Math.sqrt((Math.pow((xval-(l3val*Math.sin(phival))),2))+(Math.pow((yval-(l3val*Math.cos(phival))),2)))))
                -Math.atan((yval-(l3val*Math.cos(phival)))/(xval-(l3val*Math.sin(phival)))))/ Math.PI)*180);
    }
    private static double gettheta2(double xval, double yval, double phival, double l1val, double l2val, double l3val){
        return(((Math.PI-Math.acos((Math.pow(l1val, 2)+
                Math.pow(l2val,2)-Math.pow((xval-(l3val*Math.sin(phival))),2)-Math.pow((yval-(l3val*Math.cos(phival))),2))/
                (2*l1val*l2val)))/ Math.PI)*180);
    }
    private static double gettheta3(double phival, double t1, double t2){
        t1 = (t1/180)*Math.PI;
        t2 = (t2/180)*Math.PI;
        return(((phival-t1-t2)/ Math.PI)*180);
    }

    @Override
    public void stop() {
    }

}
