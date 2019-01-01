package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by adars on 11/4/2018.
 */

@TeleOp(name="armtester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice


public class armtester extends OpMode {
    private DcMotor motorT; //motor top
    private DcMotor motorB; //motor base
    private ElapsedTime runtime = new ElapsedTime();
    private double AB = .5;
    private double LB = .5;
    private double AT = .5;
    private double LT = .5;
    private double powerB = 0;
    private double powerT = 0;
    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");
        motorB = hardwareMap.dcMotor.get("mb");
        motorT = hardwareMap.dcMotor.get("mt");
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop(){
        telemetry.addData("Status", "Running: " + runtime.toString());


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
    @Override
    public void stop() {
    }

}
