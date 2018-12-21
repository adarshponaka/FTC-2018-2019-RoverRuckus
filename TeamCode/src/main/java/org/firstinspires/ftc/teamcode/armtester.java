package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by adars on 11/4/2018.
 */
//pavan

@TeleOp(name="armtester", group="Iterative Opmode")  // @Autonomous(...) is the other common choice


public class armtester extends OpMode {
    private DcMotor motorT; //motor top
    private DcMotor motorB; //motor base
    private ElapsedTime runtime = new ElapsedTime();
    private double A = .5;
    private double L = .5;
    private int c = 1;
    @Override
    public void init(){
        telemetry.addData("Status", "Initialized");

        motorB = hardwareMap.dcMotor.get("m1");
        motorT = hardwareMap.dcMotor.get("m2");


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
        if (gamepad1.left_trigger > 0.9) {
            c = -1;
        }
        if (gamepad1.right_trigger > 0.9) {
            c = 1;
        }
        L = getL(gamepad1.left_stick_x, gamepad1.left_stick_y, c*Math.PI / 4);
        A = getA(gamepad1.left_stick_x, gamepad1.left_stick_y, c*Math.PI / 4);
        motorB.setPower((0.5 * L * A)*c);
        telemetry.addLine("mb: " + Double.toString(0.9 * L * A));

        L = getL(gamepad1.right_stick_x, gamepad1.right_stick_y, c*Math.PI / 4);
        A = getA(gamepad1.right_stick_x, gamepad1.right_stick_y, c*Math.PI / 4);
        motorT.setPower((0.5 * L * A)*c);
        telemetry.addLine("mt: " + Double.toString(0.9 * L * A));

        telemetry.addLine("mb: " + Integer.toString(c));

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
