package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="armnew", group="Iterative Opmode")  // @Autonomous(...) is the other common choice

public class armreverse extends OpMode {
    private DcMotor motorB; //motor base
    @Override
    public void init() {
        motorB = hardwareMap.dcMotor.get("mb");

    }
    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.5) {
            motorB.setPower(1);
        } else if (gamepad1.left_bumper) {
            motorB.setPower(-1);
        } else {
            motorB.setPower(0);
        }

    }



}




