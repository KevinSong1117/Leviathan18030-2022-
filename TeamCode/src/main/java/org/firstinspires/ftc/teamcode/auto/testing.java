package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleOp", group="teleOp")

public class testing extends OpMode {
    public CRServo IR;
    public CRServo IL;

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor LTL; // lift turn left
    public DcMotor LTR; // lift turn right

    @Override
    public void init() {
        IR = hardwareMap.get(CRServo.class, "IR");
        IL = hardwareMap.get(CRServo.class, "IL");

        /*LTR = hardwareMap.get(DcMotor.class, "LTR");
        LTL = hardwareMap.get(DcMotor.class, "LTL");

        LTL.setDirection(DcMotor.Direction.FORWARD);
        LTR.setDirection(DcMotor.Direction.REVERSE);

        LTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

       /*while(LTR.getCurrentPosition() < 500 && LTL.getCurrentPosition() < 500){
           LTR.setPower(.1);
           LTL.setPower(.1);

       }
       LTR.setPower(0);
       LTL.setPower(0);*/
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

    public void loop(){
        double power = 0;

        if(gamepad1.a){
            power = .5;
            telemetry.addData("power: ", power);
            IR.setPower(power);
            IL.setPower(power);

        }
        if(gamepad1.b){
            power = -.5;
            telemetry.addData("power: ", power);
            IR.setPower(power);
            IL.setPower(power);
        }
        if(gamepad1.x){
            power = 0;
            telemetry.addData("power: ", power);
            IR.setPower(power);
            IL.setPower(power);
        }
    }

    @Override
    public void stop() {
    }
}
