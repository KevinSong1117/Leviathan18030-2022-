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
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="teleOp", group="teleOp")

public class testing extends OpMode {
    //public CRServo IR;
    //public CRServo IL;
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;  // instantiates motor variables
    public DcMotor bR;
    private ElapsedTime runtime = new ElapsedTime();
    //public DcMotor LTL; // lift turn left
    //public DcMotor LTR; // lift turn right

    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        //IR = hardwareMap.get(CRServo.class, "IR");
        //IL = hardwareMap.get(CRServo.class, "IL");

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);   // Initiates the motors
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
        double limitPower = .5;
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        double leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        double rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


        fL.setPower(leftPower * limitPower);
        fR.setPower(rightPower * limitPower); // Sets the power of the motors to currently half the power
        bL.setPower(leftPower * limitPower);
        bR.setPower(rightPower * limitPower);

        telemetry.addData("Left: ", leftPower);
        telemetry.addData("Right: ", rightPower);
        telemetry.update();

        if(gamepad1.x) fL.setPower(.5);
        if(gamepad1.a) fR.setPower(.5);
        if(gamepad1.b) bR.setPower(.5);
        if(gamepad1.y) bL.setPower(.5);

        /*if(gamepad1.a){
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
        }*/
    }

    @Override
    public void stop() {
    }
}
