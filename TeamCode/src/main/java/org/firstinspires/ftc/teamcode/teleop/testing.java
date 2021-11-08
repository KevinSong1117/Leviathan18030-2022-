package org.firstinspires.ftc.teamcode.teleop;


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

import org.firstinspires.ftc.teamcode.auto.Sensors;

@TeleOp(name="teleOp", group="teleOp")

public class testing extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;  // instantiates motor variables
    public DcMotor bR;
    public DcMotor LTL; // lift turn left
    public DcMotor LTR; // lift turn right
    public DcMotor ER;  // lift extend right
    public DcMotor EL;  // lift extend left
    public CRServo IR;
    public CRServo WR;  // Wrist Right
    public CRServo WL;  // Wrist Left

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "FL");
        fR = hardwareMap.get(DcMotor.class, "FR");
        bL = hardwareMap.get(DcMotor.class, "BL");
        bR = hardwareMap.get(DcMotor.class, "BR");
        LTL = hardwareMap.get(DcMotor.class, "LTL");
        LTR = hardwareMap.get(DcMotor.class, "LTR");
        ER = hardwareMap.get(DcMotor.class, "EL");
        EL = hardwareMap.get(DcMotor.class, "ER");

        IR = hardwareMap.get(CRServo.class, "IR");
        WR = hardwareMap.get(CRServo.class, "WR");
        WL = hardwareMap.get(CRServo.class, "WL");

        IR.setDirection(CRServo.Direction.REVERSE);
        WR.setDirection(CRServo.Direction.FORWARD);
        WL.setDirection(CRServo.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);
        EL.setDirection(DcMotor.Direction.FORWARD);
        ER.setDirection(DcMotor.Direction.REVERSE);
        LTR.setDirection(DcMotor.Direction.FORWARD);
        LTL.setDirection(DcMotor.Direction.REVERSE);


        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       /*while(LTR.getCurrentPosition() < 500 && LTL.getCurrentPosition() < 500){
           LTR.setPower(.1);
           LTL.setPower(.1);

       }
       LTR.setPower(0);
       LTL.setPower(0);*/

        waitForStart();

        double limitPower = .8; // percent of power
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
        while(opModeIsActive()){
            // test gyro
            telemetry.update();
        }
        while(opModeIsActive()){
            // tests encoder
            telemetry.addData("fl", fL.getCurrentPosition());
            telemetry.addData("fr", fR.getCurrentPosition());
            telemetry.addData("bl", bL.getCurrentPosition());
            telemetry.addData("br", bR.getCurrentPosition());
            telemetry.update();
        }
        double power = 0;
        if(gamepad2.a){
            power = .5;
            telemetry.addData("power: ", power);
            IR.setPower(power);

        }
        if(gamepad2.b){
            power = -.5;
            telemetry.addData("power: ", power);
            IR.setPower(power);
        }

        if(gamepad2.x) {
            WR.setPower(.1);
            WL.setPower(.1);
        }
        if(gamepad2.y) {
            WR.setPower(-.1);
            WL.setPower(-.1);
        }
        double extendPower = gamepad2.left_stick_y;
        double armPosition = gamepad2.right_stick_x;
        // test this for extending lift
        ER.setPower(extendPower);
        // moving the right motor towards the front of robot = retract
        // moving the left motor towards the back of the robot = extend
        EL.setPower(extendPower);

        LTR.setPower(armPosition);
        LTL.setPower(armPosition);

    }






}
