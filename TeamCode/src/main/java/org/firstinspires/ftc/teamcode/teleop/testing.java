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
    public Servo WR;  // Wrist Right
    public Servo WL;  // Wrist Left

    @Override
    public void runOpMode() throws InterruptedException {
        fL = hardwareMap.get(DcMotor.class, "FL");
        fR = hardwareMap.get(DcMotor.class, "FR");
        bL = hardwareMap.get(DcMotor.class, "BL");
        bR = hardwareMap.get(DcMotor.class, "BR");

        WR = hardwareMap.get(Servo.class, "WR");
        WL = hardwareMap.get(Servo.class, "WL");

        WR.setDirection(Servo.Direction.FORWARD);
        WL.setDirection(Servo.Direction.REVERSE);

        fR.setDirection(DcMotor.Direction.FORWARD);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.REVERSE);



        fR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);




        waitForStart();
        while(!isStopRequested()){
            if (gamepad1.left_bumper){
                WR.setPosition(.36);
                WL.setPosition(.13);
            }
            if (gamepad1.right_bumper){
                WR.setPosition(.7);
                WL.setPosition(.4);
            }
            telemetry.addData("left", gamepad1.left_trigger);
            telemetry.addData("right", gamepad1.right_trigger);
            telemetry.update();

        }
        WR.scaleRange(.36,.68);
        WL.scaleRange(.13,.38);

//left straight = .13
        //right straight = .36
        //left up = .4
        //right up = .7

        //IF THE WRIST EVER POINTS DOWN, YOU ARE BONED. GG. UNLUCKY


    }






}
