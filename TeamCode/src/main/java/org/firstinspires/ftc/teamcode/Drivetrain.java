package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Drivetrain {
    public DcMotor fl;
    public DcMotor fr;
    public DcMotor bl;
    public DcMotor br;


    public drivetrain(LinearOpMode opMode) throws InterruptedException {
        this.opMode = opMode;
        gyro = new Sensors(opMode);
        timer = new ElapsedTime();

        fr = this.opMode.hardwareMap.get(DcMotor.class, "fR");
        fl = this.opMode.hardwareMap.get(DcMotor.class, "fL");
        br = this.opMode.hardwareMap.get(DcMotor.class, "bR");
        bl = this.opMode.hardwareMap.get(DcMotor.class, "bL");

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //the left side of the robot has reversed motors meaning they need to be reversed in order to make them turn in the right way
        fr.setDirection(DcMotor.Direction.FORWARD);
        fl.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.REVERSE);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void resetEncoder() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        opMode.idle();

        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        opMode.idle();
    }
}

