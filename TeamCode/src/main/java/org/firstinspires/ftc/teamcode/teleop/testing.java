package org.firstinspires.ftc.teamcode.teleop;


import android.os.Build;

import androidx.annotation.RequiresApi;

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
import org.firstinspires.ftc.teamcode.auto.vision;

@TeleOp(name="teleOp", group="teleOp")

public class testing extends LinearOpMode {
    private vision vision;

    @RequiresApi(api = Build.VERSION_CODES.Q)
    @Override
    public void runOpMode() throws InterruptedException {
        while(!isStarted()){
            vision.gettrueColor(322, 200);
        }


        //IF THE WRIST EVER POINTS DOWN, YOU ARE BONED. GG. UNLUCKY


    }






}
