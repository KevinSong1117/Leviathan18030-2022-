/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.auto.Sensors;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="teleop", group="teleop")

public  class test extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;  // instantiates motor variables
    public DcMotor BR;
    public DcMotor LTL; // lift turn left
    public DcMotor LTR; // lift turn right
    public DcMotor ER;  // lift extend right
    public DcMotor EL;  // lift extend left
    public CRServo IR;
    public CRServo WR;  // Wrist Right
    public CRServo WL;  // Wrist Left




    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
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

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        EL.setDirection(DcMotor.Direction.FORWARD);
        ER.setDirection(DcMotor.Direction.REVERSE);
        LTR.setDirection(DcMotor.Direction.FORWARD);
        LTL.setDirection(DcMotor.Direction.REVERSE);


        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LTR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LTL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        WR.setDirection(CRServo.Direction.REVERSE);
        WL.setDirection(CRServo.Direction.FORWARD);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
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
    public void loop() {

        double limitPower = .8; // percent of power
        double driveL = gamepad1.left_stick_y;
        double driveR  =  gamepad1.right_stick_x;
        if(gamepad1.right_stick_x > -.1 && gamepad1.right_stick_x < .1){
            BL.setPower(driveL);
            BR.setPower(driveL * -1);
            FL.setPower(driveL);
            FR.setPower(driveL * -1);
        }
        else{
            BL.setPower(driveR);
            BR.setPower(driveR);
            FL.setPower(driveR);
            FR.setPower(driveR);
        }



        telemetry.addData("Left: ", driveL);
        telemetry.addData("Right: ", driveR);
        telemetry.update();
        telemetry.update();
        // tests encoder
        telemetry.addData("fl", FL.getCurrentPosition());
        telemetry.addData("fr", FR.getCurrentPosition());
        telemetry.addData("bl", BL.getCurrentPosition());
        telemetry.addData("br", BR.getCurrentPosition());
        telemetry.update();
        double power = 0;
        double position = 0;
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
        if(gamepad2.x){
            IR.setPower(0);
        }
        if(gamepad2.right_bumper) {
            position = .3;
            WR.setPower(position);
            WR.setPower(position);
        }
        WR.setPower(0);
        WR.setPower(0);
        if(gamepad2.left_bumper) {
            position = -.3;
            WR.setPower(position);
            WL.setPower(position);
        }
        WR.setPower(position);
        WR.setPower(position);
        double extendPower = gamepad2.left_stick_y;
        double armPosition = gamepad2.right_stick_y;
        // test this for extending lift
        ER.setPower(extendPower);
        // moving the right motor towards the front of robot = retract
        // moving the left motor towards the back of the robot = extend
        EL.setPower(extendPower);
        if(armPosition <= .1 && armPosition >= -.1){
            armPosition = 0;
            LTR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            LTL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        LTR.setPower(armPosition);
        LTL.setPower(armPosition);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
