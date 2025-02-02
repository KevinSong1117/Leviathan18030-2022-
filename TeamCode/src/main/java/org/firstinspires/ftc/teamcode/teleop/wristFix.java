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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

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

@TeleOp(name="teleopFix", group="wristFix")

public  class wristFix extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;  // instantiates motor variables
    public DcMotor BR;
    public DcMotor L;  // lift
    public CRServo I;  // intake
    public CRServo WR;  // Wrist Right
    public CRServo WL;  // Wrist Left
    public DcMotor DG;
    public DcMotor DG1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        L = hardwareMap.get(DcMotor.class, "L");

        I = hardwareMap.get(CRServo.class, "I");
        WR = hardwareMap.get(CRServo.class, "WR");
        WL = hardwareMap.get(CRServo.class, "WL");
        DG = hardwareMap.get(DcMotor.class, "DG");
        DG1 = hardwareMap.get(DcMotor.class, "DG1");

        I.setDirection(CRServo.Direction.REVERSE);
        WR.setDirection(CRServo.Direction.FORWARD);
        WL.setDirection(CRServo.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        L.setDirection(DcMotor.Direction.FORWARD);
        DG.setDirection((DcMotorSimple.Direction.FORWARD));
        DG1.setDirection((DcMotorSimple.Direction.FORWARD));


        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DG.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        DG1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        L.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        DG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DG1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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

    public void timeTest(int waitTime) throws InterruptedException {
        sleep(waitTime);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    double driveL;
    double driveR;
    int checker = 0;
    @Override
    public void loop() {



        if(Math.abs(gamepad1.left_stick_y) > .1){
            driveR = gamepad1.left_stick_y * .9;
            BL.setPower(driveR);
            BR.setPower(driveR * .85);
            FL.setPower(driveR);
            FR.setPower(driveR * .85);
        }
        else{
            driveL  =  gamepad1.right_stick_x * .6;
            BL.setPower(driveL);
            BR.setPower(driveL * -1);
            FL.setPower(driveL);
            FR.setPower(driveL * -1);
        }

        if (gamepad2.left_bumper){
            // straight
            WR.setPower(.5);
            WL.setPower(.5);


        }

        else if (gamepad2.right_bumper){
            // up
            WR.setPower(-.5);
            WL.setPower(-.5);
        }

        if(gamepad1.right_trigger > .5){
            DG.setPower(gamepad1.right_trigger * -.8);
        }

        if(gamepad1.left_trigger > .5){
            DG1.setPower(gamepad1.left_trigger * .8);
        }
        if(gamepad1.right_bumper){
            DG.setPower(0);
            DG1.setPower(0);
        }

        double power;

        if(gamepad2.a){
            power = .5;
            I.setPower(power);

        }
        if(gamepad2.b){
            power = -.5;
            I.setPower(power);
        }
        if(gamepad2.x){
            I.setPower(0);
        }

        if(gamepad1.left_bumper){
            if(checker == 0){
                driveR *= .5;
                checker += 1;
            }

            else{
                driveR *= 2;
                checker -= 1;
            }

        }
        if(gamepad2.dpad_up){
            ElapsedTime current = new ElapsedTime();
            double time = current.milliseconds();
            while (time < 630){
                L.setPower(-.6);
                driveR = gamepad1.left_stick_y * .8;
                driveL  =  gamepad1.right_stick_x * .5;
                if(Math.abs(gamepad1.left_stick_y) > .1){
                    BL.setPower(driveR);
                    BR.setPower(driveR );
                    FL.setPower(driveR);
                    FR.setPower(driveR);
                }
                else{
                    BL.setPower(driveL);
                    BR.setPower(driveL * -1);
                    FL.setPower(driveL);
                    FR.setPower(driveL * -1);
                }
                time = current.milliseconds();
            }

            L.setPower(-.2);
            WR.setPower(-.5);
            WL.setPower(-.5);
        }
        if(gamepad2.dpad_down){
            WR.setPower(.5);
            WL.setPower(.5);
            ElapsedTime current = new ElapsedTime();
            double time = current.milliseconds();
            while (time < 2500){
                L.setPower(-.0001);
                driveR = gamepad1.left_stick_y ;
                driveL  =  gamepad1.right_stick_x;
                if(Math.abs(gamepad1.left_stick_y) > .1){
                    BL.setPower(driveR);
                    BR.setPower(driveR * .9);
                    FL.setPower(driveR);
                    FR.setPower(driveR * .9);
                }
                else{
                    BL.setPower(driveL);
                    BR.setPower(driveL * -.9);
                    FL.setPower(driveL);
                    FR.setPower(driveL * -.9);
                }
                time = current.milliseconds();
            }
            L.setPower(0);
        }

        double extendPower;
        // some sort of sine function so that it is negative on the right and positive on the left
        double staticPower;

        // test this for extending lift
        // Static equilibrium (free body diagram and phy shi)
        if(gamepad2.left_stick_y > .1){
            staticPower = -.0005;
            extendPower = gamepad2.right_stick_y ;
            L.setPower(staticPower + (extendPower * .35));
        }
        else{
            staticPower = -.2;
            extendPower = gamepad2.left_stick_y;
        }

        L.setPower(staticPower + (extendPower * .35));
        telemetry.addData("Lift position ", L.getPower());
        telemetry.addData("encoder", L.getCurrentPosition());
        telemetry.addData("Time :", runtime);
        telemetry.addData("FR", FR.getCurrentPosition());
        telemetry.addData("BR", BR.getCurrentPosition());
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
