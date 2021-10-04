package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name="teleop", group="teleop")

public class auto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor fL;
    public DcMotor fR;
    public DcMotor bL;
    public DcMotor bR;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");

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
        double x = gamepad1.right_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double magnitude = Math.hypot(x, y);

        double fl = y + turn - x;
        double fr = y - turn - x;
        double bl = y + turn + x;
        double br = y - turn + x;

        double max = 0;
        max = Math.max(Math.abs(fl), Math.abs(br));
        max = Math.max(Math.abs(fr), max);
        max = Math.max(Math.abs(bl), max);

        //only normalize if mag isnt 0 because if it is, we want to turn and will always be from 0-1
        if (magnitude != 0) {
            //Divide everything by max (it's positive so we don't need to worry
            //about signs)
            //multiply by input magnitude as it represents true speed (from 0-1) that we want robot to move at
            fl = (fl / max) * magnitude;
            fr = (fr / max) * magnitude;
            bl = (bl / max) * magnitude;
            br = (br / max) * magnitude;
        }
        telemetry.addData("fl: ", fl);
        telemetry.addData("fr: ", fr);
        telemetry.addData("bl: ", bl);
        telemetry.addData("br ", br);
        telemetry.update();

        fL.setPower(fl);
        fR.setPower(fr);
        bL.setPower(-bl);
        bR.setPower(-br);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
