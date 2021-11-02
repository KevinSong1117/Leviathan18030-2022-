package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="teleOp", group="teleOp")

public class testing extends OpMode {
    public CRServo IR;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        IR = hardwareMap.get(CRServo.class, "IR");
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
        double power = 1;
        telemetry.addData("power: ", power);
        IR.setPower(power);
    }

    @Override
    public void stop() {
    }
}
