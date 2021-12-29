package Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.Sensors;
import org.firstinspires.ftc.teamcode.auto.vision;

public class Robot
{
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap hwMap = null;
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;  // instantiates motor variables
    public DcMotor BR = null;
    public DcMotor L = null;  // lift
    public CRServo I = null;  // intake
    public CRServo WR = null;  // Wrist Right
    public CRServo WL = null;  // Wrist Left
    public DcMotor DG = null;
    //public BNO055IMU imu;
    LinearOpMode opmode;
    Sensors gyro;

    public Robot(LinearOpMode opMode)throws InterruptedException
    {
        //hwMap = hardwareMap;


        FL = this.opmode.hardwareMap.get(DcMotor.class, "FL");
        FR = this.opmode.hardwareMap.get(DcMotor.class, "FR");
        BL = this.opmode.hardwareMap.get(DcMotor.class, "BL");
        BR = this.opmode.hardwareMap.get(DcMotor.class, "BR");

        L = this.opmode.hardwareMap.get(DcMotor.class, "L");

        I = this.opmode.hardwareMap.get(CRServo.class, "I");
        WR = this.opmode.hardwareMap.get(CRServo.class, "WR");
        WL = this.opmode.hardwareMap.get(CRServo.class, "WL");
        DG = this.opmode.hardwareMap.get(DcMotor.class, "DG");

        gyro = new Sensors(opMode);
        this.opmode = opMode;

        I.setDirection(CRServo.Direction.REVERSE);
        WR.setDirection(CRServo.Direction.FORWARD);
        WL.setDirection(CRServo.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        BL.setDirection(DcMotor.Direction.REVERSE);
        L.setDirection(DcMotor.Direction.REVERSE);
        DG.setDirection((DcMotorSimple.Direction.FORWARD));


        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DG.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        L.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*
        imu = opmode.hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

         */
    }


    public double getTic() {
        double count = 4;
        if (FR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (FL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (BR.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        if (BL.getCurrentPosition() == 0) {
            count -= 1.0;
        }
        double totaldis = Math.abs(FR.getCurrentPosition()) + Math.abs(FL.getCurrentPosition()) + Math.abs(BL.getCurrentPosition()) + Math.abs(BR.getCurrentPosition());
        if (count == 0) {
            return 1;
        }
        return totaldis / count;
    }

    double angleWrapDeg(double angle) {
        double correctAngle = angle;
        while (correctAngle > 180)
        {
            correctAngle -= 360;
        }
        while (correctAngle < -180)
        {
            correctAngle += 360;
        }
        return correctAngle;
    }

    public void resetEncoder() {
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void startMotors(double right, double left) {
        FR.setPower(right);
        FL.setPower(left);
        BL.setPower(left);
        BR.setPower(right);
        opmode.telemetry.addData("right power", right);
        opmode.telemetry.addData("left power", left);
        opmode.telemetry.update();
    }
    public void stopMotors() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
    }

    public void moveForward(double tics, double power) {
        while (!opmode.isStopRequested() && opmode.opModeIsActive()) {
            resetEncoder();
            while ((getTic() < tics) && opmode.opModeIsActive()) {
                startMotors(power, power);
            }
            stopMotors();
            break;
        }
    }

    public void turn(double degree, double power){
        while (opmode.opModeIsActive() && !opmode.isStopRequested()) {
            if (angleWrapDeg(degree - gyro.getAngle()) > 0) {
                while ((angleWrapDeg(degree - gyro.getAngle()) > 0) && opmode.opModeIsActive()) {
                    startMotors(power, -power);
                }

            } else {
                while ((angleWrapDeg(degree - gyro.getAngle()) < 0) && opmode.opModeIsActive()) {
                    startMotors(-power, power);
                }
            }
            stopMotors();
            break;
        }
    }

    public void spinDucks(double power, long time) { // Sets power to rubber duck spinner for a set amount of time and then stops
        DG.setPower(power);
        opmode.sleep(time);
        DG.setPower(0);
    }

    public void lift(long height){ //Lifts the arm up to height value which is the milisec amount for sleep()
        L.setPower(.6);
        opmode.sleep(height);
        L.setPower(.2);
    }
    public void deliver(){  // Sets the power to outtake wheels fo 3 seconds and stops them
        I.setPower(-.5);
        opmode.sleep(3000);
        I.setPower(0);
    }
    public void down(){ //Sets power so that arm slowly goes down
        I.setPower(.001);
    }

    public void deliverBlue(String level){
        moveForward(200, -.5);
        turn(100, .5);
        if(level.equals("3")){
            lift(275);
            WR.setPower(-.5);
            WL.setPower(-.5);
            opmode.sleep(1000);
            moveForward(510, .5);
            deliver();
            moveForward(300, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            opmode.sleep(100);
            turn(-140,.5);
            moveForward(1800,.9);
        }
        else if(level.equals("2")){
            lift(420);
            WR.setPower(-.5);
            WL.setPower(-.5);
            opmode.sleep(1000);
            moveForward(550, .5);
            deliver();
            moveForward(300, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            turn(-140,.5);
            moveForward(1800,.9);
        }
        else{
            lift(630);
            WR.setPower(-.5);
            WL.setPower(-.5);
            opmode.sleep(1000);
            moveForward(650, .5);
            deliver();
            moveForward(400, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            opmode.sleep(100);
            turn(-140,.5);
            moveForward(1600,.9);
        }
    }

    public void deliverRed(String level)
    {
        moveForward(200, -.5);
        turn(-110, .5);
        if(level.equals("3")){
            lift(275);
            WR.setPower(-.5);
            WL.setPower(-.5);
            opmode.sleep(1000);
            moveForward(500, .5);
            deliver();
            moveForward(300, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            opmode.sleep(100);
            turn(140,.5);
            moveForward(1800,.9);
        }
        else if(level.equals("2")){
            lift(420);
            WR.setPower(-.5);
            WL.setPower(-.5);
            opmode.sleep(1000);
            moveForward(550, .5);
            deliver();
            moveForward(300, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            turn(140,.5);
            moveForward(1800,.9);
        }
        else{
            lift(630);
            WR.setPower(-.5);
            WL.setPower(-.5);
            opmode.sleep(1000);
            moveForward(650, .5);
            deliver();
            moveForward(400, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            opmode.sleep(100);
            turn(140,.5);
            moveForward(1600,.9);
        }
    }



    /* PID

        public void movePIDFGyro(double inches, double kp, double ki, double kd, double f, double threshold, double time){
        timer.reset();
        resetEncoder();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();

        double initialError = Math.abs(inches); //-20
        double error = initialError;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;


        while (timeAtSetPoint < time) {
            if (inches < 0){
                error = inches + getTic() / COUNTS_PER_INCH;
            }
            else{
                error = inches - getTic() / COUNTS_PER_INCH;
            }

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / initialError;
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;

            double difference = gyro.angleDiff(initialHeading);

            if (difference > .4){
                if (power > 0) {
                    startMotors((power + f), (power + f) * .8);
                }
                else {
                    startMotors((power - f) * .8, (power - f));
                }
            }
            else if(difference < -.5){
                if (power > 0) {
                    startMotors((power + f), (power + f));
                }
                else {
                    startMotors((power - f),(power - f));
                }
            }
            else{
                if (power > 0) {
                    startMotors(power + f,  power + f);
                }
                else {
                    startMotors(power - f,power - f);
                }
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }
    public void turnHeading(double finalAngle, double kp, double ki, double kd, double f, double threshold, double time) {
        timer.reset();

        double pastTime = 0;
        double currentTime = timer.milliseconds();

        double initialHeading = gyro.getAngle();
        finalAngle = angleWrapDeg(finalAngle);

        double initialAngleDiff = angleWrapDeg(finalAngle - initialHeading);
        double error = initialAngleDiff;
        double pastError = error;

        double integral = 0;

        double timeAtSetPoint = 0;
        double firstTimeAtSetPoint = 0;
        boolean atSetpoint = false;

        while (timeAtSetPoint < time) {
            error = gyro.newAngleDiff(gyro.getAngle(), finalAngle);

            currentTime = timer.milliseconds();
            double dt = currentTime - pastTime;

            double proportional = error / Math.abs(initialAngleDiff);
            integral += dt * ((error + pastError) / 2.0);
            double derivative = (error - pastError) / dt;

            double power = kp * proportional + ki * integral + kd * derivative;
            if (power > 0) {
                if (Math.abs(kp) < .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors(-power - f, power + f);
            }
            else{
                if (Math.abs(kp) < .0001){
                    power = 0 * proportional + ki * integral + kd * derivative;
                }
                startMotors(-power + f, power - f);
            }

            if (Math.abs(error) < threshold){
                if (!atSetpoint){
                    atSetpoint = true;
                    firstTimeAtSetPoint = currentTime;
                }
                else{
                    timeAtSetPoint = currentTime - firstTimeAtSetPoint;
                }
            }
            else{
                atSetpoint = false;
            }

            pastTime = currentTime;
            pastError = error;
        }
        stopMotors();
    }

     */
}
