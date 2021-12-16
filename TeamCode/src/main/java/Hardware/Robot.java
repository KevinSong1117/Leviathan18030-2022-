package Hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.Sensors;
import org.firstinspires.ftc.teamcode.auto.vision;

public class Robot extends LinearOpMode
{
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
    public BNO055IMU imu;
    Orientation angles;
    float curHeading;
    Sensors gyro;
    private vision v;

    public void init
    {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        L = hardwareMap.get(DcMotor.class, "ER");

        I = hardwareMap.get(CRServo.class, "IR");
        WR = hardwareMap.get(CRServo.class, "WR");
        WL = hardwareMap.get(CRServo.class, "WL");
        DG = hardwareMap.get(DcMotor.class, "DG");


        I.setDirection(CRServo.Direction.REVERSE);
        WR.setDirection(CRServo.Direction.FORWARD);
        WL.setDirection(CRServo.Direction.REVERSE);

        FR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        L.setDirection(DcMotor.Direction.FORWARD);
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
    }

    @Override
    public void runOpMode() throws InterruptedException {
        init;
    }

    public void spinDucks(double power, long time) { // Sets power to rubber duck spinner for a set amount of time and then stops
        DG.setPower(power);
        sleep(time);
        DG.setPower(0);
    }

    public void moveForward(double tics, double power) {
        while (!isStopRequested() && opModeIsActive()) {
            resetEncoder();
            while ((getTic() < tics) && opModeIsActive()) {
                startMotors(power, power);
            }
            stopMotors();
            break;
        }
    }
    public void lift(long height){ //Lifts the arm up to height value which is the milisec amount for sleep()
        L.setPower(.6);
        sleep(height);
        L.setPower(.2);
    }
    public void deliver(){  // Sets the power to outtake wheels fo 3 seconds and stops them
        I.setPower(-.5);
        sleep(3000);
        I.setPower(0);
    }
    public void down(){ //Sets power so that arm slowly goes down
        I.setPower(.001);
    }
    public void deliverA(String level){
        if(level.equals("3")){
            lift(275);
            WR.setPower(-.5);
            WL.setPower(-.5);
            sleep(1000);
            moveForward(510, .5);
            deliver();
            moveForward(300, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            sleep(100);
            turn(-140,.5);
            moveForward(1800,.9);
        }
        else if(level.equals("2")){
            lift(420);
            WR.setPower(-.5);
            WL.setPower(-.5);
            sleep(1000);
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
            sleep(1000);
            moveForward(650, .5);
            deliver();
            moveForward(400, -.5);
            down();
            WR.setPower(.5);
            WL.setPower(.5);
            sleep(100);
            turn(-140,.5);
            moveForward(1600,.9);
        }
    }

    public void turn(double degree, double power){
        while (opModeIsActive() && !isStopRequested()) {
            if (angleWrapDeg(degree - gyro.getAngle()) > 0) {
                while ((angleWrapDeg(degree - gyro.getAngle()) > 0) && opModeIsActive()) {
                   startMotors(power, -power);
                }

            } else {
                while ((angleWrapDeg(degree - gyro.getAngle()) < 0) && opModeIsActive()) {
                    startMotors(-power, power);
                }
            }
            stopMotors();
            break;
        }
    }

    public double getEncoderAvg(){
        double flEncoder = FL.getCurrentPosition();
        double frEncoder = FR.getCurrentPosition();
        double blEncoder = BL.getCurrentPosition();
        double brEncoder = BR.getCurrentPosition();

        double ret = flEncoder + frEncoder + blEncoder + brEncoder;
        ret /= 4;

        return ret;
    }
    private void checkOrientation() {
        // read the orientation of the robot
        angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        // and save the heading
        curHeading = angles.firstAngle; //Gets the orientation of the robot
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
        telemetry.addData("right power", right);
        telemetry.addData("left power", left);
        telemetry.update();
    }
    public void stopMotors() {
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
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
