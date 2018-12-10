package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 * This is a class that:
 *    - Contains all of your robot hardware
 *    - Does all initialization code
 *    - Has all of the methods for driving and turning
 *
 *    - This example is for the robot named Suspect, built by Astrid and Ari in 45 min
 */
public class Beholder
{
    // declare hardware imu, motors, servos, sensors
    BNO055IMU imu;
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor ML = null;
    public DcMotor MR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public DcMotor Intake = null;
    public CRServo IH = null;


    // create arrays for your motors (change sizes to match YOUR number of motors)
    public DcMotor[] LeftMotors = new DcMotor[3];
    public DcMotor[] RightMotors = new DcMotor[3];
    public DcMotor[] AllMotors = new DcMotor[6];

    // you will need a reference to your OpMode
    private LinearOpMode OpModeReference;

    // define and calculate constants...
    //static final double     COUNTS_PER_MOTOR_ORBITAL = 1120 ;    // REV Hex HD 40:1
    static final double     COUNTS_PER_MOTOR_ORBITAL = 537.6 ;    // Orbital 20
    static final double     WHEEL_DIAMETER_INCHES   = 4;     // For figuring circumference
    static final double     WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_ORBITAL / WHEEL_CIRCUMFERENCE_INCHES);


       // this is the CONSTRUCTOR for this class
    // from your OpMode, you'll have to pass a reference to the OpMode as the parameter
    // Will look like this:
    //      Beholder robot = new Beholder(this);
    public Beholder(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    // This is a method for all of the initialization code - all of the
    // stuff that happens after clicking the init button, but before
    // clicking the start button.
    public void init() {

        // this is the IMU crap...just...accept it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // get all your hardware from the hardware map
        // defined in the config on your robot controller phone.
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        ML = OpModeReference.hardwareMap.get(DcMotor.class, "ML");
        MR = OpModeReference.hardwareMap.get(DcMotor.class, "MR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");
        Intake = OpModeReference.hardwareMap.get(DcMotor.class, "Intake");
        IH = OpModeReference.hardwareMap.get(CRServo.class, "IH");
        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "imu");

        // initialize the IMU
        imu.initialize(parameters);

        // now add each motor to your motor arrays (this example only has 2 motors)
        // left
        LeftMotors[0] = FL;
        LeftMotors[1] = ML;
        LeftMotors[2] = BL;
        // right
        RightMotors[0] = FR;
        RightMotors[1] = MR;
        RightMotors[2] = BR;
        // all
        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = ML;
        AllMotors[3] = MR;
        AllMotors[4] = BL;
        AllMotors[5] = BR;

        // set the direction for all left, then all right motors
        for (DcMotor m : LeftMotors)
            m.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : RightMotors)
            m.setDirection(DcMotor.Direction.REVERSE);

        // set any properties that apply to ALL motors
        for (DcMotor m : AllMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // just a method to stop driving
    public void StopDriving() {
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    // this is a drive method - takes speed and inches
    // WARNING: YOU WILL NEED TO IMPLEMENT REVERSE
    public void Drive(double speed, double inches) {
        // Ensure that the opmode is still active
        if (OpModeReference.opModeIsActive()) {

            // calculate the number of ticks you want to travel (cast to integer)
            int targetTicks = (int) (inches * COUNTS_PER_INCH);

            // reset ticks to 0 on all motors
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // set target position on all motors
            // mode must be changed to RUN_TO_POSITION
            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            // turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed);

            // just keep looping while both motors are busy
            // stop if driver station stop button pushed
            while (OpModeReference.opModeIsActive() && (FL.isBusy() && FR.isBusy())) {
                OpModeReference.telemetry.addData("target ticks", targetTicks);
                OpModeReference.telemetry.addData("FRC", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FLC", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("MRC", MR.getCurrentPosition());
                OpModeReference.telemetry.addData("MLC", ML.getCurrentPosition());
                OpModeReference.telemetry.addData("BRC", BR.getCurrentPosition());
                OpModeReference.telemetry.addData("BLC", BL.getCurrentPosition());
                OpModeReference.telemetry.update();
            }

            // once all motors get to where they need to be, turn them off
            StopDriving();

            // set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // this is a method to get the current heading/z angle from the IMU
    // WE WANT THE Z ANGLE :)
    // AxesOrder.XYZ means we want thirdAngle
    // AxesOrder.ZYX would mean we want firstAngle
    public double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    // This method calculates the difference of the current angle from the start angle
    // If you're left of your original angle, the value will be POSITIVE
    // If you're right of your original angle, the value will be NEGATIVE
    public double GetAngleDifference(double startAngle) {
        double angleDifference = GetCurrentZAngle() - startAngle;

        // handle going past the 0 or 180 barriers
        // where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }

    // This method makes the robot turn.
    // DO NOT try to turn more than 180 degrees in either direction
    // targetAngleDifference is the number of degrees you want to turn
    // should be positive if turning left, negative if turning right
    public void Turn(double targetAngleDifference, double power) {

        // before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        // just some boolean variables to tell if we've stepped motor power down
        // might actually want more than two steps
        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        // if target angle is Negative, we're turning RIGHT
        if (targetAngleDifference < 0) {
            // turning right, so we want all right motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(-power);
            for (DcMotor m : LeftMotors)
                m.setPower(power);
            // sleep a tenth of a second
            // WARNING - not sure why this is needed - but sometimes right turns didn't work without
            OpModeReference.sleep(100);

            // we're turning right, so our target angle difference will be negative (ex: -90)
            // so GetAngleDifference will go from 0 to -90
            // keep turning while difference is greater than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power/4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power/2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/2);
                    firstStepDownComplete = true;
                }

                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("FRC", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FLC", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("MRC", MR.getCurrentPosition());
                OpModeReference.telemetry.addData("MLC", ML.getCurrentPosition());
                OpModeReference.telemetry.addData("BRC", BR.getCurrentPosition());
                OpModeReference.telemetry.addData("BLC", BL.getCurrentPosition());
                OpModeReference.telemetry.update();
            }
            // if targetAngleDifference is Positive, we're turning LEFT
        } else if (targetAngleDifference > 0) {
            // turning left so want all left motors going backwards
            for (DcMotor m : RightMotors)
                m.setPower(power);
            for (DcMotor m : LeftMotors)
                m.setPower(-power);

            // WARNING not sure if this sleep is needed - seemed necessary for right turns
            OpModeReference.sleep (100);

            // we're turning right, so our target angle difference will be positive (ex: 90)
            // so GetAngleDifference will go from 0 to 90
            // keep turning while difference is less than target
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                // THIS CODE IS FOR STEPPING DOWN MOTOR POWER
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power/4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power/2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/2);
                    firstStepDownComplete = true;
                }
                OpModeReference.telemetry.addData("target", targetAngleDifference);
                OpModeReference.telemetry.addData("current", GetAngleDifference(startAngle));
                OpModeReference.telemetry.addData("FRC", FR.getCurrentPosition());
                OpModeReference.telemetry.addData("FLC", FL.getCurrentPosition());
                OpModeReference.telemetry.addData("MRC", MR.getCurrentPosition());
                OpModeReference.telemetry.addData("MLC", ML.getCurrentPosition());
                OpModeReference.telemetry.addData("BRC", BR.getCurrentPosition());
                OpModeReference.telemetry.addData("BLC", BL.getCurrentPosition());
                OpModeReference.telemetry.update();


            }
        } else {
            // is zero - not turning - just return
            return;
        }

        // Turn all motors off
        StopDriving();
    }
}
