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
 *    - Contains all of our robot hardware
 *    - Does all initialization code
 *    - Has all of the methods for driving and turning
 */
public class Beholder
{
    //Here we start setting up our motors, servos, and our imu device.
    BNO055IMU imu;
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor ML = null;
    public DcMotor MR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public CRServo Intake = null;
    public DcMotor IH = null;


    //Now we're making arrays for our motors, allowing us to easily reference our right motors and our left motors.
    public DcMotor[] LeftMotors = new DcMotor[3];
    public DcMotor[] RightMotors = new DcMotor[3];
    public DcMotor[] AllMotors = new DcMotor[6];

    //Here we'll represent our opmode.
    private LinearOpMode OpModeReference;

    //Here we define & calculate some math to let us set our motors to run for a set distance.
    static final double     COUNTS_PER_MOTOR_ORBITAL = 537.6 ;    //For our Orbital 20s.
    static final double     WHEEL_DIAMETER_INCHES   = 4;     //For figuring circumference.
    static final double     WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_ORBITAL / WHEEL_CIRCUMFERENCE_INCHES);


    //This part lets us reference this file in our autonomous programs.
    public Beholder(LinearOpMode opMode) {
        OpModeReference = opMode;
    }

    //This bit is a method for all of the initialization code.
    public void init() {

        //This is the imu setup code.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; //Note: see the calibration sample opmode for more info.
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //get all your hardware from the hardware map
        //defined in the config on your robot controller phone.
        FL = OpModeReference.hardwareMap.get(DcMotor.class, "FL");
        FR = OpModeReference.hardwareMap.get(DcMotor.class, "FR");
        ML = OpModeReference.hardwareMap.get(DcMotor.class, "ML");
        MR = OpModeReference.hardwareMap.get(DcMotor.class, "MR");
        BL = OpModeReference.hardwareMap.get(DcMotor.class, "BL");
        BR = OpModeReference.hardwareMap.get(DcMotor.class, "BR");
        Intake = OpModeReference.hardwareMap.get(CRServo.class, "Intake");
        IH = OpModeReference.hardwareMap.get(DcMotor.class, "IH");
        imu = OpModeReference.hardwareMap.get(BNO055IMU.class, "imu");

        //Now we initialize the IMU.
        imu.initialize(parameters);

        //Now we add the motors to the arrays.
        LeftMotors[0] = FL;
        LeftMotors[1] = ML;
        LeftMotors[2] = BL;

        RightMotors[0] = FR;
        RightMotors[1] = MR;
        RightMotors[2] = BR;

        AllMotors[0] = FL;
        AllMotors[1] = FR;
        AllMotors[2] = ML;
        AllMotors[3] = MR;
        AllMotors[4] = BL;
        AllMotors[5] = BR;

        //Now set the direction for all left, then all right motors
        for (DcMotor m : LeftMotors)
            m.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor m : RightMotors)
            m.setDirection(DcMotor.Direction.REVERSE);

        //Now we set any properties that apply to all of the motors
        for (DcMotor m : AllMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //This is just a method to stop driving
    public void StopDriving() {
        for (DcMotor m : AllMotors)
            m.setPower(0);
    }

    //This is a drive method that takes speed and inches.
    public void Drive(double speed, double inches) {
        //Ensure that the opmode is still active.
        if (OpModeReference.opModeIsActive()) {

            //Now we calculate the number of ticks we want to travel (cast to integer).
            int targetTicks = (int) (inches * COUNTS_PER_INCH);

            //Now we reset ticks to 0 on all motors.
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //Now we set target position on all motors.
            //Also, the mode must be changed to RUN_TO_POSITION.
            for(DcMotor m : AllMotors) {
                m.setTargetPosition(targetTicks);
                m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            //Now we turn all motors on!
            for (DcMotor m : AllMotors)
                m.setPower(speed);

            //Now we just keep looping while both motors are busy.
            //It should be able to stop if driver station stop button pushed.
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

            //And once all motors get to where they need to be, turn them off
            StopDriving();

            //Now we set motors back to RUN_USING_ENCODERS
            for (DcMotor m : AllMotors)
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    //This is a method to get the current heading/z angle from the IMU.
    //Brief aside, we want the Z angle for our purposes.
    //AxesOrder.XYZ means we want thirdAngle.
    //AxesOrder.ZYX would mean we want firstAngle.
    public double GetCurrentZAngle() {
        Orientation currentAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return currentAngles.thirdAngle;
    }

    //This method calculates the difference of the current angle from the start angle.
    //If we're left of your original angle, the value will be POSITIVE.
    //If we're right of your original angle, the value will be NEGATIVE.
    public double GetAngleDifference(double startAngle) {
        double angleDifference = GetCurrentZAngle() - startAngle;

        //Now we handle going past the 0 or 180 barriers, where we switch from positive to negative or vice versa
        if (angleDifference < -180)
            angleDifference += 360;
        else if (angleDifference > 180)
            angleDifference -=360;

        return angleDifference;
    }

    //This method makes the robot turn.
    //With the structure of the code, we can't turn more than 180 degrees in either direction.
    //Our targetAngleDifference is the number of degrees we want to turn.
    //The targetAngleDifference should be positive if turning left, negative if turning right.
    public void Turn(double targetAngleDifference, double power) {

        //Before starting the turn, take note of current angle as startAngle
        double startAngle = GetCurrentZAngle();

        //These are some boolean variables to tell if we've stepped motor power down.
        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        //If our target angle is negative, we're turning right.
        if (targetAngleDifference < 0) {
            //We're turning right, so we want all right motors going backwards.
            for (DcMotor m : RightMotors)
                m.setPower(-power);
            for (DcMotor m : LeftMotors)
                m.setPower(power);
            OpModeReference.sleep(100);

            //We're turning right, so our target angle difference will be negative (ex: -90).
            //So GetAngleDifference will go from 0 to -90.
            //We'll keep turning while difference is greater than target.
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                //This code is for stepping down motor power.
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power / 4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power / 4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(-power / 2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(power / 2);
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
            //If our targetAngleDifference is Positive, we're turning left.
        } else if (targetAngleDifference > 0) {
            //Turning left, so we want all left motors going backwards.
            for (DcMotor m : RightMotors)
                m.setPower(power);
            for (DcMotor m : LeftMotors)
                m.setPower(-power);
            OpModeReference.sleep(100);

            //We're turning right, so our target angle difference will be positive (ex: 90).
            //So GetAngleDifference will go from 0 to 90.
            //Keep turning while difference is less than target.
            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                //This code is for stepping down motor power.
                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power / 4);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power / 4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : RightMotors)
                        m.setPower(power / 2);
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power / 2);
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
            //It's zero -- not turning -- just return.
            return;

        }
    }

    //These next two methods are simply edited copies of the Turn method, changed to use one flank of wheels each.
    //As such, the repetitive comments have been removed, and only relevant ones have been kept.
    public void TurnR(double targetAngleDifference, double power) {

            double startAngle = GetCurrentZAngle();

            boolean firstStepDownComplete = false;
            boolean secondStepDownComplete = false;

            if (targetAngleDifference < 0) {
                for (DcMotor m : RightMotors)
                    m.setPower(-power);
                OpModeReference.sleep(100);
                //You'll notice that only RightMotors are given power here, allowing us to turn just with the right motors.

                while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                    if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                        for (DcMotor m : RightMotors)
                            m.setPower(-power/4);
                        secondStepDownComplete = true;
                    } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                        for (DcMotor m : RightMotors)
                            m.setPower(-power/2);
                        firstStepDownComplete = true;
                    }
                    //Same with the motors down here.

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
            } else if (targetAngleDifference > 0) {
                for (DcMotor m : RightMotors)
                    m.setPower(power);

                OpModeReference.sleep (100);

                while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                    if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                        for (DcMotor m : RightMotors)
                            m.setPower(power/4);
                        secondStepDownComplete = true;
                    } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                        for (DcMotor m : RightMotors)
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
            } else {
                return;
            }

            StopDriving();
        }

        public void TurnL(double targetAngleDifference, double power) {

        double startAngle = GetCurrentZAngle();

        boolean firstStepDownComplete = false;
        boolean secondStepDownComplete = false;

        if (targetAngleDifference < 0) {
            for (DcMotor m : LeftMotors)
                m.setPower(power);
            //Here we only have the LeftMotors, not the RightMotors.
            OpModeReference.sleep(100);

            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) > targetAngleDifference) {

                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
                    for (DcMotor m : LeftMotors)
                        m.setPower(power/2);
                    firstStepDownComplete = true;
                }
                //Same with the motors here.

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

        } else if (targetAngleDifference > 0) {
            for (DcMotor m : LeftMotors)
                m.setPower(-power);
            OpModeReference.sleep (100);

            while (OpModeReference.opModeIsActive() && GetAngleDifference(startAngle) < targetAngleDifference) {

                if (!secondStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.75) {
                    for (DcMotor m : LeftMotors)
                        m.setPower(-power/4);
                    secondStepDownComplete = true;
                } else if (!firstStepDownComplete && GetAngleDifference(startAngle) / targetAngleDifference > 0.50) {
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

            return;
        }

        StopDriving();
    }
}

