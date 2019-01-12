package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Driver Boi Current", group="Linear Opmode")

public class DriverBoi_Current extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor MR = null;
    private DcMotor ML = null;
    private CRServo Intake1 = null;
    private CRServo Intake2 = null;
    private DcMotor IH = null;
    //Here we start setting up our motors.

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        MR = hardwareMap.get(DcMotor.class, "MR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        ML = hardwareMap.get(DcMotor.class, "ML");
        Intake1 = hardwareMap.get(CRServo.class, "Intake1");
        Intake2 = hardwareMap.get(CRServo.class, "Intake2");
        IH = hardwareMap.get(DcMotor.class, "IH");
        //Here we finish setting up all our motors.

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        MR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        ML.setDirection(DcMotor.Direction.FORWARD);
        IH.setDirection(DcMotor.Direction.FORWARD);
        //Here we set the right set of motors to reverse to let us not spin when we drive.
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Here we tell all motors to brake when they're not recieving commands.

        double IHpower;
        double Lpower;
        double Rpower;
        double Ipower;
        IHpower = 0;
        Lpower = 0;
        Rpower = 0;
        Ipower = 0;
        //Here we set up our variables to let us set the motor power.

        boolean TurboSpeed;
        TurboSpeed = true;
        //Now we create a variable to toggle our motors from half power to full power.

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            if (gamepad1.right_bumper && TurboSpeed) {
                TurboSpeed = false;
            }
            if (gamepad1.right_bumper && !TurboSpeed) {
                TurboSpeed = true;
            }
            //This allows us to press a button and toggle TurboSpeed.

            if (TurboSpeed) {
                Lpower = -gamepad1.left_stick_y * 0.5;
                Rpower = -gamepad1.right_stick_y * 0.5;
                Ipower = gamepad2.right_stick_y;
            }
            else if (!TurboSpeed) {
                Lpower = -gamepad1.left_stick_y;
                Rpower = -gamepad1.right_stick_y;
                Ipower = gamepad2.right_stick_y;
            }
            else{
                Lpower = -gamepad1.left_stick_y * 0.5;
                Rpower = -gamepad1.right_stick_y * 0.5;
                Ipower = gamepad2.right_stick_y;
            }
            //Here we take the inputs from the joysticks and put them into variables.
            //The first one allows us to use our the full speed of our motors.

            //The following section of code is designed to slow the hinge on our intake on its descent, preventing it from slamming into the ground
            if (-gamepad2.right_stick_y > 0) {
                IHpower = -gamepad2.left_stick_y * 0.5;
            }
            //This is for the ascent, in which we keep the power at its normal level
            else if (-gamepad2.right_stick_y < 0) {
                IHpower = -gamepad2.left_stick_y * 0.2;
            }
            //This is for the descent, so we decrease our variable, thus decreasing the motor's power.
            else {
                IHpower = -gamepad2.left_stick_y * 0.5;
            }
            //The code got angry without this last bit, as it didn't account for -gamepad2.right_stick_y = 0.

            if (gamepad1.right_trigger > 0.5) {
                Lpower = 0.5;
                Rpower = 0.5;
            } else if (gamepad1.left_trigger > 0.5) {
                Lpower = -0.5;
                Rpower = -0.5;
            }
            //These bits allow us to drive straight forwards & backward.

            FR.setPower(Rpower);
            BR.setPower(Rpower);
            MR.setPower(Rpower);
            FL.setPower(Lpower);
            BL.setPower(Lpower);
            ML.setPower(Lpower);
            //Here we set the power to our drive motors.
            Intake1.setPower(Ipower);
            Intake2.setPower(-Ipower);
            //Here we set the power to our intake servo motors.
            IH.setPower(IHpower);
            //Here we set the power to our intake's hinge.

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Left (%.2f), Right (%.2f), Hinge (%.2f", Lpower, Rpower, IHpower);
            telemetry.update();
            //Here we use some simple telemetry to display the running time and motor power levels.
        }
    }
}
