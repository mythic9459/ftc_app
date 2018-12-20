package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name="Autonomous Test Driving Beholder", group="Linear Opmode")

public class Autonomous_Test_Driving_Beholder extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor MR = null;
    private DcMotor ML = null;
    private CRServo Intake = null;
    private DcMotor IH = null;

    @Override
    public void runOpMode() {
        Beholder robot = new Beholder(this);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");
        MR = hardwareMap.get(DcMotor.class, "MR");
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        ML = hardwareMap.get(DcMotor.class, "ML");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        IH = hardwareMap.get(DcMotor.class, "IH");




        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        MR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        ML.setDirection(DcMotor.Direction.FORWARD);
        IH.setDirection(DcMotor.Direction.FORWARD);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ML.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double IHpower;
        double Lpower;
        double Rpower;
        double Ipower;


        // call the initialization method
        robot.init();

        waitForStart();
        runtime.reset();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        while (opModeIsActive()) {


            Lpower = -gamepad1.left_stick_y *0.5;
            Rpower = -gamepad1.right_stick_y *0.5;
            Ipower = gamepad2.right_stick_y;

            if (-gamepad2.right_stick_y > 0){
            IHpower = -gamepad2.left_stick_y *0.5;
            }

            else if(-gamepad2.right_stick_y < 0){
                IHpower = -gamepad2.left_stick_y *0.2;
            }
            else {
                IHpower = -gamepad2.left_stick_y *0.5;
            }

            if (gamepad1.dpad_up == true) {
                Lpower = 0.5;
                Rpower = 0.5;
            }
            else if (gamepad1.dpad_down == true) {
                Lpower = -0.5;
                Rpower = -0.5;
            }
            else if (gamepad1.dpad_left == true) {
                robot.Turn(45, 0.5);
            }
            else if (gamepad1.dpad_right == true) {
                robot.Turn(-45, 0.5);
            }



            FR.setPower(Rpower);
            BR.setPower(Rpower);
            MR.setPower(Rpower);
            FL.setPower(Lpower);
            BL.setPower(Lpower);
            ML.setPower(Lpower);
            Intake.setPower(Ipower);

            IH.setPower(IHpower);



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "Left (%.2f), Right (%.2f), Hinge (%.2f", Lpower, Rpower, IHpower);
            telemetry.update();
        }
    }
}
