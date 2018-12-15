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
    private CRServo Intake = null;
    private DcMotor IH = null;

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
        Intake = hardwareMap.get(CRServo.class, "Intake");
        IH = hardwareMap.get(DcMotor.class, "IH");




        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        MR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        ML.setDirection(DcMotor.Direction.FORWARD);
        IH.setDirection(DcMotor.Direction.FORWARD);
        IH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //IntakeHinge.setDirection(DcMotor.Direction.FORWARD);

        double IHpower;
        double Lpower;
        double Rpower;
        double Ipower;
        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            Lpower = -gamepad1.left_stick_y *0.5;
            Rpower = -gamepad1.right_stick_y *0.5;
            Ipower = gamepad2.right_stick_y;
            IHpower = -gamepad2.left_stick_y *0.5;

            FR.setPower(Rpower);
            BR.setPower(Rpower);
            MR.setPower(Rpower);
            FL.setPower(Lpower);
            BL.setPower(Lpower);
            ML.setPower(Lpower);

            Intake.setPower(Ipower);
            IH.setPower(IHpower);



            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), Hinge (%.2f", Lpower, Rpower, Ipower);
            telemetry.update();
        }
    }
}
