package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Driver BoiXD", group="Linear Opmode")

public class Advertisement extends LinearOpMode{

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Beholder robot = new Beholder(this);

        robot.init();

        double Lpower;
        double Rpower;

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


             Lpower  = -gamepad1.left_stick_y *0.5 ;
             Rpower = -gamepad1.right_stick_y *0.5 ;







            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", Lpower, Rpower);
            telemetry.update();
        }
    }
}
