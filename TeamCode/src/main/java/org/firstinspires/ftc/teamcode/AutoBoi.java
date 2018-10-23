package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Auto Boi", group="Linear Opmode")

public class AutoBoi extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor MR = null;
    private DcMotor ML = null;

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

        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        MR.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.FORWARD);
        ML.setDirection(DcMotor.Direction.FORWARD);

        double Lpower;
        double Rpower;

        Lpower = .50;
        Rpower = .50;

        FR.setPower(Rpower);
        BR.setPower(Rpower);
        MR.setPower(Rpower);
        FL.setPower(Lpower);
        BL.setPower(Lpower);
        ML.setPower(Lpower);




        }
    }

