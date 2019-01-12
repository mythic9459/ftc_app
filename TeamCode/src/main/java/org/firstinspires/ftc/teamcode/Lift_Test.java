package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Disabled
@Autonomous(name = "Lift Test", group = "IMU1")
public class Lift_Test extends LinearOpMode {

    //Our autonomous programs use a standardized drive speed, so we set that up here.
    static final double DRIVE_SPEED = 0.2;

    @Override
    public void runOpMode() {

        //Now we create an instance of the hardware robot class, and pass an instance of this OpMode.
        //This allows us to access our autonomous source opmode, with our driving and turning methods.
        Beholder robot = new Beholder(this);

        //Now we call the initialization method.
        robot.init();

        //Wait until the start button is clicked!
        waitForStart();

        //Now we start the logging of measured acceleration.
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Lower lift until touches ground
        robot.Intake1.setPower(1);
        robot.Intake2.setPower(-1);
        sleep(1000);
        robot.Intake1.setPower(0);
        robot.Intake2.setPower(0);

        //Turn slightly right to free hook
        robot.Turn(-15, DRIVE_SPEED);
        sleep(100);

        //Retract Lift
        robot.Intake1.setPower(1);
        robot.Intake2.setPower(-1);
        sleep(1000);
        robot.Intake1.setPower(0);
        robot.Intake2.setPower(0);

        //Turn right to align
        robot.Turn(15, DRIVE_SPEED);
        sleep(100);

        //Now we manually stop, just in case.
        robot.StopDriving();
    }
}
