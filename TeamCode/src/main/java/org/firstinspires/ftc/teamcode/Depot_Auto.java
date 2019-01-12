package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "Depot Auto", group = "IMU1")
public class Depot_Auto extends LinearOpMode {

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

        //This drives us to the alliance depot.
        robot.Drive(DRIVE_SPEED, 60);
        sleep(100);

        //Now we drop off our game piece.
        robot.Intake1.setPower(1);
        robot.Intake2.setPower(-1);
        sleep(1000);
        robot.Intake1.setPower(0);
        robot.Intake2.setPower(0);

        //Now we turn to face the back of our robot to the crater.
        robot.Turn(43, DRIVE_SPEED);
        sleep(100);

        //Now we drive across the field and into the crater.
        robot.Drive(DRIVE_SPEED, -85);

        //Now we manually stop, just in case.
        robot.StopDriving();
    }
}
