package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "Crater Auto Alpha", group = "IMU1")
public class Crater_Auto_Alpha extends LinearOpMode {

    //Our autonomous programs use a standardized drive speed, so we set that up here.
    static final double DRIVE_SPEED = 0.2;

    @Override
    public void runOpMode() {

        //Now we create an instance of the hardware robot class, and pass an instance of this OpMode.
        //This allows us to access our autonomous source opmode, with our driving and turning methods.
        Beholder robot = new Beholder(this);

        //Now we call the initialization method.
        robot.init();

        // Wait until the start button is clicked!
        waitForStart();

        //Now we start the logging of measured acceleration.
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //Now we drive to the lip of the crater & knock off the center mineral.
        robot.Drive(DRIVE_SPEED, 32);
        sleep(100);

        //Now we back up so as to not hit the other minerals.
        robot.Drive(DRIVE_SPEED, -10);
        sleep(100);

        //Now we turn to be parallel with the markers for the minerals.
        robot.Turn(85, DRIVE_SPEED);
        sleep(100);

        //Now we drive towards our alliance's wall and stop short.
        robot.Drive(DRIVE_SPEED, 46);
        sleep(100);

        //Now we turn to face the alliance depot.
        robot.Turn(43,DRIVE_SPEED);
        sleep(100);

        //Now we drive to the alliance depot.
        robot.Drive(DRIVE_SPEED, 40);
        sleep(100);

        //Now we drop off our game piece.

        //Now we drive backwards across the field and into the crater.
        robot.Drive(DRIVE_SPEED, -95);

        //Now we manually stop, just in case.
        robot.StopDriving();
    }
}
