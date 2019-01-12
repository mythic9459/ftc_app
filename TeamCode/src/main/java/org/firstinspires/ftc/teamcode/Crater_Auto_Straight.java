package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "Crater Auto Straight", group = "IMU1")
public class Crater_Auto_Straight extends LinearOpMode {

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

        robot.Drive(DRIVE_SPEED, 50);
        sleep(100);

        //Now we manually stop, just in case.
        robot.StopDriving();
    }
}
