package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * This is an OpMode that uses a hardware robot class
 */
@Autonomous(name = "Crater Auto", group = "IMU1")
public class Crater_Auto extends LinearOpMode {

    // this is the motor power so when you make changes you can just make here
    // feel free to define multiple like FULL_POWER, HALF_POWER, etc.
    static final double DRIVE_SPEED = 0.3;

    @Override
    public void runOpMode() {

        // -------------------------------------------------------------------------------
        // create an instance of the hardware robot class, pass an instance of THIS OpMode
        Beholder robot = new Beholder(this);

        // call the initialization method
        robot.init();

        // -------------------------------------------------------------------------------
        // Wait until the start button is clicked!
        waitForStart();

        // -------------------------------------------------------------------------------
        // Start the logging of measured acceleration
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        // -------------------------------------------------------------------------------
        // now do all of your driving and claiming depots and getting off lander or whatever
        // sleeps are not required
        // -------------------------------------------------------------------------------
        // HaHa You Cant do that
        // drive forward about 24 inches
        robot.Drive(DRIVE_SPEED, 8);
        sleep(1000);

        robot.Turn(45, DRIVE_SPEED);
        sleep(1000);

        robot.Drive(DRIVE_SPEED, 40);
        sleep(1000);

        robot.Turn(50, DRIVE_SPEED);
        sleep(1000);

        robot.Drive(DRIVE_SPEED, 69);
        sleep(1000);

        robot.Turn(60, DRIVE_SPEED);
        sleep(1000);

        robot.Turn(83, DRIVE_SPEED);
        sleep(1000);


        robot.Drive(DRIVE_SPEED, 80);
        sleep(1000);

        robot.Turn(30, DRIVE_SPEED);
        sleep(1000);

        robot.Drive(DRIVE_SPEED, 80);
        sleep(1000);

        // turn LEFT 90 degrees
        /*robot.Turn(90, DRIVE_SPEED);
        sleep(1000);*/
        // turn RIGHT 90 degrees
        /*robot.Turn(-90, DRIVE_SPEED);
        sleep(1000);*/



        robot.StopDriving();
    }
}
