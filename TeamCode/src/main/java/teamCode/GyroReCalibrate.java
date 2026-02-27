package teamCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Gyro ReCalibrate", group="Pinpoint")

public class GyroReCalibrate extends LinearOpMode
{
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    @Override
    public void runOpMode()
    {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();
        waitForStart();

        odo.recalibrateIMU();
        sleep(2500);
        telemetry.addLine("Gyro Reset");
        resetRuntime();
    }
}
