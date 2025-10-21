package teamCode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Gyro ReCalibrate", group="Pinpoint")
//@Disabled

public class GyroReCalibrate extends LinearOpMode
{


    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    DriveToPoint nav = new DriveToPoint(); //OpMode member for the point-to-point navigation class

    @Override
    public void runOpMode()
    {

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        odo.setOffsets(68, -178);//these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();
        waitForStart();

        odo.recalibrateIMU();
        sleep(2500);
        telemetry.addLine("Gyro Reset");
        resetRuntime();
    }
}
