package teamCode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp(name = "LimeLight Readings", group = "Sensor")
@Disabled
public class LimeLightReadings extends LinearOpMode
{

    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        waitForStart();

        while (opModeIsActive())
        {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
            {
                LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
                Pose3D pose = tag.getTargetPoseCameraSpace();

                double x = pose.getPosition().x;
                double y = pose.getPosition().y;
                double z = pose.getPosition().z;

                telemetry.addData("Tag ID", tag.getFiducialId());
                telemetry.addData("Distance (Z)", "%.2f meters", z);
                telemetry.addData("Side Offset (X)", "%.2f", x);
            }
            else
            {
                telemetry.addData("Limelight", "Searching for Tag...");
            }

            // CRITICAL: You must update telemetry every loop
            telemetry.update();
        }

        // CRITICAL: Only stop the limelight AFTER the loop finishes
        limelight.stop();
    }
}