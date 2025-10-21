package teamCode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import java.util.List;

public class EyeOfSauronHuskyLensSubsystem extends SubsystemBase
{
    private final HuskyLens huskyLens;
    private HuskyLens.Block targetBlock;

    public EyeOfSauronHuskyLensSubsystem(final HardwareMap hardwareMap)
    {
        // Find the HuskyLens in the hardware map
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Set the HuskyLens algorithm to AprilTag recognition
        try
        {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        }
        catch (Exception e)
        {
            // Handle error (e.g., provide telemetry)
        }
    }

    @Override
    public void periodic()
    {
        // Run this in the loop to continuously get the latest data
        try
        {
            HuskyLens.Block[] blocks = huskyLens.blocks();
            targetBlock = null;

            // Iterate through detected blocks to find a specific AprilTag ID
            for (HuskyLens.Block block : blocks)
            {
                if (block.id == 1)
                { // Change '1' to the AprilTag ID you want to aim at
                    targetBlock = block;
                    break;
                }
            }
        }
        catch (Exception e)
        {
            // Handle error
            targetBlock = null;
        }
    }

    public boolean isTargetDetected()
    {
        return targetBlock != null;
    }

    public double getTargetCenterX()
    {
        return targetBlock != null ? targetBlock.x : -1;
    }

}
