package teamCode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.dfrobot.HuskyLens;

public class HuskyLensAimingSubsystem extends SubsystemBase
{
    private final HuskyLens m_huskyLens;
    private HuskyLens.Block targetBlock;

    public HuskyLensAimingSubsystem(final HardwareMap hardwareMap)
    {
        // Find the HuskyLens in the hardware map
        m_huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        // Set the HuskyLens algorithm to AprilTag recognition
        try
        {
            m_huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
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
            HuskyLens.Block[] blocks = m_huskyLens.blocks();
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
