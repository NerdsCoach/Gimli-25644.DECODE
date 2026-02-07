package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HuskyLensColorSubsystem extends SubsystemBase
{
    private final HuskyLens m_huskyLensColor;
    private HuskyLens.Block targetBlock;

    public HuskyLensColorSubsystem(final HardwareMap hardwareMap)
    {
        m_huskyLensColor = hardwareMap.get(HuskyLens.class, "huskylens");

        // Set the HuskyLens algorithm to AprilTag recognition
        try
        {
            m_huskyLensColor.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
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
            HuskyLens.Block[] blocks = m_huskyLensColor.blocks();
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
