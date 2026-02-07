package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

public class HuskyLensSubsystem extends SubsystemBase
{
    private final HuskyLens m_huskyLens;
    private final int targetId;//added1/27

    private HuskyLens.Block detectedTag = null; // Store the most relevant detected tag

    public HuskyLensSubsystem(HuskyLens huskyLens, int targetId)
    {
        m_huskyLens = huskyLens;
        this.targetId = targetId;//added1/27
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }
    // This runs automatically every loop to keep data fresh
    @Override
    public void periodic()
    {
        HuskyLens.Block[] blocks = m_huskyLens.blocks();
        detectedTag = null; // Reset every loop

        for (HuskyLens.Block block : blocks)
        {
                if (block.id == targetId)//changed1.27

                {
                detectedTag = block; // Found our specific tag!
                break;
            }
        }
    }

    public boolean isTagDetected()
    {
        return detectedTag != null;
    }

    public double getTargetWidth()
    {
        return isTagDetected() ? (double)detectedTag.width : -1.0;
    }

    public int getTargetCenterX()
    {
        return isTagDetected() ? detectedTag.x : 160; // 160 is the middle of the screen
    }

}
