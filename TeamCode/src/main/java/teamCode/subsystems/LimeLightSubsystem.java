package teamCode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class LimeLightSubsystem extends SubsystemBase {
    private final Limelight3A m_limelight;

    public LimeLightSubsystem(HardwareMap hardwareMap, int targetId) {
        m_limelight = hardwareMap.get(Limelight3A.class, "limelight");
        m_limelight.pipelineSwitch(0);
        m_limelight.setPollRateHz(100);
        m_limelight.start();
    }

    public double getHorizontalOffsetForTag(int targetId)
    {
        LLResult result = m_limelight.getLatestResult();
        if (result != null && result.isValid())
        {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults())
            {
                if (fr.getFiducialId() == targetId)
                {
                    return fr.getTargetXDegrees(); // This is the 'tx' for THIS specific tag
                }
            }
        }
        return 0;
    }

    /**
     * Checks if a specific AprilTag is currently visible.
     */
    public boolean isTagDetected(int targetId)
    {
        LLResult result = m_limelight.getLatestResult();
        if (result != null && result.isValid())
        {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults())
            {
                if (fr.getFiducialId() == targetId) return true;
            }
        }
        return false;
    }
    public LLResult getLatestResult()
    {
        return m_limelight.getLatestResult();
    }
}
