package teamCode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

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
//}
//
//    @Override
//    public void periodic()
//    {
//        latestResult = m_limelight.getLatestResult();
//        targetFiducial = null; // Reset every loop
//
//        if (latestResult != null && latestResult.isValid())
//        {
//            List<LLResultTypes.FiducialResult> fiducials = latestResult.getFiducialResults();
//            for (LLResultTypes.FiducialResult fiducial : fiducials)
//            {
//                if (fiducial.getFiducialId() == targetId)
//                {
//                    targetFiducial = fiducial;
//                    break;
//                }
//            }
//        }
//    }
////    public LLResult getLatestResult()
////    {
////        return m_limelight.getLatestResult();
////    }
//
//    public double getHorizontalOffset()
//    {
//        LLResult result = m_limelight.getLatestResult();
//        if (result != null && result.isValid())
//        {
//            return result.getTx(); // Degrees from crosshair (-30 to 30)
//        }
//        return 0; // Return 0 if no tag is seen
//    }
//
//    public boolean hasTarget()
//    {
//        LLResult result = m_limelight.getLatestResult();
//        return result != null && result.isValid();
//    }
//
//    public boolean isTagDetected()
//    {
//        return targetFiducial != null;
//    }
//
//    public LLResult getResultForTag(int targetId) {
//        LLResult result = m_limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            // Get all visible AprilTags
//            List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
//
//            for (LLResultTypes.DetectorResult detection : detections) {
//                if (detection.getTargetId() == targetId) {
//                    return result; // Found the specific tag we want
//                }
//            }
//        }
//        return null;
//    }
//    /**
//     * @return Horizontal offset in degrees for aiming
//     */
////    public double getHorizontalOffset()
////    {
////        return isTagDetected() ? targetFiducial.getTargetXDegrees() : 0.0;
////    }
//
//    /**
//     * @return Robot's strafe distance relative to the tag in meters
//     */
//    public double getStrafeOffset()
//    {
//        return isTagDetected() ? targetFiducial.getRobotPoseTargetSpace().getPosition().y : 0.0;
//    }
//}
//
//
//
