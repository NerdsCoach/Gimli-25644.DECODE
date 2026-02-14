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
import java.util.List;
//
//public class LimeLightSubsystem extends SubsystemBase
//{
//    private final Limelight3A limelight;
//
//    public LimeLightSubsystem(HardwareMap hardwareMap)
//    {
//        // Initialize the Limelight using the hardware map
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//
//        // Apply your specific configurations
//        limelight.setPollRateHz(100);
//        limelight.start();
//    }
//
//    {
//    private final Limelight3A m_limelight;
//    private final int targetId;
//    private LLResult latestResult;
//    private LLResultTypes.FiducialResult targetFiducial = null;
//
//    public LimeLightSubsystem(Limelight3A limelight, int targetId)
//    {
//        this.m_limelight = limelight;
//        this.targetId = targetId;
//
//        m_limelight.setPollRateHz(100);
//        m_limelight.pipelineSwitch(0); // Ensure pipeline 0 is set to AprilTags
//        m_limelight.start();
//    }
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
//
//    public boolean isTagDetected()
//    {
//        return targetFiducial != null;
//    }
//
//    /**
//     * @return Horizontal offset in degrees for aiming
//     */
//    public double getHorizontalOffset()
//    {
//        return isTagDetected() ? targetFiducial.getTargetXDegrees() : 0.0;
//    }
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
////public class LimeLightSubsystem extends SubsystemBase
////{
//////    private final HuskyLens m_huskyLens;
//////    private final int targetId;//added1/27
//////
//////    Limelight3A m_limelight;
//////
//////
//////    private HuskyLens.Block detectedTag = null; // Store the most relevant detected tag
//////
//////    public LimeLightSubsystem(HuskyLens huskyLens, int targetId)
//////    {
//////        m_huskyLens = huskyLens;
//////        this.targetId = targetId;//added1/27
//////        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//////    }
//////    // This runs automatically every loop to keep data fresh
//////
//////
//////    public void init()
//////    {
//////        m_limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
//////        m_limelight.start(); // This tells Limelight to start looking!
//////        m_limelight.pipelineSwitch(0); // Switch to pipeline number 0
//////    }
//////
//////
//////    public boolean isTagDetected()
//////    {
//////        LLResult result = m_limelight.getLatestResult();
//////        if (result != null && result.isValid()) {
//////            double tx = result.getTx(); // How far left or right the target is (degrees)
//////            double ty = result.getTy(); // How far up or down the target is (degrees)
//////            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
//////
//////            telemetry.addData("Target X", tx);
//////            telemetry.addData("Target Y", ty);
//////            telemetry.addData("Target Area", ta);
//////        } else
//////        {
//////            telemetry.addData("Limelight", "No Targets");
//////        }
//////
//////
//////// april tag tracking
//////        if (result != null && result.isValid())
//////        {
//////            Pose3D botpose = result.getBotpose();
//////            if (botpose != null) {
//////                double x = botpose.getPosition().x;
//////                double y = botpose.getPosition().y;
//////                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
//////            }
//////        }
//////
//////        //april tag and angle tracking
//////        // First, tell Limelight which way your robot is facing
//////        double robotYaw = imu.getAngularOrientation().firstAngle;
//////        m_limelight.updateRobotOrientation(robotYaw);
//////        if (result != null && result.isValid())
//////        {
//////            Pose3D botpose_mt2 = result.getBotpose_MT2();
//////            if (botpose_mt2 != null)
//////            {
//////                double x = botpose_mt2.getPosition().x;
//////                double y = botpose_mt2.getPosition().y;
//////                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
//////            }
//////        }
//////
//////        // April tag and field  positon tracking
//////        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//////        for (LLResultTypes.FiducialResult fiducial : fiducials) {
//////            int id = fiducial.getFiducialId(); // The ID number of the fiducial
//////            double x = detection.getTargetXDegrees(); // Where it is (left-right)
//////            double y = detection.getTargetYDegrees(); // Where it is (up-down)
//////            double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getY();
//////            telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
//////        }
//////
//////        fiducial.getRobotPoseTargetSpace(); // Robot pose relative it the AprilTag Coordinate System (Most Useful)
//////        fiducial.getCameraPoseTargetSpace(); // Camera pose relative to the AprilTag (useful)
//////        fiducial.getRobotPoseFieldSpace(); // Robot pose in the field coordinate system based on this tag alone (useful)
//////        fiducial.getTargetPoseCameraSpace(); // AprilTag pose in the camera's coordinate system (not very useful)
//////        fiducial.getTargetPoseRobotSpace(); // AprilTag pose in the robot's coordinate system (not very useful)
//////
//////
//////
//////
//////        return true;
//////
//////    }
////
////
////}
