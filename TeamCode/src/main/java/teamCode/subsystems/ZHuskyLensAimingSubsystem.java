//package teamCode.subsystems;
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.sun.tools.javac.util.List;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//
//public class HuskyLensAimingSubsystem extends SubsystemBase
//{
//    private final HuskyLens huskyLens;
//    public int targetX = -1;
//    public boolean targetDetected = false;
//    private static final int TARGET_ID = 1; // The ID you trained on
//
//    public HuskyLensAimingSubsystem(HardwareMap hardwareMap) {
//        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens"); // Name from config
//        // Set the algorithm on the HuskyLens (e.g., Object Tracking)
//        // This usually needs to be done once in the OpMode init or subsystem init
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
//        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
//                .setCameraPose(cameraPosition, cameraOrientation).build();
//    }
//
//    public void targetDetected()
//    {
////        List<HuskyLens.Block> blocks = huskyLens.blocks();
//        HuskyLens.Block[] blocks1 = huskyLens.blocks();
//        targetDetected = false;
//        targetX = -1;
//
//        for (HuskyLens.Block block : blocks1)
//        {
//            if (block.id == TARGET_ID)
//            {
//                targetX = block.x; // X coordinate of the center of the block
//                targetDetected = true;
//                break;
//            }
//        }
//    }
////    private final HuskyLens m_huskyLens;
////    private HuskyLens.Block targetBlock;
////
////    public HuskyLensAimingSubsystem(final HardwareMap hardwareMap)
////    {
////        // Find the HuskyLens in the hardware map
////        m_huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
////
////        // Set the HuskyLens algorithm to AprilTag recognition
////        try
////        {
//////            m_huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
////            m_huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_TRACKING);
////        }
////        catch (Exception e)
////        {
////            // Handle error (e.g., provide telemetry)
////        }
////    }
////
////    @Override
////    public void periodic()
////    {
////        // Run this in the loop to continuously get the latest data
////        try
////        {
////            HuskyLens.Block[] blocks = m_huskyLens.blocks();
////            targetBlock = null;
////
////            // Iterate through detected blocks to find a specific AprilTag ID
////            for (HuskyLens.Block block : blocks)
////            {
////                if (block.id == 1)
////                { // Change '1' to the AprilTag ID you want to aim at
////                    targetBlock = block;
////                    break;
////                }
////            }
////        }
////        catch (Exception e)
////        {
////            // Handle error
////            targetBlock = null;
////        }
////    }
////
////    public boolean isTargetDetected()
////    {
////        return targetBlock != null;
////    }
////
////    public double getTargetCenterX()
////    {
////        return targetBlock != null ? targetBlock.x : -1;
////    }
//}
