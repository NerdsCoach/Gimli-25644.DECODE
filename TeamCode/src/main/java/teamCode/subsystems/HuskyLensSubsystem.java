package teamCode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class HuskyLensSubsystem extends SubsystemBase
{
    private final HuskyLens m_huskyLens;
    private HuskyLens.Block detectedTag = null; // Store the most relevant detected tag


    public HuskyLensSubsystem(HuskyLens huskyLens)
    {
        m_huskyLens = huskyLens;
        // Configure as per your robot configuration name ("m_huskyLens")
//        huskyLens = hardwareMap.get(HuskyLens.class, "m_huskyLens");
        // Set the algorithm to TAG_RECOGNITION programmatically
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    @Override
    public void periodic() {
        // Periodically check for AprilTags
        HuskyLens.Block[] blocks = m_huskyLens.blocks();
        if (blocks.length > 0) {
            // Find the tag you want to aim for (e.g., ID 1)
            for (HuskyLens.Block block : blocks) {
                // The HuskyLens returns ID 0 for untrained tags
                // You will need to "learn" specific tags using the HuskyLens UI first
                if (block.id == 4) { // Assuming ID 1 is your target
                    detectedTag = block;
                    break;
                }
            }
        } else {
            detectedTag = null;
        }
    }

    public boolean isTagDetected() {
        return detectedTag != null;
    }

    public int getTargetCenterX()
    {
        return detectedTag != null ? detectedTag.x : 0; // X position of the target
    }




//    private final HuskyLens m_huskyLens;
//    private HuskyLens.Block detectedTag = null; // Store the most relevant detected tag
//    Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
//        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -70, 0, 0);
//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
//                .setCameraPose(cameraPosition, cameraOrientation).build();
//
//    public HuskyLensSubsystem(HuskyLens huskyLens)
//    {
//        m_huskyLens = huskyLens;
//        // Configure as per your robot configuration name ("m_huskyLens")
////        huskyLens = hardwareMap.get(HuskyLens.class, "m_huskyLens");
//        // Set the algorithm to TAG_RECOGNITION programmatically
////        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//    }
//
//    @Override
//    public void periodic()
//    {
//        // Periodically check for AprilTags
//        HuskyLens.Block[] blocks = m_huskyLens.blocks();
//        if (blocks.length > 0)
//        {
//            // Find the tag you want to aim for (e.g., ID 1)
//            for (HuskyLens.Block block : blocks)
//            {
//                // The HuskyLens returns ID 0 for untrained tags
//                // You will need to "learn" specific tags using the HuskyLens UI first
//                if (block.id == 1)
//                { // Assuming ID 1 is your target
//                    detectedTag = block;
//                    break;
//                }
//            }
//        }
//        else
//        {
//            detectedTag = null;
//        }
//    }
//
//    public boolean isTagDetected()
//    {
//        return detectedTag != null;
//    }
//
//    public int getTargetCenterX()
//    {
//        return detectedTag != null ? detectedTag.x : 0; // X position of the target
//    }


//    private final DcMotor m_turnTableMotor;
//    private final HuskyLens m_huskyLens;
//
//    private final double Kp = 0.005;
//    private final int CENTER_X = 160;
//    public int targetX = -1;
//    public boolean targetDetected = false;
//    private HuskyLens.Block targetBlock;
//
////    private HuskyLens.Block targetBlock;
//
//    public HuskyLensSubsystem(DcMotor turnTableMotor, HuskyLens m_huskyLens)
//    {
//       targetX = -1;
//       targetDetected = false;
//
//       m_turnTableMotor = turnTableMotor;
//       m_huskyLens = m_huskyLens;
//
////       m_huskyLens = hwMap.get(HuskyLens.class, "m_huskyLens");
////       m_turnTableMotor = hwMap.get(DcMotor.class, "turnTableMotor");
//
//        m_huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
//        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
//        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
//                .setCameraPose(cameraPosition, cameraOrientation).build();
//    }
////    public HuskyLensSubsystem(DcMotor turnTableMotor, HuskyLens m_huskyLens)
////    {
////        this.m_turnTableMotor = turnTableMotor;
////        this.m_turnTableMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
////
////        this.m_huskyLens = m_huskyLens;
////
////        m_huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
////        Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
////        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
////        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
////                .setCameraPose(cameraPosition, cameraOrientation).build();
////    }
//
//    public void AimAtTag (int targetId)
//    {
//        HuskyLens.Block[] blocks = m_huskyLens.blocks();
//        for (HuskyLens.Block block : blocks)
//        {
//            if(block.id == targetId)
//            {
////                double error = 160 - block.x;
////                m_turnTableMotor.setPower(error * Kp);
//                m_turnTableMotor.setPower(0.1);
//                targetX = block.x; // X coordinate of the center of the block
//                return;
//            }
//        }
//    }
//    public void Stop ()
//    {
//        m_turnTableMotor.setPower(0.0);
//
//    }
//    public boolean isTargetDetected()
//    {
//        return targetBlock != null;
////        return targetBlock = true;
//    }
}
