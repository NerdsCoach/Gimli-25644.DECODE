package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.LimeLightSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class AimingOnCommand extends CommandBase
{
    private final LimeLightSubsystem m_limelightSubsystem;
    private final TurnTableSubsystem m_turnTableSubsystem;
    private final LightSubsystem m_lightSubsystem;
    private Telemetry m_telemetry;

    private static final double deadband = 1.5;
    private static final double minPower = 0.04; //0.08
    private static final double kP = 0.025;
    private static final double kD = 0.001;
    private final int m_targetId;
    private double lastError = 0;

    public AimingOnCommand(LimeLightSubsystem limeLightSubsystem, TurnTableSubsystem turnTableSubsystem, LightSubsystem lightSubsystem, int targetId, Telemetry telemetry)
    {
        m_limelightSubsystem = limeLightSubsystem;
        m_telemetry = telemetry;
        m_turnTableSubsystem = turnTableSubsystem;
        m_lightSubsystem = lightSubsystem;
        m_targetId = targetId;
        addRequirements(turnTableSubsystem, lightSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        LLResult result = m_limelightSubsystem.getLatestResult();
        int currentPosition = m_turnTableSubsystem.getCurrentPosition();

        // 1. HANDLE TAG LOST (Search/Recovery Mode)
        if (!m_limelightSubsystem.isTagDetected(m_targetId))
        {
            m_lightSubsystem.off(0.0);
            m_turnTableSubsystem.stop();
            return;
        }

        // 2. CALCULATE CORRECTION
        m_lightSubsystem.on(0.28);
        double error = m_limelightSubsystem.getHorizontalOffsetForTag(m_targetId);

        // Deadband check
        if (Math.abs(error) < deadband)
        {
            m_turnTableSubsystem.stop();
            lastError = error;
            return;
        }

        // PD Control
        double correction = (error * kP) + ((error - lastError) * kD);
        lastError = error;

        // Smart Power Floor & Clamping
        if (Math.abs(correction) < minPower)
        {
            correction = (error > 0) ? minPower : -minPower;
        }
        correction = Math.max(-0.4, Math.min(0.4, correction));

        // 3. DIRECTIONAL SAFETY LIMITS
        // We only stop if we are AT a limit AND trying to move FURTHER past it.
        // If correction is negative, we are moving toward negative numbers (left).
        // If correction is positive, we are moving toward positive numbers (right).

        boolean atUpperLimit = (currentPosition >= 835);
        boolean atLowerLimit = (currentPosition <= -835);

        if (atUpperLimit && correction > 0)
        {
            // Block movement further right
            m_turnTableSubsystem.stop();
        }
        else if (atLowerLimit && correction < 0)
        {
            // Block movement further left
            m_turnTableSubsystem.stop();
        }
        else
        {
            // Allow movement (either within bounds, or moving back toward 0)
            m_turnTableSubsystem.turnSpeed(correction);
        }

        // Telemetry for debugging
        m_telemetry.addData("Turntable Pos", currentPosition);
        m_telemetry.addData("Correction", correction);
        m_telemetry.addData("Tag Error", error);
        lastError = error;
    }
    @Override
    public boolean isFinished()
    {
        return false; // Continue indefinitely until centered or interrupted
    }

    @Override
    public void end(boolean interrupted)
    {
        m_turnTableSubsystem.stop();
        m_lightSubsystem.off(0.0);
    }
//    private final LimeLightSubsystem m_limelightSubsystem;
//    private final TurnTableSubsystem m_turnTableSubsystem;
//    private final LightSubsystem m_lightSubsystem;
//    private Telemetry m_telemetry;
//
//    private static final double deadband = 1.5;
//    private static final double minPower = 0.04;//0.08
//    private static final double kP = 0.025;
//    private static final double kD = 0.001;
//    private final int m_targetId;
//    private double lastError = 0;
//
//    public AimingOnCommand(LimeLightSubsystem limeLightSubsystem, TurnTableSubsystem turnTableSubsystem, LightSubsystem lightSubsystem, int targetId, Telemetry telemetry)
//    {
//        m_limelightSubsystem = limeLightSubsystem;
//        m_telemetry = telemetry;
//        m_turnTableSubsystem = turnTableSubsystem;
//        m_lightSubsystem = lightSubsystem;
//        m_targetId = targetId;
//        addRequirements(turnTableSubsystem, lightSubsystem);
//    }
//
//    @Override
//    public void initialize()
//    {
//    }
//
//    @Override
//    public void execute()
//    {
//        LLResult result = m_limelightSubsystem.getLatestResult();
//        int currentPosition = m_turnTableSubsystem.getCurrentPosition();
//
//        // 1. HANDLE TAG LOST (Search/Recovery Mode)
//        if (!m_limelightSubsystem.isTagDetected(m_targetId))
//        {
//            m_lightSubsystem.off(0.0);
//            m_turnTableSubsystem.stop();
//            return;
//        }
//
//        // 2. CALCULATE CORRECTION
//        m_lightSubsystem.on(0.28);
////        double error = m_limelightSubsystem.getHorizontalOffsetForTag(m_targetId);
//
//
//// 2. CALCULATE CORRECTION
//        m_lightSubsystem.on(0.28);
//
//        double error = 0;
//        boolean foundTarget = false;
//
//// We loop through the detected tags to find the one matching your m_targetId
//        for (LLResultTypes.FiducialResult fr : result.getFiducialResults())
//        {
//
//            if (fr.getFiducialId() == m_targetId)
//            {
//                // Robot's position relative to the AprilTag center
//                // x is sideways, z is forward distance
//                Pose3D pose = fr.getRobotPoseTargetSpace();
//                double x = pose.getPosition().x;
//                double z = pose.getPosition().z;
//
//                // GOAL OFFSET: Distance (meters) from the tag center to the goal center
//                // If the goal is 10 inches behind the tag, use 0.254
//                double goalDepth = 0.0;//0.3
//
//                // We use -x because if the robot is to the right (+x),
//                // it needs a negative angle to turn left toward the center.
//                error = Math.toDegrees(Math.atan2(-x, Math.abs(z) + goalDepth));
//
//                foundTarget = true;
//                break;
//            }
//        }
//
//        if (!foundTarget)
//        {
//            m_turnTableSubsystem.stop();
//            return;
//        }
//
//        // Deadband check
//        if (Math.abs(error) < deadband)
//        {
//            m_turnTableSubsystem.stop();
////            lastError = error;
//            return;
//        }
//
//        // PD Control
//
//        double correction = (error * kP) + ((error - lastError) * kD);
//        lastError = error;
//
//        // Smart Power Floor & Clamping
//        if (Math.abs(error) > deadband && Math.abs(correction) < minPower) {
//            correction = Math.copySign(minPower, error);
//        }
////        if (Math.abs(correction) < minPower)
////        {
////            correction = (error > 0) ? minPower : -minPower;
////        }
//        correction = Math.max(-0.4, Math.min(0.4, correction));
//
//        // 3. DIRECTIONAL SAFETY LIMITS
//        // We only stop if we are AT a limit AND trying to move FURTHER past it.
//        // If correction is negative, we are moving toward negative numbers (left).
//        // If correction is positive, we are moving toward positive numbers (right).
//
//        boolean atUpperLimit = (currentPosition >= 835);
//        boolean atLowerLimit = (currentPosition <= -835);
//
//        if (atUpperLimit && correction > 0)
//        {
//            // Block movement further right
//            m_turnTableSubsystem.stop();
//        }
//        else if (atLowerLimit && correction < 0)
//        {
//            // Block movement further left
//            m_turnTableSubsystem.stop();
//        }
//        else
//        {
//            // Allow movement (either within bounds, or moving back toward 0)
//            m_turnTableSubsystem.turnSpeed(correction);
//        }
//
//        // Telemetry for debugging
//        m_telemetry.addData("Turntable Pos", currentPosition);
//        m_telemetry.addData("Correction", correction);
//        m_telemetry.addData("Tag Error", error);
//        lastError = error;
//    }
//
//
//    @Override
//    public boolean isFinished()
//    {
//        return false; // Continue indefinitely until centered or interrupted
//    }
//
//    @Override
//    public void end(boolean interrupted)
//    {
//        m_turnTableSubsystem.stop();
//        m_lightSubsystem.off(0.0);
//    }

}
