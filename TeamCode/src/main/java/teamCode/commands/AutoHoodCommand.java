package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import teamCode.Constants;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.LimeLightSubsystem;

public class AutoHoodCommand extends CommandBase
{
    private final HoodServoSubsystem m_hoodServoSubsystem;
    private final LimeLightSubsystem m_limelightSubsystem;
    private double m_lastKnownHoodPosition;

    public AutoHoodCommand(HoodServoSubsystem hoodServoSubsystem, LimeLightSubsystem limeLightSubsystem)
    {
        this.m_hoodServoSubsystem = hoodServoSubsystem;
        this.m_limelightSubsystem = limeLightSubsystem;
        m_lastKnownHoodPosition = 1.0;
        addRequirements(this.m_hoodServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }
    @Override
    public void execute()
    {
        LLResult result = m_limelightSubsystem.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
        {
            LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
            Pose3D pose = tag.getTargetPoseCameraSpace();

            if (pose != null)
            {
                double zMeters = Math.abs(pose.getPosition().z);
                double safeDistance = Math.max(0.49, Math.min(zMeters, 2.42));

                // 2. Calculate and set the Hood Position using your custom math!
                double targetHoodPosition = m_hoodServoSubsystem.getTargetFromDistance(safeDistance);
                m_hoodServoSubsystem.pivotHood(targetHoodPosition);
                m_lastKnownHoodPosition = targetHoodPosition; // SAVE IT TO MEMORY HERE!
            }
        }
        else
        {
            m_hoodServoSubsystem.pivotHood(m_lastKnownHoodPosition);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
