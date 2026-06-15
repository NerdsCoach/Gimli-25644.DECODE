package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LimeLightSubsystem;

public class AutoLauncherCommand extends CommandBase
{
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;
    private final HoodServoSubsystem m_hoodSubsystem;
    private final LimeLightSubsystem m_limelightSubsystem;

    private double m_lastKnownSpeed;
    private final int m_targetId;
    private double m_smoothedDistance = 0.0;

    private final InterpLUT m_velocityLUT = new InterpLUT();

    public AutoLauncherCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem,
                               HoodServoSubsystem hoodSubsystem, LimeLightSubsystem limelightSubsystem,
                               int targetId)
    {
        this.m_launcherSubsystem = launcherSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        this.m_hoodSubsystem = hoodSubsystem;
        this.m_limelightSubsystem = limelightSubsystem;
        this.m_targetId = targetId;

        m_lastKnownSpeed = 1500.0;

        addRequirements(this.m_launcherSubsystem, this.m_axeSubsystem, this.m_hoodSubsystem);

//        m_velocityLUT.add(0.49, 1450.0);
//        m_velocityLUT.add(0.65, 1550.0);
//        m_velocityLUT.add(0.78, 1600.0);
//        m_velocityLUT.add(0.85, 1650.0);
//        m_velocityLUT.add(0.99, 1750.0);
//        m_velocityLUT.add(1.12, 1850.0);
//        m_velocityLUT.add(1.28, 1950.0);
//        m_velocityLUT.add(1.36, 2050.0);
//        m_velocityLUT.add(1.61, 2150.0);
//        m_velocityLUT.add(1.93, 2250.0);//Hood up
//        m_velocityLUT.add(1.95, 2260.0);
//        m_velocityLUT.add(2.14, 2300.0);
//        m_velocityLUT.add(2.22, 2350.0);
//        m_velocityLUT.add(2.38, 2450.0);
//        m_velocityLUT.add(2.42, 2550.0);
       // added 40
        m_velocityLUT.add(0.20, 1490.0);
        m_velocityLUT.add(0.53, 1690.0);
        m_velocityLUT.add(0.80, 1790.0);
        m_velocityLUT.add(0.99, 1860.0);
        m_velocityLUT.add(1.07, 1950.0);
        m_velocityLUT.add(1.20, 2040.0);
        m_velocityLUT.add(1.35, 2140.0);
        m_velocityLUT.add(1.49, 2240.0);
        m_velocityLUT.add(1.79, 2300.0);
        m_velocityLUT.add(2.05, 2400.0);
        m_velocityLUT.add(2.36, 2520.0);
        m_velocityLUT.add(2.75, 2710.0);
        // Finalize the LUT
        m_velocityLUT.createLUT();
    }

        @Override
        public void execute()
        {
            LLResult result = m_limelightSubsystem.getLatestResult();

            boolean goalFound = false;

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
            {
                for (LLResultTypes.FiducialResult tag : result.getFiducialResults())
                {
                    if (tag.getFiducialId() == m_targetId) // Safely isolating Tag 24
                    {
                        Pose3D pose = tag.getTargetPoseCameraSpace();
                        if (pose != null)
                        {
                            double rawZ = Math.abs(pose.getPosition().z);
                            if (m_smoothedDistance == 0.0)
                            {
                                m_smoothedDistance = rawZ;
                            }

                                // Constrain within your safe LUT boundaries
                                double safeDistance = Math.max(0.20, Math.min(m_smoothedDistance, 2.75));

                                // Set Flywheel Speed
                                double targetVelocity = m_velocityLUT.get(safeDistance);
                                m_launcherSubsystem.setMotorVelocity(targetVelocity);
                                m_lastKnownSpeed = targetVelocity;

                                // Set Hood Position
                                double targetHoodPosition = m_hoodSubsystem.getTargetFromDistance(safeDistance);
                                m_hoodSubsystem.pivotHood(targetHoodPosition);

                                goalFound = true;
                                break;
                            }
                        }
                    }
                }
            }
//            LLResult result = m_limelightSubsystem.getLatestResult();
//
//            boolean goalFound = false;
//
//            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
//            {
//                for (LLResultTypes.FiducialResult tag : result.getFiducialResults())
//                {
//                    if (tag.getFiducialId() == m_targetId) // Safely isolating Tag 24
//                    {
//                        Pose3D pose = tag.getTargetPoseCameraSpace();
//                        if (pose != null)
//                        {
//                            double rawZ = Math.abs(pose.getPosition().z);
//
//                            // --- DYNAMIC TUNING FOR ON-THE-MOVE TRACKING ---
//                            // Ask your drivetrain if the robot is currently moving
//
//                            // If moving, use a high factor (e.g., 0.8) for fast, raw tracking with zero lag.
//                            // If stopped, use a low factor (e.g., 0.15) to aggressively freeze and smooth the jitter.
//
//                            // Constrain within your safe LUT boundaries
//                            double safeDistance = Math.max(0.20, Math.min(m_smoothedDistance, 2.75));
//
//                            // Set Flywheel Speed
//                            double targetVelocity = m_velocityLUT.get(safeDistance);
//                            m_launcherSubsystem.setMotorVelocity(targetVelocity);
//                            m_lastKnownSpeed = targetVelocity;
//
//                            goalFound = true;
//                            break;
//                        }
//                    }
//                }
//            }

        @Override
        public void end(boolean interrupted)
        {

        }

        @Override
        public boolean isFinished()
        { return false; }
    }
