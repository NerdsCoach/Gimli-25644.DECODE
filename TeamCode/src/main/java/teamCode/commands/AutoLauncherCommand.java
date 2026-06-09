package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LimeLightSubsystem;

public class AutoLauncherCommand extends CommandBase
{
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;
    private final HoodServoSubsystem m_hoodSubsystem;
    private final LimeLightSubsystem m_limelightSubsystem;

    private final InterpLUT m_velocityLUT = new InterpLUT();

    private static final double m_aimFar = Constants.AimingConstants.kFarAim;
    private static final double m_aimClose = Constants.AimingConstants.kCloseAim;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;

    private double m_lastKnownSpeed;

    public AutoLauncherCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem,
                               HoodServoSubsystem hoodSubsystem, LimeLightSubsystem limelightSubsystem)
    {

        this.m_launcherSubsystem = launcherSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        this.m_hoodSubsystem = hoodSubsystem;
        this.m_limelightSubsystem = limelightSubsystem;

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
            this.m_axeSubsystem.pivotAxe(m_axeDown);

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
            {
                LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
                Pose3D pose = tag.getTargetPoseCameraSpace();

                if (pose != null)
                {
                    double zMeters = Math.abs(pose.getPosition().z);
                    double safeDistance = Math.max(0.49, Math.min(zMeters, 2.42));
                    double targetVelocity = m_velocityLUT.get(safeDistance);

                    m_launcherSubsystem.setMotorVelocity(targetVelocity);
                    m_lastKnownSpeed = targetVelocity;
                }
            }
            else
            {
                m_launcherSubsystem.setMotorVelocity(m_lastKnownSpeed);
            }
        }

        @Override
        public void end(boolean interrupted)
        {

        }

        @Override
        public boolean isFinished()
        { return false; }
    }
