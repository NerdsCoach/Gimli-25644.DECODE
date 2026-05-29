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
import teamCode.subsystems.SorterServoSubsystem;

public class LauncherBellyCommand extends CommandBase
{
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;
    private final SorterServoSubsystem m_sorterSubsystem;
    private final HoodServoSubsystem m_hoodSubsystem;
    private final LimeLightSubsystem m_limelightSubsystem;

    private final InterpLUT m_velocityLUT = new InterpLUT();

    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;

    private double m_lastKnownSpeed;
    private double m_lastKnownHoodPosition;

    public LauncherBellyCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem,
                                HoodServoSubsystem hoodSubsystem, LimeLightSubsystem limelightSubsystem,
                                SorterServoSubsystem sorterServo)
    {
        this.m_launcherSubsystem = launcherSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        this.m_hoodSubsystem = hoodSubsystem;
        this.m_limelightSubsystem = limelightSubsystem;
        this.m_sorterSubsystem = sorterServo;

        m_lastKnownSpeed = 1500.0;
        m_lastKnownHoodPosition = 1.0;

        // Added m_hoodSubsystem back to requirements so no other command steals it while firing
        addRequirements(this.m_launcherSubsystem, this.m_hoodSubsystem, this.m_sorterSubsystem, this.m_axeSubsystem);

        m_velocityLUT.add(0.20, 1450.0);
        m_velocityLUT.add(0.53, 1650.0);
        m_velocityLUT.add(0.80, 1750.0);
        m_velocityLUT.add(0.99, 1820.0);
        m_velocityLUT.add(1.07, 1920.0);
        m_velocityLUT.add(1.20, 2000.0);
        m_velocityLUT.add(1.35, 2100.0);
        m_velocityLUT.add(1.49, 2200.0);
        m_velocityLUT.add(1.79, 2260.0);
        m_velocityLUT.add(2.05, 2360.0);
        m_velocityLUT.add(2.36, 2480.0);

        m_velocityLUT.createLUT();
    }

    @Override
    public void execute()
    {
        LLResult result = m_limelightSubsystem.getLatestResult();
        this.m_axeSubsystem.pivotAxe(m_axeDown);
        this.m_sorterSubsystem.spinSorter(-0.75);

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
        {
            LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
            Pose3D pose = tag.getTargetPoseCameraSpace();

            if (pose != null)
            {
                double zMeters = Math.abs(pose.getPosition().z);
                double safeDistance = Math.max(0.49, Math.min(zMeters, 2.42));

                // 1. Calculate and set the Flywheel Speed
                double targetVelocity = m_velocityLUT.get(safeDistance);
                m_launcherSubsystem.setMotorVelocity(targetVelocity);
                m_lastKnownSpeed = targetVelocity;

                // 2. Calculate and set the Hood Position using your custom math!
                double targetHoodPosition = m_hoodSubsystem.getTargetFromDistance(safeDistance);
                m_hoodSubsystem.pivotHood(targetHoodPosition);
                m_lastKnownHoodPosition = targetHoodPosition; // SAVE IT TO MEMORY HERE!
            }
        }
        else
        {
            m_launcherSubsystem.setMotorVelocity(m_lastKnownSpeed);
            m_hoodSubsystem.pivotHood(m_lastKnownHoodPosition);
            // Optional: If vision drops out, you could either freeze the hood where it is,
            // or send it to a default fallback position here. Leaving it blank freezes it.
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        this.m_axeSubsystem.pivotAxe(m_axeUp);
        this.m_sorterSubsystem.spinSorter(0.0 );
        this.m_launcherSubsystem.setMotorVelocity(0.0);
        // Optional: override hood location on end if you want it to fold flat when done
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}


//package teamCode.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.util.InterpLUT;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//
//import teamCode.Constants;
//import teamCode.subsystems.AxeSubsystem;
//import teamCode.subsystems.HoodServoSubsystem;
//import teamCode.subsystems.LauncherSubsystem;
//import teamCode.subsystems.LimeLightSubsystem;
//import teamCode.subsystems.SorterServoSubsystem;
//
//public class LauncherBellyCommand extends CommandBase
//{
//    private final LauncherSubsystem m_launcherSubsystem;
//    private final AxeSubsystem m_axeSubsystem;
//    private final SorterServoSubsystem m_sorterSubsystem;
////    private final HoodServoSubsystem m_hoodSubsystem;
//    private final LimeLightSubsystem m_limelightSubsystem;
//
//    private final InterpLUT m_velocityLUT = new InterpLUT();
//
////    private static final double m_aimFar = Constants.AimingConstants.kFarAim;
////    private static final double m_aimClose = Constants.AimingConstants.kCloseAim;
//    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
//    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
//
//
//    private double m_lastKnownSpeed;
//
//    public LauncherBellyCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem,
//                                /*HoodServoSubsystem hoodSubsystem,*/ LimeLightSubsystem limelightSubsystem,
//                                SorterServoSubsystem sorterServo)
//    {
//        this.m_launcherSubsystem = launcherSubsystem;
//        this.m_axeSubsystem = axeSubsystem;
////        this.m_hoodSubsystem = hoodSubsystem;
//        this.m_limelightSubsystem = limelightSubsystem;
//        this.m_sorterSubsystem = sorterServo;
//
//        m_lastKnownSpeed = 1500.0;
//
//        addRequirements(this.m_launcherSubsystem,/* this.m_hoodSubsystem,*/ this.m_sorterSubsystem, this.m_axeSubsystem);
//
//        m_velocityLUT.add(0.20, 1450.0); //TODO Closer Numbers
//        m_velocityLUT.add(0.53, 1650.0);
//        m_velocityLUT.add(0.80, 1750.0);
//        m_velocityLUT.add(0.99, 1820.0);
//        m_velocityLUT.add(1.07, 1920.0);
//        m_velocityLUT.add(1.20, 2000.0);
//        m_velocityLUT.add(1.35, 2100.0);
//        m_velocityLUT.add(1.49, 2200.0);
//        m_velocityLUT.add(1.79, 2260.0);
//        m_velocityLUT.add(2.05, 2360.0);
//        m_velocityLUT.add(2.36, 2480.0);
//        // Finalize the LUT
//        m_velocityLUT.createLUT();
//    }
//
//        @Override
//        public void execute()
//        {
//            LLResult result = m_limelightSubsystem.getLatestResult();
//            this.m_axeSubsystem.pivotAxe(m_axeDown);
//            this.m_sorterSubsystem.spinSorter(-0.75);
//
//            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty())
//            {
//                LLResultTypes.FiducialResult tag = result.getFiducialResults().get(0);
//                Pose3D pose = tag.getTargetPoseCameraSpace();
//
//                if (pose != null)
//                {
//                    double zMeters = Math.abs(pose.getPosition().z);
//                    double safeDistance = Math.max(0.49, Math.min(zMeters, 2.42));
//                    double targetVelocity = m_velocityLUT.get(safeDistance);
//
//                    m_launcherSubsystem.setMotorVelocity(targetVelocity);
//                    m_lastKnownSpeed = targetVelocity;
//
////                    if (targetVelocity > 1900)
////                    {
////                        this.m_hoodSubsystem.pivotHood(m_aimFar);
////                    }
////                    else
////                    {
////                        this.m_hoodSubsystem.pivotHood(m_aimClose);
////                    }
//                }
//            }
//            else
//            {
//                m_launcherSubsystem.setMotorVelocity(m_lastKnownSpeed);
//            }
//        }
//
//        @Override
//        public void end(boolean interrupted)
//        {
//            this.m_axeSubsystem.pivotAxe(m_axeUp);
//            this.m_sorterSubsystem.spinSorter(0.0 );
//            this.m_launcherSubsystem.setMotorVelocity(0.0);
//
//        }
//
//        @Override
//        public boolean isFinished()
//        {
//            return false;
//        }
//    }
