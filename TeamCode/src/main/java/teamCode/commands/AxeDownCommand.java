package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.TreeMap;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.HoodServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.LimeLightSubsystem;

public class AxeDownCommand extends CommandBase
    {
        private final AxeSubsystem m_axeSubsystem;

        private static final double m_aimFar = Constants.AimingConstants.kFarAim;
        private static final double m_aimClose = Constants.AimingConstants.kCloseAim;
        private static final double m_axeDown = Constants.AxeConstants.kAxeDown;

        private final TreeMap<Double, Double> m_velocityFromDistanceLUT = new TreeMap<>();
        private double m_lastKnownSpeed;

        public AxeDownCommand(AxeSubsystem axeSubsystem)
        {

            this.m_axeSubsystem = axeSubsystem;

            addRequirements(this.m_axeSubsystem);
        }

        @Override
        public void execute()
        {
            this.m_axeSubsystem.pivotAxe(m_axeDown);

        }

        @Override
        public void end(boolean interrupted)
        {
        }

        @Override
        public boolean isFinished()
        { return false; }
    }
