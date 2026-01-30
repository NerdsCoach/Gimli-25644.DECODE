package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import java.util.TreeMap;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.HuskyLensSubsystem;
import teamCode.subsystems.LauncherSubsystem;

public class ALauncherOnCommand extends CommandBase
{
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;

    private TreeMap<Double, Double> distanceLUT = new TreeMap<>();
    private TreeMap<Double, Double> velocityLUT = new TreeMap<>();

    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;
    private double m_lastKnownSpeed;

    public ALauncherOnCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem)
    {
        m_position = m_down;

        this.m_launcherSubsystem = launcherSubsystem;
        this.m_axeSubsystem = axeSubsystem;

        addRequirements(this.m_launcherSubsystem, this.m_axeSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
                // If we lose sight, DON'T set to 0. Use the saved memory!
                m_launcherSubsystem.setMotorVelocity(2350);
                this.m_axeSubsystem.pivotAxe(m_axeDown);
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
