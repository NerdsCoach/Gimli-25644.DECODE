package teamCode.commands;
import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.AimingServoSubsystem;
import teamCode.subsystems.HoodServoSubsystem;

public class HoodTestingCommand extends CommandBase
{
    private HoodServoSubsystem m_hoodServoSubsystem;

    private static final double m_farAim = Constants.AimingConstants.kFarAim;
    private static final double m_closeAim = Constants.AimingConstants.kCloseAim;
    private int m_position;
    private static final int m_up = 1;
    private static final int m_down = 0;

    public HoodTestingCommand(HoodServoSubsystem hoodServoSubsystem)
    {
        m_position = m_down;

        this.m_hoodServoSubsystem = hoodServoSubsystem;
        addRequirements(this.m_hoodServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        if (m_position == m_down)
        {
            this.m_hoodServoSubsystem.pivotHood(m_closeAim);
            m_position = m_up;
        }
        else if (m_position == m_up)
        {
            this.m_hoodServoSubsystem.pivotHood(m_farAim);
            m_position = m_down;
        }
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
