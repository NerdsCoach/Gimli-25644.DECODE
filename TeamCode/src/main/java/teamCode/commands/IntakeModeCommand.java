package teamCode.commands;

import static teamCode.Constants.SorterConstants.kSorterPos1;
import static teamCode.Constants.SorterConstants.kSorterPos2;
import static teamCode.Constants.SorterConstants.kSorterPos3;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.SorterServoSubsystem;

public class IntakeModeCommand  extends CommandBase
{
    private SorterServoSubsystem m_sorterServoSubsystem;

    private static final double m_sorterPos1 = kSorterPos1;
    private static final double m_sorterPos2 = kSorterPos2;
    private static final double m_sorterPos3 = kSorterPos3;
    private double m_position;
    private static final double  m_pos1= 1;
    private static final int m_pos2 = 2;
    private static final int m_pos3 = 3;

    public IntakeModeCommand(SorterServoSubsystem sorterSubsystem)
    {
        this.m_sorterServoSubsystem = sorterSubsystem;
        m_position = m_pos1;
        addRequirements(m_sorterServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_sorterServoSubsystem.sort(kSorterPos1);
        if (m_sorterServoSubsystem.atTarget(kSorterPos1))
        {
            if (m_position == m_pos1)
            {
                this.m_sorterServoSubsystem.sort(m_sorterPos2);
                m_position = m_pos2;
            }
            else if (m_position == m_pos2)
            {
                this.m_sorterServoSubsystem.sort(m_sorterPos3);
                m_position = m_pos3;
            }
            else if (m_position == m_pos3)
            {
                this.m_sorterServoSubsystem.sort(m_sorterPos1);
                m_position = m_pos1;
            }
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
