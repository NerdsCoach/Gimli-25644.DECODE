package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import static teamCode.Constants.SorterConstants.kScorePos1;
import static teamCode.Constants.SorterConstants.kScorePos2;
import static teamCode.Constants.SorterConstants.kScorePos3;
import teamCode.Constants;
import teamCode.subsystems.SorterServoSubsystem;

public class ScoreTestingModeCommand extends CommandBase
{
    private SorterServoSubsystem m_sorterServoSubsystem;

    private static final double m_scorePos1 = kScorePos1;
    private static final double m_scorePos2 = kScorePos2;
    private static final double m_scorePos3 = kScorePos3;
    private double m_position;
    private static final double  m_pos1= 1;
    private static final int m_pos2 = 2;
    private static final int m_pos3 = 3;

    public ScoreTestingModeCommand(SorterServoSubsystem sorterSubsystem)
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
        this.m_sorterServoSubsystem.sort(Constants.SorterConstants.kSorterPos1);
        if (m_sorterServoSubsystem.atTarget(Constants.SorterConstants.kSorterPos1))
        {
            if (m_position == m_pos1)
            {
                this.m_sorterServoSubsystem.sort(m_scorePos2);
                m_position = m_pos2;
            }
            else if (m_position == m_pos2)
            {
                this.m_sorterServoSubsystem.sort(m_scorePos3);
                m_position = m_pos3;
            } else if (m_position == m_pos3)
            {
                this.m_sorterServoSubsystem.sort(m_scorePos1);
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
