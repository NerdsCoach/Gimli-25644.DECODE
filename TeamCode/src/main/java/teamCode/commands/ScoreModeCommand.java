package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import static teamCode.Constants.SorterConstants.kScorePos1;
import static teamCode.Constants.SorterConstants.kScorePos2;
import static teamCode.Constants.SorterConstants.kScorePos3;

import static teamCode.Constants.TransferConstants.kTransferDown;
import static teamCode.Constants.TransferConstants.kTransferUp;


import teamCode.Constants;
import teamCode.subsystems.LauncherMotorSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TransferServoSubsystem;

public class ScoreModeCommand extends CommandBase
{
    private SorterServoSubsystem m_sorterServoSubsystem;
    private TransferServoSubsystem m_transferServoSubsystem;
    private LauncherMotorSubsystem m_launcherMotorSubsystem;

    private static final double m_scorePos1 = Constants.SorterConstants.kScorePos1;
    private static final double m_scorePos2 = Constants.SorterConstants.kScorePos2;
    private static final double m_scorePos3 = Constants.SorterConstants.kScorePos3;
    private double m_position;
    private static final double  m_pos1= 1;
    private static final int m_pos2 = 2;
    private static final int m_pos3 = 3;
    private StateMachine m_stateMachine;
    public int sortPos = 1;

    enum StateMachine
    {
        TURN_SORTER_1,
        TURN_SORTER_2,
        TURN_SORTER_3,
        TRANSFER_UP,
        TRANSFER_DOWN,
        COUNTER_SCORE,
        TURN_OFF,
    }

    public ScoreModeCommand(SorterServoSubsystem sorterSubsystem, TransferServoSubsystem transferServoSubsystem, LauncherMotorSubsystem launcherMotorSubsystem)
    {
        this.m_sorterServoSubsystem = sorterSubsystem;
        this.m_transferServoSubsystem = transferServoSubsystem;
        this.m_launcherMotorSubsystem = launcherMotorSubsystem;
        m_stateMachine = StateMachine.COUNTER_SCORE;

        m_position = m_pos1;
        addRequirements(m_sorterServoSubsystem, m_transferServoSubsystem,m_launcherMotorSubsystem);
    }
    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        switch (m_stateMachine)
        {
            case COUNTER_SCORE:
                if (sortPos == 1)
                {
                    m_stateMachine = StateMachine.TURN_SORTER_1;
                }
                else if (sortPos == 2)
                {
                    m_stateMachine = StateMachine.TURN_SORTER_2;
                }
                else if (sortPos == 3)
                {
                    m_stateMachine = StateMachine.TURN_SORTER_3;
                }
                else if (sortPos == 4)
                {
                    m_stateMachine = StateMachine.TURN_OFF;
                }
                break;

            case TURN_SORTER_1:
                this.m_sorterServoSubsystem.sort(kScorePos1);
                this.m_launcherMotorSubsystem.launch();
                sortPos = sortPos + 1;
                m_stateMachine = StateMachine.TRANSFER_UP;
                break;

            case TURN_SORTER_2:
                if(this.m_transferServoSubsystem.atTarget(kTransferDown))
                {
                    this.m_sorterServoSubsystem.sort(kScorePos2);
                    sortPos = sortPos + 1;
                    m_stateMachine = StateMachine.TRANSFER_UP;
                    break;
                }

            case TURN_SORTER_3:
                if(this.m_transferServoSubsystem.atTarget(kTransferDown))
                {
                    this.m_sorterServoSubsystem.sort(kScorePos3);
                    sortPos = sortPos + 1;
                    m_stateMachine = StateMachine.TRANSFER_UP;
                    break;
                }

            case TRANSFER_UP:
                if(this.m_sorterServoSubsystem.atTarget(kScorePos1))
                {
                    this.m_transferServoSubsystem.transfer(kTransferUp);
                    m_stateMachine =StateMachine.TRANSFER_DOWN;
                    break;
                }

            case TRANSFER_DOWN:
                if(this.m_transferServoSubsystem.atTarget(kTransferUp))
                {
                    this.m_transferServoSubsystem.transfer(kTransferDown);
                    m_stateMachine = StateMachine.COUNTER_SCORE;
                    break;
                }

            case TURN_OFF:
                if(this.m_transferServoSubsystem.atTarget(kTransferDown))
                {
                    this.m_launcherMotorSubsystem.stop();
                    m_stateMachine =StateMachine.COUNTER_SCORE;
                    sortPos = 1;
                    break;
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
