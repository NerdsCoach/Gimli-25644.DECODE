package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.subsystems.SorterServoSubsystem;

public class BellyOfTheBeastCommand extends CommandBase
{
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;

    private final SorterServoSubsystem m_sorterSubsystem;

    public BellyOfTheBeastCommand(SorterServoSubsystem sorterSubsystem)
    {
        this.m_sorterSubsystem = sorterSubsystem;
        addRequirements(this.m_sorterSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
            this.m_sorterSubsystem.spinSorter(-0.75); //-1.0
    }
    @Override
    public void end(boolean interrupted)
    {
        this.m_sorterSubsystem.spinSorter(0.0);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
