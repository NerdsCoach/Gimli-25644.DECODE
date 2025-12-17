package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.subsystems.SorterServoSubsystem;

public class SorterCommand extends CommandBase
{
    private final SorterServoSubsystem m_sorterServoSubsystem;

    public SorterCommand(SorterServoSubsystem clock)
    {
        this.m_sorterServoSubsystem = clock;
        addRequirements(this.m_sorterServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_sorterServoSubsystem.spinSorter(-.75);
    }
}
