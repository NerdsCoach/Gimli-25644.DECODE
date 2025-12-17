package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.SorterServoSubsystem;

public class SorterOffCommand extends CommandBase
{
    private final SorterServoSubsystem m_sorterServoSubsystem;
    private DoubleSupplier m_rightTriggerValue;
    private DoubleSupplier m_leftTriggerValue;

    public SorterOffCommand(SorterServoSubsystem clock)
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
        this.m_sorterServoSubsystem.spinSorter(0.0);
    }
}
