package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.LimitSwitchSubsystem;
import teamCode.subsystems.SorterServoSubsystem;

public class UnJamCommand extends CommandBase
{
    private final AxeSubsystem m_axeSubsystem;
    private final LimitSwitchSubsystem m_limitSwitchSubsystem;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;

    private final SorterServoSubsystem m_sorterSubsystem;

    public UnJamCommand(SorterServoSubsystem sorterSubsystem, AxeSubsystem axeSubsystem, LimitSwitchSubsystem transfer)
    {
        this.m_sorterSubsystem = sorterSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        this.m_limitSwitchSubsystem = transfer;

        addRequirements(this.m_sorterSubsystem, this.m_axeSubsystem, m_limitSwitchSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
            this.m_sorterSubsystem.spinSorter(-0.75); //-1.0
            this.m_limitSwitchSubsystem.setTransferPower(0.3);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        this.m_sorterSubsystem.spinSorter(-0.75); //-1.0
        this.m_axeSubsystem.pivotAxe(m_axeDown);
    }
}
