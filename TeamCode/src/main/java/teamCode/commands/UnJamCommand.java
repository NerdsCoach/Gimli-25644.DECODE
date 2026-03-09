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
    private final SorterServoSubsystem m_sorterSubsystem;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;


    public UnJamCommand(SorterServoSubsystem sorterSubsystem, AxeSubsystem axeSubsystem, LimitSwitchSubsystem transfer)
    {
        this.m_axeSubsystem = axeSubsystem;
        this.m_limitSwitchSubsystem = transfer;
        this.m_sorterSubsystem = sorterSubsystem;

        addRequirements(this.m_axeSubsystem, m_limitSwitchSubsystem, this.m_sorterSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_axeSubsystem.pivotAxe(0.6);
        this.m_limitSwitchSubsystem.setTransferPower(0.3);
        this.m_sorterSubsystem.spinSorter(0.75); //-1.0
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        this.m_axeSubsystem.pivotAxe(m_axeDown);
        this.m_limitSwitchSubsystem.setTransferPower(0.0);
        this.m_sorterSubsystem.spinSorter(-0.75); //-1.0

    }
}
