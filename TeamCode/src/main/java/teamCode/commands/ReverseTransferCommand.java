package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import teamCode.subsystems.LimitSwitchSubsystem;


public class ReverseTransferCommand extends CommandBase
{
    private final LimitSwitchSubsystem m_limitSwitchSubsystem;
    private final DoubleSupplier m_leftTriggerValue;

    public ReverseTransferCommand(LimitSwitchSubsystem transfer, DoubleSupplier leftTrigger)
    {
        this.m_limitSwitchSubsystem = transfer;
        this.m_leftTriggerValue = leftTrigger;

        addRequirements(this.m_limitSwitchSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute() {
        // Get the 0 to 1.0 value
        double triggerVal = this.m_leftTriggerValue.getAsDouble();
        this.m_limitSwitchSubsystem.setTransferPower(triggerVal);

    }

    @Override
    public boolean isFinished()
    {
        // If the trigger is basically zero, tell the scheduler this command is DONE.
        return this.m_leftTriggerValue.getAsDouble() < 0.05;
    }

    @Override
    public void end(boolean interrupted)
    {
        this.m_limitSwitchSubsystem.setTransferPower(0);
    }
}
