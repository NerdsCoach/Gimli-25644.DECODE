package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.LimitSwitchSubsystem;

//TODO Run like a trigger

public class ReverseTransferCommand extends CommandBase
{
//    private final TransferSubsystem m_transferServoSubsystem;
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
    public void execute()
    {
//        this.m_transferServoSubsystem.spinTransfer(.5);

        this.m_limitSwitchSubsystem.setPower(
                this.m_leftTriggerValue.getAsDouble() * .5);
    }

}
