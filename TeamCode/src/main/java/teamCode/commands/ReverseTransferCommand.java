package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.TransferSubsystem;

//TODO Run like a trigger

public class ReverseTransferCommand extends CommandBase
{
    private final TransferSubsystem m_transferServoSubsystem;
    private DoubleSupplier m_leftTriggerValue;

    public ReverseTransferCommand(TransferSubsystem transfer, DoubleSupplier leftTrigger)
    {
        this.m_transferServoSubsystem = transfer;
        this.m_leftTriggerValue = leftTrigger;

        addRequirements(this.m_transferServoSubsystem);
    }

    @Override
    public void initialize()
    {

    }

    @Override
    public void execute()
    {
//        this.m_transferServoSubsystem.spinTransfer(.5);

        this.m_transferServoSubsystem.spinTransfer(
                this.m_leftTriggerValue.getAsDouble() * .5);
    }

}
