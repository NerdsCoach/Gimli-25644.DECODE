package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.subsystems.GimlisBoxMotorSubsystem;

public class DeParkingCommand extends CommandBase
{
    private GimlisBoxMotorSubsystem m_parkingSubsystem;

    public DeParkingCommand(GimlisBoxMotorSubsystem parkingSubsystem)
    {
        this.m_parkingSubsystem = parkingSubsystem;

        addRequirements(m_parkingSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
//        this.m_parkingSubsystem.dePark();
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
