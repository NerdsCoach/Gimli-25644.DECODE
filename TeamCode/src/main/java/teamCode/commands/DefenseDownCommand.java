package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.ParkingSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class DefenseDownCommand extends CommandBase
{
    private ParkingSubsystem m_parkingSubsystem;

    public DefenseDownCommand(ParkingSubsystem parkingSubsystem)
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
            this.m_parkingSubsystem.parking(Constants.ParkConstants.kDefenseDown);
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
