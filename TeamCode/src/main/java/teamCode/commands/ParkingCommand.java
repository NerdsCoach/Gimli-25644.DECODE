package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.ParkingSubsystem;
import teamCode.subsystems.SorterServoSubsystem;

public class ParkingCommand extends CommandBase
{
    private ParkingSubsystem m_parkingSubsystem;
    private IntakeServoSubsystem m_intakeSubsystem;
    private LauncherSubsystem m_launcherSubsystem;
    private SorterServoSubsystem m_bellyOfTheBeastSubsystem;

    public ParkingCommand(ParkingSubsystem parkingSubsystem, IntakeServoSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, SorterServoSubsystem bellyOfTheBeastSubsystem)
    {
        this.m_parkingSubsystem = parkingSubsystem;
        this.m_intakeSubsystem = intakeSubsystem;
        this.m_launcherSubsystem = launcherSubsystem;
        this.m_bellyOfTheBeastSubsystem = bellyOfTheBeastSubsystem;

        addRequirements(m_parkingSubsystem, this.m_intakeSubsystem, this.m_launcherSubsystem, this.m_bellyOfTheBeastSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
            this.m_parkingSubsystem.parking(Constants.ParkConstants.kParkLiftLimit);
            this.m_intakeSubsystem.spinIntake(0.0);
            this.m_launcherSubsystem.setMotorVelocity(0.0);
            this.m_bellyOfTheBeastSubsystem.spinSorter(0.0);
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
