package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.LauncherSubsystem;
import teamCode.subsystems.ParkingSubsystem;
import teamCode.subsystems.SorterServoSubsystem;
import teamCode.subsystems.TurnTableSubsystem;

public class ParkingCommand extends CommandBase
{
    private ParkingSubsystem m_parkingSubsystem;
    private IntakeMotorSubsystem m_intakeMotorSubsystem;
    private LauncherSubsystem m_launcherSubsystem;
    private SorterServoSubsystem m_bellyOfTheBeastSubsystem;
    private TurnTableSubsystem m_turnTableSubsystem;

    public ParkingCommand(ParkingSubsystem parkingSubsystem, IntakeMotorSubsystem intakeMotorSubsystem, LauncherSubsystem launcherSubsystem, SorterServoSubsystem bellyOfTheBeastSubsystem, TurnTableSubsystem turnTableSubsystem)
    {
        this.m_parkingSubsystem = parkingSubsystem;
        this.m_intakeMotorSubsystem = intakeMotorSubsystem;
        this.m_launcherSubsystem = launcherSubsystem;
        this.m_bellyOfTheBeastSubsystem = bellyOfTheBeastSubsystem;
        this.m_turnTableSubsystem = turnTableSubsystem;

        addRequirements(m_parkingSubsystem, this.m_intakeMotorSubsystem, this.m_launcherSubsystem, this.m_bellyOfTheBeastSubsystem, this.m_turnTableSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
            this.m_parkingSubsystem.parking(Constants.ParkConstants.kParkLiftLimit);
            this.m_intakeMotorSubsystem.spinMotorIntake(0.0);
            this.m_launcherSubsystem.setMotorVelocity(0.0);
            this.m_bellyOfTheBeastSubsystem.spinSorter(0.0);
            this.m_turnTableSubsystem.stop();
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
