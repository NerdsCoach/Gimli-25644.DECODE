package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import teamCode.Constants;
import teamCode.subsystems.IntakeMotorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;

public class IntakeServoCommand extends CommandBase
{
    private final IntakeServoSubsystem m_intakeServoSubsystem;
    private static final double m_intakeOn = Constants.IntakeConstants.kIntakeOn;
    private static final double m_intakeOff = Constants.IntakeConstants.kIntakeOff;
    private static final double m_servoOn = Constants.IntakeConstants.kServoOn;

    public IntakeServoCommand(IntakeServoSubsystem intakeServo)
    {
        this.m_intakeServoSubsystem = intakeServo;
        addRequirements(this.m_intakeServoSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
            this.m_intakeServoSubsystem.spinServo(m_intakeOn);
    }
    @Override
    public void end(boolean interrupted)
    {
        this.m_intakeServoSubsystem.spinServo(m_intakeOff);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
