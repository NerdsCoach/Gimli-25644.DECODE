package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import teamCode.Constants;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.SorterServoSubsystem;

public class IntakeModeCommand extends CommandBase
{
    private final IntakeServoSubsystem m_intakeWheelSubsystem;
    private static final double m_intakeOn = Constants.IntakeConstants.kIntakeOn;
    private static final double m_intakeOff = Constants.IntakeConstants.kIntakeOff;
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;

//    private final SorterServoSubsystem m_sorterSubsystem;

    public IntakeModeCommand(IntakeServoSubsystem intake /*SorterServoSubsystem sorterSubsystem*/)
    {
        this.m_intakeWheelSubsystem = intake;
//        this.m_sorterSubsystem = sorterSubsystem;
        addRequirements(this.m_intakeWheelSubsystem /*, this.m_sorterSubsystem*/);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        if (m_position == m_off)
        {
            this.m_intakeWheelSubsystem.spinIntake(m_intakeOn);
//            this.m_sorterSubsystem.spinSorter(-.75);
            m_position = m_on;
        }
        else if (m_position == m_on)
        {
            this.m_intakeWheelSubsystem.spinIntake(m_intakeOff);
//            this.m_sorterSubsystem.spinSorter(0.0);
            m_position = m_off;
        }
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
