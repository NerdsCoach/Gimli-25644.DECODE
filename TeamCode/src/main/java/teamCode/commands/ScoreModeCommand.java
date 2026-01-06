package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.TransferSubsystem;
public class ScoreModeCommand extends CommandBase
{
    /* Axe Down, Launcher Motor On, Transfer On x 3, Off */
    private final ColorSensorSubsystem m_colorSubsystem;
    private final AxeSubsystem m_axeSubsystem;
    private static final double m_axeUp = Constants.AxeConstants.kAxeUp;
    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;
    private final TransferSubsystem m_transferSubsystem;
    private DigitalChannel m_limitSwitch;

    private int m_position;
    private static final int  m_sample = 1;
    private static final int  m_specimen = 0;

    // Define target hues based on user's readings
    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance


    public ScoreModeCommand(ColorSensorSubsystem colorSubsystem, AxeSubsystem axeSubsystem, TransferSubsystem transferSubsystem)
    {
        this.m_colorSubsystem = colorSubsystem;
        this.m_axeSubsystem = axeSubsystem;
        this.m_transferSubsystem = transferSubsystem;
        addRequirements(m_colorSubsystem, m_axeSubsystem, m_transferSubsystem);
    }
    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        if (m_position == m_specimen)
        {
            this.m_axeSubsystem.pivotAxe(m_axeDown);
            this.m_transferSubsystem.spinTransfer(-.75);
            m_position = m_sample;
        }
        else if (m_position == m_sample)
        {
            this.m_axeSubsystem.pivotAxe(m_axeUp);
            m_position = m_specimen;
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
