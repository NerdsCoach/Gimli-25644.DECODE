package teamCode.commands;

import android.graphics.Color;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.LightSubsystem;

public class ColorModeOnCommand extends CommandBase
{
    private final ColorSensorSubsystem m_colorSubsystem;
    private final LightSubsystem m_lightSubsystem;
    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    public double m_lastKnownColor;
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;


    public ColorModeOnCommand(ColorSensorSubsystem colorSubsystem, LightSubsystem lightSubsystem)
    {
        this.m_colorSubsystem = colorSubsystem;
        this.m_lightSubsystem = lightSubsystem;

        addRequirements(m_colorSubsystem, m_lightSubsystem);
    }

    @Override
    public void execute()
    {
            NormalizedRGBA colors = m_colorSubsystem.getNormalizedColors();

            // Convert to HSV for reliable color detection
            float[] hsv = new float[3];
            Color.RGBToHSV(
                    (int) (colors.red * 255),
                    (int) (colors.green * 255),
                    (int) (colors.blue * 255),
                    hsv
            );
            float hue = hsv[0];


        if (m_position == m_off)
        {
            if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE)
            {
                m_lightSubsystem.setLEDGreen();
                this.m_lastKnownColor = 0.5;
            }
            else if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE)
            {
                m_lightSubsystem.setLEDPurple();
                this.m_lastKnownColor = 0.722;
            }
            else
            {
                m_lightSubsystem.lastKnown(m_lastKnownColor); // Default to off
            }
            m_position = m_on;
        }
        else if (m_position == m_on)
        {

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
        return false;
    }
}
