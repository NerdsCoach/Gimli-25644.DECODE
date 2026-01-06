package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamCode.subsystems.ColorSensorSubsystem;
/* Sees correct color, Axe Down, Launcher On, Transfer On x3, Off */
public class ScoreTestingCommand extends CommandBase
{
    private final ColorSensorSubsystem m_colorSubsystem;
    // Define target hues based on user's readings
    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    public ScoreTestingCommand(ColorSensorSubsystem colorSubsystem)
    {
        m_colorSubsystem = colorSubsystem;
        addRequirements(m_colorSubsystem);
    }

    @Override
    public void execute()
    {
        NormalizedRGBA colors = m_colorSubsystem.getNormalizedColors();

        // Convert to HSV for reliable color detection
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
        float hue = hsv[0];

          if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE)
        {
            m_colorSubsystem.setLEDGreen();
        }
          else if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE)
        {
            m_colorSubsystem.setLEDPurple();
        }
          else
        {
            m_colorSubsystem.setLEDOff(); // Default to off
        }
    }
}
