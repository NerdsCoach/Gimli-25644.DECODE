package teamCode.commands;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.LightSubsystem;
import teamCode.subsystems.PrismLEDSubsystem;

public class ColorModeOnLEDCommand extends CommandBase
{
    private final ColorSensorSubsystem m_colorSubsystem;
    private final PrismLEDSubsystem m_prismLightSubsystem;
    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    public double m_lastKnownColor;
    private int m_position;
    private static final int  m_off = 1;
    private static final int  m_on = 0;


    public ColorModeOnLEDCommand(ColorSensorSubsystem colorSubsystem, PrismLEDSubsystem prismLEDSubsystem)
    {
        this.m_colorSubsystem = colorSubsystem;
        this.m_prismLightSubsystem = prismLEDSubsystem;

        addRequirements(m_colorSubsystem, m_prismLightSubsystem);
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
        float saturation = hsv[1];

        // Only update if the color is "vibrant" enough (saturation > 0.3)
        // This prevents random hue jumps when the sensor sees the floor
        if (saturation > 0.3) {
            if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE) {
                m_prismLightSubsystem.setGreen();
            } else if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE) {
                m_prismLightSubsystem.setPurple();
            }
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
