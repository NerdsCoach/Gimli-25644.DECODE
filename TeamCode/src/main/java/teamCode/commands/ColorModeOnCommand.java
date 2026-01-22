package teamCode.commands;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamCode.Constants;
import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.IntakeServoSubsystem;
import teamCode.subsystems.SorterServoSubsystem;

//TODO making it a toggle??

public class ColorModeOnCommand extends CommandBase
{
    private final ColorSensorSubsystem m_colorSubsystem;
    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    private static final int  m_off = 1;


    public ColorModeOnCommand(ColorSensorSubsystem colorSubsystem)
    {
        this.m_colorSubsystem = colorSubsystem;

        addRequirements(m_colorSubsystem);
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
