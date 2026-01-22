package teamCode.commands;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.util.function.DoubleSupplier;

import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.GamepadSubsystem;
//TODO Making it work with color and the timer

public class TimerCommand extends CommandBase
{
    private GamepadSubsystem m_gamepadSubsystem;
    public DoubleSupplier m_timer;
    private ColorSensorSubsystem m_colorSubsystem;

    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    public TimerCommand(GamepadSubsystem gamepadSubsystem, DoubleSupplier timer, ColorSensorSubsystem color)
    {
        this.m_gamepadSubsystem = gamepadSubsystem;
//        this.m_colorSubsystem = color;
        this.m_timer = timer;

        addRequirements(m_gamepadSubsystem, this.m_colorSubsystem);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        this.m_gamepadSubsystem.inEndGame(m_timer.getAsDouble());

//        NormalizedRGBA colors = m_colorSubsystem.getNormalizedColors();
//
//        // Convert to HSV for reliable color detection
//        float[] hsv = new float[3];
//        Color.RGBToHSV(
//                (int) (colors.red * 255),
//                (int) (colors.green * 255),
//                (int) (colors.blue * 255),
//                hsv
//        );
//        float hue = hsv[0];
//
//        if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE)
//        {
//            m_colorSubsystem.setLEDGreen();
//        }
//        else if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE)
//        {
//            m_colorSubsystem.setLEDPurple();
//        }
//        else
//        {
//            m_colorSubsystem.setLEDOff(); // Default to off
//        }
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
