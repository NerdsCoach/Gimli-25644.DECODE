package teamCode.commands;

import android.graphics.Color;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamCode.subsystems.ColorSensorSubsystem;

public class ColorModeOffCommand extends CommandBase
{
    /* Senses color and displays it Via Light */
    private final ColorSensorSubsystem m_colorSubsystem;

    public ColorModeOffCommand(ColorSensorSubsystem colorSubsystem)
    {
        this.m_colorSubsystem = colorSubsystem;

        addRequirements(m_colorSubsystem);
    }

    @Override
    public void execute()
    {
            m_colorSubsystem.setLEDOff(); // Default to off
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
