package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import java.util.function.DoubleSupplier;
import teamCode.subsystems.DriveSubsystem;
import teamCode.subsystems.GyroSubsystem;
import teamCode.subsystems.GamepadSubsystem;

public class DriveFieldOrientedCommand extends CommandBase
{

    public DriveSubsystem m_driveSubsystem;
    public DoubleSupplier m_leftX;
    public DoubleSupplier m_leftY;
    public DoubleSupplier m_rightX;
    public DoubleSupplier m_rightY;
    public GyroSubsystem m_gyroSubsystem;
    public GamepadSubsystem m_gamepadSubsystem;

    public DriveFieldOrientedCommand(DriveSubsystem driveSubsystem, GamepadSubsystem gamepadSubsystem, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY)
    {
        this.m_driveSubsystem = driveSubsystem;
        this.m_gamepadSubsystem = gamepadSubsystem;
        addRequirements(m_driveSubsystem, m_gyroSubsystem );

        this.m_leftX = leftX;
        this.m_leftY = leftY;
        this.m_rightX = rightX;
        this.m_rightY = rightY;
    }

    @Override
    public void initialize()
    {
        System.out.println("Running Initialize!");
    }

    @Override
    public void execute()
    {
        this.m_driveSubsystem.headingDrive
                (
                        m_leftX.getAsDouble(),
                        m_leftY.getAsDouble(),
                        m_rightX.getAsDouble(),
                        m_rightY.getAsDouble()
                );
        System.out.println("Running Execute!");
    }
}
