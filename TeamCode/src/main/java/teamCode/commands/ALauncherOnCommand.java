package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import teamCode.Constants;
import teamCode.subsystems.AxeSubsystem;
import teamCode.subsystems.LauncherSubsystem;

public class ALauncherOnCommand extends CommandBase {
    private final LauncherSubsystem m_launcherSubsystem;
    private final AxeSubsystem m_axeSubsystem;

    private static final double m_axeDown = Constants.AxeConstants.kAxeDown;

    public ALauncherOnCommand(LauncherSubsystem launcherSubsystem, AxeSubsystem axeSubsystem) {
        this.m_launcherSubsystem = launcherSubsystem;
        this.m_axeSubsystem = axeSubsystem;

        addRequirements(this.m_launcherSubsystem, this.m_axeSubsystem);
    }

    @Override
    public void initialize() {
        this.m_launcherSubsystem.fudgeFactor(100);
        this.m_axeSubsystem.pivotAxe(m_axeDown);
    }

    @Override
    public void execute()
    {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        // Returning true makes this a one-shot command, running once per button press.
        return true;
    }
}
