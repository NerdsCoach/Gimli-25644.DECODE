package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import teamCode.subsystems.HoodServoSubsystem;

public class HoodTestingDownCommand extends CommandBase
{
    private final HoodServoSubsystem hood;

    public HoodTestingDownCommand(HoodServoSubsystem hood)
    {
        this.hood = hood;

        addRequirements(hood); // Claims the hood subsystem
    }

    @Override
    public void initialize() {
        // Optional: set a default starting position when the command starts
        hood.nudgeHoodDown();
    }

    @Override
    public void execute() {
        // Check Right Bumper (Nudge Up)
        // "If it's pressed now, BUT it wasn't pressed in the last loop cycle..."
    }

    @Override
    public boolean isFinished() {
        // Return false so this command stays active while you are practicing/tuning
        return true;
    }
}
