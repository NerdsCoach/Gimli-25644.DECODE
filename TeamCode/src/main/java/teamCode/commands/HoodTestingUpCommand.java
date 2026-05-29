package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import teamCode.subsystems.HoodServoSubsystem;

public class HoodTestingUpCommand extends CommandBase
{
    private final HoodServoSubsystem hood;
    public HoodTestingUpCommand(HoodServoSubsystem hood)
    {
        this.hood = hood;
        addRequirements(hood); // Claims the hood subsystem
    }

    @Override
    public void initialize() {
        // Optional: set a default starting position when the command starts
        hood.nudgeHoodUp();
    }

    @Override
    public void execute() {
        // Check Right Bumper (Nudge Up)


        // Display the "fake" position reading on the Driver Hub screen
    }

    @Override
    public boolean isFinished() {
        // Return false so this command stays active while you are practicing/tuning
        return true;
    }
}
