package teamCode;

public class Constants
{
    public static final class DriveTrainConstants
    {
        public static final int kRandomValue = 0;
    }

    public static final class ParkConstants
    {
        public static final int kLiftPark = 50; //???
        public static final int kDePark = -50;
        public static final int kParkLiftLimit= 1000;
        public static final int kDeParkLimit= 0;
    }

    public static final class TurnTableConstants
    {
        public static final int kTurnTableLeft = 50; //???
        public static final int kTurnTableRight = -50; //???
    }

    public static final class IntakeConstants
    {
        public static final int kIntakeOn = -1;
        public static final int kIntakeOff = 0;
    }

    public static final class LightConstants
    {
        public static final double kLightOn = 0.5;
        public static final double kLightOff = 0.0;
    }

    public static final class AxeConstants
    {
        public static final double kAxeDown = 0.5;
        public static final double kAxeUp = 0.68;
    }

    public static final class LauncherServoConstants
    {
        public static final double kAimFar = 0.5;
        public static final double kAimClose = 0.6;
    }
}
