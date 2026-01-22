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
        public static final int kParkLiftLimit= 2000;
        public static final int kDeParkLimit= 25;
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
        public static final int kOutTake = 1;
    }

    public static final class AxeConstants
    {
        public static final double kAxeDown = 0.5;
        public static final double kAxeUp = 0.62;// down...
    }

    public static final class AimingConstants
    {
        public static final double kCloseAim = .7; //Up
        public static final double kFarAim = .89;// down...
    }

}
