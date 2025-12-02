package teamCode;

public class Constants
{
    public static final class DriveTrainConstants
    {
        public static final int kRandomValue = 0;
    }
    //Call values with
//    Constants.DriveTrainConstants.kRandomValue

    public static final class ParkConstants
    {
        public static final int kLiftPark = -50; //???
        public static final int kDePark = 50;
    }

    public static final class TurnTableConstants
    {
       public static final int kTurnTable = 150; //???
    }

    public static final class LauncherConstants
    {
        public static final int kLaunch = 150;//???
    }

    public static final class SorterConstants
    {
        public static final double kSorterPos1 = 0.24;//0.14
        public static final double kSorterPos2 = 0.17;//0.18
        public static final double kSorterPos3 = 0.095;//0.01

//intake 3 should be close to launch 1
        public static final double kScorePos1 = 0.06;// 0.05
        public static final double kScorePos2 = 0.13;//0.11
        public static final double kScorePos3 = 0.20;
    }

    public static final class TransferConstants
    {
        public static final double kTransferDown = 0.15;
        public static final double kTransferUp = 0.5;
    }

    public static final class LightConstants
    {
        public static final double kLightOn = 0.5;
        public static final double kLightOff = 0.0;
    }
}
