package teamCode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import teamCode.subsystems.ColorSensorSubsystem;
import teamCode.subsystems.LightSubsystem;

public class ColorSensorCommand extends CommandBase
{
    private final ColorSensorSubsystem m_colorSubsystem;

    // Define target hues based on user's readings
    private static final float TARGET_GREEN_HUE = 160.0f;
    private static final float TARGET_PURPLE_HUE = 240.0f;
    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance

    public ColorSensorCommand(ColorSensorSubsystem colorSubsystem, LightSubsystem lightSubsystem)
    {
        m_colorSubsystem = colorSubsystem;
        addRequirements(m_colorSubsystem);
    }

    @Override
    public void execute()
    {
        NormalizedRGBA colors = m_colorSubsystem.getNormalizedColors();

        // Convert to HSV for reliable color detection
        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(
                (int) (colors.red * 255),
                (int) (colors.green * 255),
                (int) (colors.blue * 255),
                hsv
        );
        float hue = hsv[0];

        // Detection logic based on standard Hue ranges
//        if (hue > 90 && hue < 150)
//        {
//            m_colorSubsystem.setLEDGreen();
//        } else if (hue > 250 && hue < 300)
//        {
//            m_colorSubsystem.setLEDPurple();
//        } else
//        {
//            m_colorSubsystem.setLEDOff();
//        }

        if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE)
        {
            m_colorSubsystem.setLEDGreen();
        } else if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE)
        {
            m_colorSubsystem.setLEDPurple();
        } else
        {
            m_colorSubsystem.setLEDOff(); // Default to off
        }
    }
}
//    public RevColorSensorV3 m_colorSensor;
//    public ColorSensorSubsystem m_colorSensorSubsystem;
//    public LightSubsystem m_lightSubsystem;
//
//    View relativeLayout;
//
//    public ColorSensorCommand(ColorSensorSubsystem colorSensorSubsystem, LightSubsystem lightSubsystem)
//    {
////        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
////        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//        this.m_colorSensorSubsystem = colorSensorSubsystem;
//        this.m_lightSubsystem = lightSubsystem;
//        addRequirements(colorSensorSubsystem, lightSubsystem); // If using a subsystem
//    }
//
//    @Override
//    public void execute()
//    {
//            relativeLayout.post(new Runnable()
//            {
//                public void run()
//                {
//                    relativeLayout.setBackgroundColor(Color.WHITE);
//                }
//            });
//            float gain = 2;
//
//            final float[] hsvValues = new float[3];
//
//            boolean xButtonPreviouslyPressed = false;
//            boolean xButtonCurrentlyPressed = false;
//
//            m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
//
//            if (m_colorSensor instanceof SwitchableLight)
//            {
//                ((SwitchableLight)m_colorSensor).enableLight(true);
//            }
//
//        if (this.m_colorSensorSubsystem.atTargetGreen(0.003, 0.010, 0.007))
//        {
//            this.m_lightSubsystem.on(0.55);
//        }
//        else if (this.m_colorSensorSubsystem.atTargetPurple(0.012, 0.013, 0.024))
//        {
//            this.m_lightSubsystem.on(0.722);
//        }
//
//
////        public boolean atTarget(double targetRed, double targetGreen, double targetBlue, double targetAlpha)
////    {
////        return this.m_colorSensor.red() >= targetRed-0.1 && this.m_colorSensor.red() <= targetRed+0.1 &&
////               this.m_colorSensor.green() >= targetGreen-0.1 && this.m_colorSensor.green() <= targetGreen+0.1 &&
////               this.m_colorSensor.blue() >= targetBlue-0.1 && this.m_colorSensor.blue() <= targetBlue+0.1 &&
////               this.m_colorSensor.alpha() >= targetAlpha-0.1 && this.m_colorSensor.alpha() <= targetAlpha+0.1;
//
////    }
////        if (this.m_colorSensorSubsystem.atTargetGreen(0.003, 0.010, 0.007))
////        {
////            this.m_lightSubsystem.on(0.55);
////        }
////        else if (this.m_colorSensorSubsystem.atTargetPurple(0.012, 0.013, 0.024))
////        {
////            this.m_lightSubsystem.on(0.722);
////        }
//    }
//
//    @Override
//    public boolean isFinished()
//    {
//        return true;
//    }
//}
