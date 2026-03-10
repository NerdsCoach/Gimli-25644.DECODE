//package teamCode.commands;
//
//import android.graphics.Color;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import teamCode.subsystems.ColorSensorSubsystem;
//import teamCode.subsystems.LightSubsystem;
//import teamCode.subsystems.PrismLEDSubsystem;
//
//public class ColorModeOnLEDCommand extends CommandBase
//{
//    private final ColorSensorSubsystem m_colorSubsystem;
//    private final PrismLEDSubsystem m_prismLightSubsystem;
//    private static final float TARGET_GREEN_HUE = 160.0f;
//    private static final float TARGET_PURPLE_HUE = 240.0f;
//    private static final float HUE_TOLERANCE = 10.0f; // Allow +/- 10 degrees variance
//
//    public double m_lastKnownColor;
//    private int m_position;
//    private static final int  m_off = 1;
//    private static final int  m_on = 0;
//    private final ElapsedTime m_gapTimer = new ElapsedTime();
//    private double m_lastSeenTime = 0;
//
//    public ColorModeOnLEDCommand(ColorSensorSubsystem colorSubsystem, PrismLEDSubsystem prismLEDSubsystem)
//    {
//        this.m_colorSubsystem = colorSubsystem;
//        this.m_prismLightSubsystem = prismLEDSubsystem;
//
//        addRequirements(m_colorSubsystem, m_prismLightSubsystem);
//    }
//    @Override
//    public void initialize()
//    {
//        m_gapTimer.reset();
//        m_lastSeenTime = m_gapTimer.seconds();
//    }
//
//
//    @Override
//    public void execute() {
//        NormalizedRGBA colors = m_colorSubsystem.getNormalizedColors();
//        float[] hsv = new float[3];
//        Color.RGBToHSV((int) (colors.red * 255), (int) (colors.green * 255), (int) (colors.blue * 255), hsv);
//
//        float hue = hsv[0];
//        float saturation = hsv[1];
//
//        if (saturation > 0.3) {
//            // We see a piece: Keep the timer updated
//            m_lastSeenTime = m_gapTimer.seconds();
//
//            if (Math.abs(hue - TARGET_GREEN_HUE) < HUE_TOLERANCE) {
//                m_prismLightSubsystem.setGreen();
//            } else if (Math.abs(hue - TARGET_PURPLE_HUE) < HUE_TOLERANCE) {
//                m_prismLightSubsystem.setPurple();
//            }
//        }
//        else {
//            // No piece: Check if we should clear the lights
//            if (m_gapTimer.seconds() - m_lastSeenTime > 1.0) {
//                // Set to "Searching" (Dim White)
//                m_prismLightSubsystem.lastKnown(10, 10, 10);
//            }
//        }
//    }
//
//
//    @Override
//    public void end(boolean interrupted)
//    {
//    }
//
//    @Override
//    public boolean isFinished()
//    {
//        return false;
//    }
//}
