//import com.arcrobotics.ftclib.command.CommandOpMode;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//
////package teamCode;
////
////import android.app.Activity;
////import android.graphics.Color;
////import android.view.View;
////
////import com.arcrobotics.ftclib.command.CommandOpMode;
////import com.arcrobotics.ftclib.command.button.Button;
////import com.arcrobotics.ftclib.command.button.GamepadButton;
////import com.arcrobotics.ftclib.gamepad.GamepadEx;
////import com.arcrobotics.ftclib.gamepad.GamepadKeys;
////import com.qualcomm.hardware.rev.RevColorSensorV3;
////import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
////import com.qualcomm.robotcore.hardware.DistanceSensor;
////import com.qualcomm.robotcore.hardware.HardwareMap;
////import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
////import com.qualcomm.robotcore.hardware.NormalizedRGBA;
////import com.qualcomm.robotcore.hardware.SwitchableLight;
////import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
////
////import teamCode.commands.ColorModeOnCommand;
////import teamCode.subsystems.ColorSensorSubsystem;
////import teamCode.subsystems.LightSubsystem;
////
////@TeleOp(name = "Sensor: Color")
//public class ColorTest extends CommandOpMode
//{
//    private NormalizedColorSensor m_colorSensor;
////
////    private ColorSensorSubsystem m_colorSensorSubsystem;
////    private ColorModeOnCommand m_colorCommand;
////
////    @Override
////    public void initialize(HardwareMap hardwareMap)
////    {
////        m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
////    }
////
////    public enum DetectedColor
////    {
////        GREEN,
////        PURPLE,
////        UNKNOWN
////    }
////
////
//    public void initialize (HardwareMap hardwareMap)
//    {
//        m_colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
//    }
////
////    public DetectedColor getDetectedColor ()
////    {
////        NormalizedRGBA colors = m_colorSensor.getNormalizedColors(); // returns 4 values Alpha is how much light is being returned
////        float normRed;
////        float normGreen;
////        float normBlue;
////        normRed = colors.red / colors.alpha;
////        normGreen = colors.green / colors.alpha;
////        normBlue = colors.blue / colors.alpha; // so you always get the same reading at different distances
////
////        telemetry.addData("red", normRed);
////        telemetry.addData("green", normGreen);
////        telemetry.addData("blue", normBlue);
////
////        //TODO Add if statements for each color
////
////        return DetectedColor.UNKNOWN;
////
////
////
////    }
//    private RevColorSensorV3 m_colorSensor;
//    private LightSubsystem m_lightSubsystem;
//    private GamepadEx m_driver2;
//    private Button m_circle;
////
////
////    View relativeLayout;
////
//    @Override public void initialize()
//    {
//        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//
//        this.m_driver2 = new GamepadEx(gamepad2);
//
//        try
//        {
//            runSample(); // actually execute the sample
//        } finally
//        {
//            relativeLayout.post(new Runnable()
//            {
//                public void run()
//                {
//                    relativeLayout.setBackgroundColor(Color.WHITE);
//                }
//            });
//        }
//    }
//
//    protected void runSample()
//    {
//        float gain = 2;
//
//        final float[] hsvValues = new float[3];
//
//        boolean xButtonPreviouslyPressed = false;
//        boolean xButtonCurrentlyPressed = false;
//
//        m_colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
//
//        if (m_colorSensor instanceof SwitchableLight)
//        {
//            ((SwitchableLight)m_colorSensor).enableLight(true);
//        }
//
//        // Wait for the start button to be pressed.
//        waitForStart();
//
//        // Loop until we are asked to stop
////        while (opModeIsActive())
////        {
//            // Explain basic gain information via telemetry
//            telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
//            telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");
//
//            // Update the gain value if either of the A or B gamepad buttons is being held
////            if (gamepad2.b)
////            {
////                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
//                gain += 0.005;
////                this.m_colorCommand = new ColorModeOnCommand(this.m_colorSensorSubsystem);
////            }
////        this.m_colorCommand = new ColorModeOnCommand(this.m_colorSensorSubsystem, this.m_lightSubsystem);
////        this.m_circle = (new GamepadButton(this.m_driver2, GamepadKeys.Button.B))
////                .whenPressed(this.m_colorCommand);
//
//            // Show the gain value via telemetry
//            telemetry.addData("Gain", gain);
//
//            m_colorSensor.setGain(gain);
//
//            // Check the status of the X button on the gamepad
//            xButtonCurrentlyPressed = gamepad1.x;
//
//            // If the button state is different than what it was, then act
//            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed)
//            {
//                // If the button is (now) down, then toggle the light
//                if (xButtonCurrentlyPressed)
//                {
//                    if (m_colorSensor instanceof SwitchableLight)
//                    {
//                        SwitchableLight light = (SwitchableLight)m_colorSensor;
//                        light.enableLight(!light.isLightOn());
//                    }
//                }
//            }
//            xButtonPreviouslyPressed = xButtonCurrentlyPressed;
//
//            // Get the normalized colors from the sensor
//            NormalizedRGBA colors = m_colorSensor.getNormalizedColors();
//
//            Color.colorToHSV(colors.toColor(), hsvValues);
//
//            telemetry.addLine()
//                    .addData("Red", "%.3f", colors.red)
//                    .addData("Green", "%.3f", colors.green)
//                    .addData("Blue", "%.3f", colors.blue);
//            telemetry.addLine()
//                    .addData("Hue", "%.3f", hsvValues[0])
//                    .addData("Saturation", "%.3f", hsvValues[1])
//                    .addData("Value", "%.3f", hsvValues[2]);
//            telemetry.addData("Alpha", "%.3f", colors.alpha);
//
//            if (m_colorSensor instanceof DistanceSensor)
//            {
//                telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) m_colorSensor).getDistance(DistanceUnit.CM));
//            }
//
//            telemetry.update();
//
//            if (m_colorSensor.red() == 0.003 && m_colorSensor.green() == 0.010 && m_colorSensor.blue() == 0.007 )
//            {
//                this.m_lightSubsystem.on(0.5);
//            }
//            else if (m_colorSensor.red() == 0.012 && m_colorSensor.green() == 0.013 && m_colorSensor.blue() == 0.024)
//            {
//                this.m_lightSubsystem.on(0.722);
//            }
//
//        }
//    }
////}
////
////
