package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import teamCode.PrismLEDDriver;

public class PrismLEDSubsystem extends SubsystemBase
 {
    private final PrismLEDDriver prism;

    public PrismLEDSubsystem(HardwareMap hMap, String PrismLED)
    {
        // Get the generic I2C client from the hub
        I2cDeviceSynch i2cClient = hMap.get(I2cDeviceSynch.class, "prismLED");

        // Wrap it in your custom Prism driver
        this.prism = new PrismLEDDriver(i2cClient);
    }
     public void setGreen()
     {
         prism.setRGB(0, 255, 0);
     }
     public void setPurple()
     {
         prism.setRGB(255, 0, 255);
     }
     public void lastKnown (int r, int g, int b)
     {
         prism.setRGB(r, g, b);
     }
}





