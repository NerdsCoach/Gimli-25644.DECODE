package teamCode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

import teamCode.PrismLEDDriver;



public class PrismLEDSubsystem extends SubsystemBase
 {
    private final PrismLEDDriver prism;
     public PrismLEDSubsystem(HardwareMap hMap, String prismLED)
     {
         Object device = hMap.get(prismLED);
         I2cDeviceSynch i2cClient = ((I2cDeviceSynchDevice<I2cDeviceSynch>) device).getDeviceClient();
         this.prism = new PrismLEDDriver(i2cClient);

         // 1. Force the Prism into Manual RGB mode (Register 0x01 = 0)
         this.prism.setAnimation(0);

         // 2. Set the LEDs to BRIGHT WHITE (Maximum power test)
         this.prism.setRGB(255, 255, 255);
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





