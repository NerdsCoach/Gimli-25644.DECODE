package teamCode;


import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;

public class PrismLEDDriver extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    private static final int REG_RED = 0x03;
    private static final int REG_GREEN = 0x04;
    private static final int REG_BLUE = 0x05;
    // goBILDA Prism default 7-bit address is 0x38
    public final I2cAddr ADDRESS = I2cAddr.create7bit(0x38);

    public PrismLEDDriver(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);
        // This line replaces the @I2cAddrConfig annotation
        this.deviceClient.setI2cAddress(I2cAddr.create7bit(0x38));
        this.deviceClient.engage();
//        this.deviceClient.setI2cAddress(ADDRESS);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

     public void setRGB(int r, int g, int b)
     {
            deviceClient.write8(REG_RED, r);
            deviceClient.write8(REG_GREEN, g);
            deviceClient.write8(REG_BLUE, b);
     }



    // Example register: 0x01 is often used for Artboard/Animation selection
    public void setAnimation(int id)
    {
        deviceClient.write8(0x01, id);
    }

    @Override protected boolean doInitialize()
    { return true; }
    @Override public Manufacturer getManufacturer()
    {return Manufacturer.Other; }
    @Override public String getDeviceName()
    { return "goBILDA Prism LED Driver"; }
}
