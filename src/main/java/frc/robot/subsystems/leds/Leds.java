package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.fasterxml.jackson.annotation.JsonFormat.Feature;

public class Leds {

    private static Leds instance;
    public final int ledPort = 0;
    public final int ledStart = 0;
    public final int ledEnd = 7;

    public CANdle candle = new CANdle(0);

    public Leds(){
        LedConfigs();
    }

    public void LedConfigs(){
        LEDConfigs configs = new LEDConfigs();
        CANdleFeaturesConfigs featuresConfigs = new CANdleFeaturesConfigs();
        configs.BrightnessScalar = LedConstants.brightnessScaler;
        featuresConfigs.VBatOutputMode = VBatOutputModeValue.On;
        candle.getConfigurator().apply(configs);
        candle.getConfigurator().apply(featuresConfigs);
    }
}
