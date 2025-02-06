// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.LEDConstants;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleSubsystem extends SubsystemBase {
  private static final int LEDS_IN_STRIP = LEDConstants.LEDS_IN_STRIP;
  private CANdle candle;
  private Animation animation;
  
  private int candleChannel;
  private double animateSpeed = 0.4;

  public CANdleSubsystem(CANdle candleDevice) {
    this.candle = candleDevice;

    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.disableWhenLOS = true;
    configAll.stripType = LEDStripType.GRB; // the BTF-Lighting LED strip uses GRB format
    configAll.brightnessScalar = 1;
    configAll.vBatOutputMode = VBatOutputMode.Off;
    configAll.v5Enabled = true;
    
    candleDevice.configAllSettings(configAll);

    setColor(LEDConstants.yellow);
    setAnimation(AnimationTypes.Larson);
  }

  @Override
  public void periodic() { 
    if (animation != null) {  
      candle.animate(animation, candleChannel);
    }
  }

  public enum AnimationTypes {
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff,
    SetAll,
    Empty
  }

  public void setColor(Color color) {
    setColor((int)(color.red*255), (int)(color.green*255), (int)(color.blue*255));
  }

  /**
   * Set the colors for the LED Strip
   * 
   * @param r - Red (0 to 255)
   * @param g - Green (0 to 255)
   * @param b - Blue (0 to 255)
   */
  public void setColor(int r, int g, int b) {
    clearAllAnims();
    setAnimation(AnimationTypes.SetAll);

    r = MathUtil.clamp(r, 0, 255);
    g = MathUtil.clamp(g, 0, 255);
    b = MathUtil.clamp(b, 0, 255);

    candle.setLEDs(r, g, b);
  }

  /**
   * Set an individual LED to a specified color
   * @param r
   * @param g
   * @param b
   * @param startIdx - 0 based index for starting LED. The strip starts at 8.
   * @param numOfLEDs - Number of LEDs to light up with this color
   */
  public void setLEDs(int r, int g, int b, int startIdx, int numOfLEDs){
    r = MathUtil.clamp(r, 0, 255);
    g = MathUtil.clamp(g, 0, 255);
    b = MathUtil.clamp(b, 0, 255);
    
    if (startIdx < 0 || startIdx > LEDS_IN_STRIP)
      startIdx = 0;
    
    if (numOfLEDs < 0 || numOfLEDs > LEDS_IN_STRIP)
      numOfLEDs = LEDS_IN_STRIP;

    candle.setLEDs(r, g, b, 0, startIdx, numOfLEDs);
  }

  public void setAnimation(AnimationTypes toChange) {
    clearAllAnims();

    switch (toChange) {
      default:
      case ColorFlow:
        candleChannel = 0;
        animation = new ColorFlowAnimation(255, 150, 0, 0, 1, LEDS_IN_STRIP, Direction.Forward, 8);
        break;
      case Fire:
        candleChannel = 1;
        animation = new FireAnimation(0.5, 0.7, LEDS_IN_STRIP, 0.8, 0.5, false, 8);
        break;
      case Larson:
        candleChannel = 2;
        animation = new LarsonAnimation(255, 150, 0, 100, 0.001, LEDS_IN_STRIP, BounceMode.Front, 30, 0);
        break;
      case Rainbow:
        candleChannel = 3;
        animation = new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, false, 8);
        break;
      case RgbFade:
        candleChannel = 4;
        animation = new RgbFadeAnimation(0.7, 0.4, LEDS_IN_STRIP, 8);
        break;
      case SingleFade:
        candleChannel = 5;
        animation = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LEDS_IN_STRIP, 8);
        break;
      case Strobe:
        candleChannel = 6;
        animation = new StrobeAnimation(0, 0, 255, 0, 0, LEDS_IN_STRIP, 8);
        break;
      case Twinkle:
        candleChannel = 7;
        animation = new TwinkleAnimation(0, 255, 0, 0, 1, LEDS_IN_STRIP, TwinklePercent.Percent100, 0);
        break;
      case TwinkleOff:
        candleChannel = 8;
        animation = new TwinkleOffAnimation(70, 90, 175, 0, 0.2, LEDS_IN_STRIP, TwinkleOffPercent.Percent76, 8);
        break;
      case Empty:
        candleChannel = 9;
        animation = new RainbowAnimation(1, 0.7, LEDS_IN_STRIP, false, 8);
        break;
      case SetAll:
        animation = null;
        break;
    }

    if (animation != null) {
      animation.setSpeed(animateSpeed);
    }
  }

  public void clearAllAnims() {
    for(int i = 0; i < 10; ++i) {
      candle.clearAnimation(i);
    }
  }
}