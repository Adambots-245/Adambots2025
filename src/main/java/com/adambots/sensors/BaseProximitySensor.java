package com.adambots.sensors;

public interface BaseProximitySensor {

    /**
     * Returns the detection of the photoeye with regards to whether or not the PhotoEye is defined as inverted
     *
     * @return Whether or not the PhotoEye is detecting something
     */
    boolean isDetecting();

}