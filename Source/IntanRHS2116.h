/*
 * Project : BRAIN PLUS
 * File    : IntanRHS2116.h
 * Author  : Clerici Lorenzo (ISEA)
 * Created : 2025-09-19
 * Purpose : Thin wrapper for sending ASCII commands to the Intan RHS2116
 *
 * Data-acquisition plugin for the Intan RHS2116 over a serial link.
 * Responsibilities:
 *   - Configure device (rate, bandwidth, DSP) and manage start/stop.
 *   - Serial I/O thread: sync on 0xAA, read fixed-size frames, enqueue.
 *   - Packet pool queue for lock-efficient producer/consumer flow.
 *   - Parse/de-interleave samples, convert to µV, publish to DataBuffer.
 *   - Expose Open Ephys/JUCE DataThread interface and editor stub.
 */



#pragma once
#include "ofSerial.h"
#include <string>



class IntanRHS2116 {
public:
    explicit IntanRHS2116(ofSerial& serial);

    // Configuration commands
    void setSampleRate(int sampleRate); //ok
    void setLowerBandwidth(double lowerBandwidthHz); //ok
    void setUpperBandwidth(double upperBandwidthHz); //ok
    void configure();

    // Acquisition control
    void startAcquisition();
    void stopAcquisition();
    void reset();

    // Stimulation-related commands
    void stim(int mode, int channel1, int channel2); //ok
    void setNumberOfClkNeg(int clkNeg); //ok
    void setNumberOfClkPos(int clkPos); //ok
    void setStimPolarity(int stimPol); //ok
    void setStimType(int stimType); //ok
    void setVoltage(double voltage); //ok
    void setStepSize(int stepSizeNa); //ok
    void setNegStimCurrent(int negStimCurrent); //ok
    void setPosStimCurrent(int posStimCurrent); //ok
    void setContinuousStim(int mode); //ok
    void setNumberOfClkCR(int clkCR); //ok
    void setStateCR(int stateCR); //ok

    // DSP controls
    void setDspEnable(bool enable); //ok
    void setDspFrequency(int kFactor); //ok

private:
    ofSerial& serial_;
    static constexpr unsigned int kWriteDelayMs = 100; // inter-command guard time
    bool sendCommand(const std::string& command);
};
