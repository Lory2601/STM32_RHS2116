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
    void setSampleRate(int sampleRate);
    void setLowerBandwidth(double lowerBandwidthHz);
    void setUpperBandwidth(double upperBandwidthHz);
    void configure();

    // Acquisition control
    void startAcquisition();
    void stopAcquisition();
    void reset();

    // Stimulation-related commands
    void stim(int mode, int channel1, int channel2);
    void setNumberOfClkNeg(int clkNeg);
    void setNumberOfClkPos(int clkPos);
    void setStimPolarity(int stimPol);
    void setStimType(int stimType);
    void setVoltage(double voltage);
    void setStepSize(int stepSizeNa);
    void setNegStimCurrent(int negStimCurrent);
    void setPosStimCurrent(int posStimCurrent);
    void setContinuousStim(int mode);
    void setNumberOfClkCR(int clkCR);
    void setStateCR(int stateCR);

    // DSP controls
    void setDspEnable(bool enable);
    void setDspFrequency(int kFactor);

private:
    ofSerial& serial_;
    static constexpr unsigned int kWriteDelayMs = 100; // inter-command guard time
    bool sendCommand(const std::string& command);
};
