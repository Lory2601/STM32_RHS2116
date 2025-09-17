#pragma once

// As requested
#include "ofSerial.h"

#include <string>

/**
 * @brief Thin wrapper for sending ASCII commands to the Intan RHS2116 over an already-open ofSerial.
 *
 * This class DOES NOT open or close the serial port. It only writes commands.
 * The caller is responsible for creating and configuring the ofSerial instance.
 *
 * Example usage (opening done elsewhere):
 *   ofSerial serial;
 *   serial.setup("COM4", 115200); // Open outside this module
 *   IntanRHS2116 rhs(serial);
 *   rhs.setSampleRate(20000);
 *   rhs.configure();
 *   rhs.startAcquisition();
 */
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

    // Note: keeping the original protocol string spelling ("CONTINOUS_STIM")
    void setContinuousStim(int mode);

    void setNumberOfClkCR(int clkCR);
    void setStateCR(int stateCR);

    // DSP controls
    void setDspEnable(bool enable);
    void setDspFrequency(double frequencyHz);

private:
    ofSerial& serial_;
    static constexpr unsigned int kWriteDelayMs = 100; // inter-command guard time

    // Low-level helper to send a single ASCII command with trailing newline if not present.
    // Logs to std::cout / std::cerr. Returns true on success.
    bool sendCommand(const std::string& command);
};
