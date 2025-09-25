#include "IntanRHS2116.h"

#include <iostream>
#include <thread>
#include <chrono>

namespace {
    std::string withNewline(std::string cmd) {
        if (cmd.empty() || cmd.back() != '\n') {
            cmd.push_back('\n');
        }
        return cmd;
    }
}

// ====== Ctor ================================================================
IntanRHS2116::IntanRHS2116(ofSerial& serial)
: serial_(serial) {}

// ====== Private: sendCommand ===============================================
bool IntanRHS2116::sendCommand(const std::string& command) {
    if (!serial_.isInitialized()) {
        std::cerr << "[IntanRHS2116] ERROR: ofSerial is not initialized/open.\n";
        return false;
    }

    const std::string cmd = withNewline(command);

    // Write bytes to the serial port using the available API.
    const long written = serial_.writeData(cmd);
    if (written < 0 || static_cast<size_t>(written) != cmd.size()) {
        std::cerr << "[IntanRHS2116] ERROR: Failed to write full command. "
                  << "Written=" << written << " / " << cmd.size()
                  << " | Cmd=\"";
        for (char c : cmd) {
            if (c == '\n') std::cerr << "\\n";
            else std::cerr << c;
        }
        std::cerr << "\"\n";
        return false;
    }

    // Optional guard time to avoid overrunning the device parser.
    std::this_thread::sleep_for(std::chrono::milliseconds(kWriteDelayMs));
    return true;
}

// ====== Public API: configuration ==========================================
void IntanRHS2116::setSampleRate(int sampleRate) {
    sendCommand("SAMPLERATE:" + std::to_string(sampleRate));
    std::cout << "[STM32-RHS2116] Command sent: SAMPLERATE with value " << sampleRate << "\n";
}

void IntanRHS2116::setLowerBandwidth(double lowerBandwidthHz) {
    sendCommand("LOWERBANDWIDTH:" + std::to_string(lowerBandwidthHz));
    std::cout << "[STM32-RHS2116] Command sent: LOWERBANDWIDTH with value "
              << lowerBandwidthHz << "\n";
}

void IntanRHS2116::setUpperBandwidth(double upperBandwidthHz) {
    sendCommand("UPPERBANDWIDTH:" + std::to_string(upperBandwidthHz));
    std::cout << "[STM32-RHS2116] Command sent: UPPERBANDWIDTH with value "
              << upperBandwidthHz << "\n";
}

void IntanRHS2116::configure() {
    sendCommand("CONFIG");
    std::cout << "[STM32-RHS2116] Command sent: CONFIG\n";
}

// ====== Public API: acquisition ============================================
void IntanRHS2116::startAcquisition() {
    sendCommand("START");
    std::cout << "[STM32-RHS2116] Command sent: START\n";
}

void IntanRHS2116::stopAcquisition() {
    sendCommand("STOP");
    std::cout << "[STM32-RHS2116] Command sent: STOP\n";
}

void IntanRHS2116::reset() {
    sendCommand("RESET");
    std::cout << "[STM32-RHS2116] Command sent: RESET\n";
}

// ====== Public API: stimulation & timing ===================================
void IntanRHS2116::stim(int mode, int channel1, int channel2) {
    sendCommand("STIM:" + std::to_string(mode) + "," +
                std::to_string(channel1) + "," +
                std::to_string(channel2));
    std::cout << "[STM32-RHS2116] Command sent: STIM with mode " << mode
              << ", channel1 " << channel1
              << ", channel2 " << channel2 << "\n";
}

void IntanRHS2116::setNumberOfClkNeg(int clkNeg) {
    sendCommand("CLK_NEG:" + std::to_string(clkNeg));
    std::cout << "[STM32-RHS2116] Command sent: CLK_NEG with " << clkNeg << "\n";
}

void IntanRHS2116::setNumberOfClkPos(int clkPos) {
    sendCommand("CLK_POS:" + std::to_string(clkPos));
    std::cout << "[STM32-RHS2116] Command sent: CLK_POS with " << clkPos << "\n";
}

void IntanRHS2116::setStimPolarity(int stimPol) {
    sendCommand("STIM_POL:" + std::to_string(stimPol));
    std::cout << "[STM32-RHS2116] Command sent: STIM_POL with " << stimPol << "\n";
}

void IntanRHS2116::setStimType(int stimType) {
    sendCommand("STIM_TYPE:" + std::to_string(stimType));
    std::cout << "[STM32-RHS2116] Command sent: STIM_TYPE with " << stimType << "\n";
}

void IntanRHS2116::setVoltage(double voltage) {
    sendCommand("VOLTAGE:" + std::to_string(voltage));
    std::cout << "[STM32-RHS2116] Command sent: VOLTAGE with " << voltage << "V\n";
}

void IntanRHS2116::setStepSize(int stepSizeNa) {
    sendCommand("STEP_SIZE:" + std::to_string(stepSizeNa));
    std::cout << "[STM32-RHS2116] Command sent: STEP_SIZE with " << stepSizeNa << " nA\n";
}

void IntanRHS2116::setNegStimCurrent(int negStimCurrent) {
    sendCommand("NEG_CURRENT:" + std::to_string(negStimCurrent));
    std::cout << "[STM32-RHS2116] Command sent: NEG_CURRENT with " << negStimCurrent << "\n";
}

void IntanRHS2116::setPosStimCurrent(int posStimCurrent) {
    sendCommand("POS_CURRENT:" + std::to_string(posStimCurrent));
    std::cout << "[STM32-RHS2116] Command sent: POS_CURRENT with " << posStimCurrent << "\n";
}

// Note: spelling "CONTINOUS_STIM" kept to match original protocol string.
void IntanRHS2116::setContinuousStim(int mode) {
    sendCommand("CONTINOUS_STIM:" + std::to_string(mode));
    std::cout << "[STM32-RHS2116] Command sent: CONTINOUS_STIM with " << mode << "\n";
}

void IntanRHS2116::setNumberOfClkCR(int clkCR) {
    sendCommand("CLK_CR:" + std::to_string(clkCR));
    std::cout << "[STM32-RHS2116] Command sent: CLK_CR with " << clkCR << "\n";
}

void IntanRHS2116::setStateCR(int stateCR) {
    sendCommand("STATE_CR:" + std::to_string(stateCR));
    std::cout << "[STM32-RHS2116] Command sent: STATE_CR with " << stateCR << "\n";
}

// ====== Public API: DSP =====================================================
void IntanRHS2116::setDspEnable(bool enable) {
    sendCommand(std::string("DSP_EN:") + (enable ? "1" : "0"));
    std::cout << "[STM32-RHS2116] Command sent: DSP_EN with value " << (enable ? 1 : 0) << "\n";
}

void IntanRHS2116::setDspFrequency(int kFactor) {
    sendCommand("DSP_FREQ:" + std::to_string(kFactor));
    std::cout << "[STM32-RHS2116] Command sent: DSP_FREQ with k value " << kFactor << "\n";
}
