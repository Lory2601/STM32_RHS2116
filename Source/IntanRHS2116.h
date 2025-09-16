#pragma once
#include <string>
#include <functional>
#include <mutex>
#include <cstddef>
#include <cstdint>

/**
 * @brief Intan RHS2116 minimal command API (C++).
 *
 * This class mirrors the Python API you provided, emitting the same ASCII commands over
 * a user-supplied write function (e.g., a serial port writer).
 *
 * Integration notes for Open Ephys:
 *  - Provide a WriteFn that writes bytes to your serial backend from within a Processor plugin.
 *  - Call methods from your Processor's thread or a dedicated device thread.
 *  - Use setLogger() to route messages to Open Ephys log/console if desired.
 */
class IntanRHS2116
{
public:
    /// Function that must synchronously write all bytes and return true on success.
    using WriteFn = std::function<bool(const uint8_t* data, std::size_t size)>;

    /// Optional logger callback (e.g., to Open Ephys console).
    using LogFn   = std::function<void(const std::string& msg)>;

    /**
     * @brief Construct the driver with a write function.
     * @param writer   Synchronous byte writer (e.g., serial port write). Must be valid for the object's lifetime.
     * @param interCommandDelayMs  Delay after each command to give the device time to process (default 100 ms).
     */
    explicit IntanRHS2116(WriteFn writer, unsigned interCommandDelayMs = 100u);

    // Non-copyable / movable
    IntanRHS2116(const IntanRHS2116&) = delete;
    IntanRHS2116& operator=(const IntanRHS2116&) = delete;
    IntanRHS2116(IntanRHS2116&&) = delete;
    IntanRHS2116& operator=(IntanRHS2116&&) = delete;

    /**
     * @brief Set optional logger.
     */
    void setLogger(LogFn logger);

    /**
     * @brief Adjust inter-command delay (ms).
     */
    void setInterCommandDelay(unsigned ms);

    // ------- API methods (1:1 with the original Python names/behavior) -------
    bool set_sample_rate(int sample_rate);
    bool set_lower_bandwidth(float lower_bandwidth);
    bool set_upper_bandwidth(float upper_bandwidth);

    bool configure();
    bool start_acquisition();
    bool stop_acquisition();
    bool reset();

    bool stim(const std::string& mode, int channel1, int channel2);

    bool set_number_of_clk_neg(int clk_neg);
    bool set_number_of_clk_pos(int clk_pos);

    bool set_stim_polarity(const std::string& stim_pol);
    bool set_stim_type(const std::string& stim_type);

    bool set_voltage(float voltage);
    bool set_step_size(float step_size);

    bool set_neg_stim_current(float neg_stim_current);
    bool set_pos_stim_current(float pos_stim_current);

    // NOTE: The original Python command name is "CONTINOUS_STIM" (typo preserved intentionally).
    bool set_continuous_stim(int mode);

    bool set_number_of_clk_CR(int clk_CR);
    bool set_state_CR(int state_CR);

    bool set_dsp_enable(int enable);
    bool set_dsp_frequency(float frequency);

private:
    bool sendCommand(const std::string& command, bool appendNewline = true);
    void log(const std::string& msg);
    void sleepMs(unsigned ms);

    WriteFn writer_;
    LogFn   logger_{};
    unsigned interDelayMs_{100u};
    std::mutex ioMutex_;
};
