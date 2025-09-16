#include "IntanRHS2116.h"
#include <thread>
#include <sstream>
#include <iomanip>


IntanRHS2116::IntanRHS2116(WriteFn writer, unsigned interCommandDelayMs)
    : writer_(std::move(writer)), interDelayMs_(interCommandDelayMs)
{
}

void IntanRHS2116::setLogger(LogFn logger)
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    logger_ = std::move(logger);
}

void IntanRHS2116::setInterCommandDelay(unsigned ms)
{
    std::lock_guard<std::mutex> lock(ioMutex_);
    interDelayMs_ = ms;
}

bool IntanRHS2116::set_sample_rate(int sample_rate)
{
    std::ostringstream oss;
    oss << "SAMPLERATE:" << sample_rate;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: SAMPLERATE with value " + std::to_string(sample_rate));
    return ok;
}

bool IntanRHS2116::set_lower_bandwidth(float lower_bandwidth)
{
    std::ostringstream oss;
    oss << "LOWERBANDWIDTH:" << std::fixed << std::setprecision(3) << lower_bandwidth;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: LOWERBANDWIDTH with value " + std::to_string(lower_bandwidth));
    return ok;
}

bool IntanRHS2116::set_upper_bandwidth(float upper_bandwidth)
{
    std::ostringstream oss;
    oss << "UPPERBANDWIDTH:" << std::fixed << std::setprecision(3) << upper_bandwidth;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: UPPERBANDWIDTH with value " + std::to_string(upper_bandwidth));
    return ok;
}

bool IntanRHS2116::configure()
{
    bool ok = sendCommand("CONFIG");
    if (ok) log("Command sent: CONFIG");
    return ok;
}

bool IntanRHS2116::start_acquisition()
{
    bool ok = sendCommand("START");
    if (ok) log("Command sent: START");
    return ok;
}

bool IntanRHS2116::stop_acquisition()
{
    bool ok = sendCommand("STOP");
    if (ok) log("Command sent: STOP");
    return ok;
}

bool IntanRHS2116::reset()
{
    bool ok = sendCommand("RESET");
    if (ok) log("Command sent: RESET");
    return ok;
}

bool IntanRHS2116::stim(const std::string& mode, int channel1, int channel2)
{
    std::ostringstream oss;
    oss << "STIM:" << mode << "," << channel1 << "," << channel2;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: STIM with mode " + mode +
                ", channel1 " + std::to_string(channel1) +
                ", channel2 " + std::to_string(channel2));
    return ok;
}

bool IntanRHS2116::set_number_of_clk_neg(int clk_neg)
{
    std::ostringstream oss;
    oss << "CLK_NEG:" << clk_neg;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: CLK_NEG with " + std::to_string(clk_neg));
    return ok;
}

bool IntanRHS2116::set_number_of_clk_pos(int clk_pos)
{
    std::ostringstream oss;
    oss << "CLK_POS:" << clk_pos;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: CLK_POS with " + std::to_string(clk_pos));
    return ok;
}

bool IntanRHS2116::set_stim_polarity(const std::string& stim_pol)
{
    std::ostringstream oss;
    oss << "STIM_POL:" << stim_pol;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: STIM_POL with " + stim_pol);
    return ok;
}

bool IntanRHS2116::set_stim_type(const std::string& stim_type)
{
    std::ostringstream oss;
    oss << "STIM_TYPE:" << stim_type;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: STIM_TYPE with " + stim_type);
    return ok;
}

bool IntanRHS2116::set_voltage(float voltage)
{
    std::ostringstream oss;
    oss << "VOLTAGE:" << std::fixed << std::setprecision(3) << voltage;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: VOLTAGE with " + std::to_string(voltage) + "V");
    return ok;
}

bool IntanRHS2116::set_step_size(float step_size)
{
    std::ostringstream oss;
    oss << "STEP_SIZE:" << std::fixed << std::setprecision(3) << step_size;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: STEP_SIZE with " + std::to_string(step_size) + " nA");
    return ok;
}

bool IntanRHS2116::set_neg_stim_current(float neg_stim_current)
{
    std::ostringstream oss;
    oss << "NEG_CURRENT:" << std::fixed << std::setprecision(3) << neg_stim_current;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: NEG_CURRENT with " + std::to_string(neg_stim_current));
    return ok;
}

bool IntanRHS2116::set_pos_stim_current(float pos_stim_current)
{
    std::ostringstream oss;
    oss << "POS_CURRENT:" << std::fixed << std::setprecision(3) << pos_stim_current;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: POS_CURRENT with " + std::to_string(pos_stim_current));
    return ok;
}

bool IntanRHS2116::set_continuous_stim(int mode)
{
    // NOTE: Command name preserved as in Python: "CONTINOUS_STIM"
    std::ostringstream oss;
    oss << "CONTINOUS_STIM:" << mode;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: CONTINOUS_STIM with " + std::to_string(mode));
    return ok;
}

bool IntanRHS2116::set_number_of_clk_CR(int clk_CR)
{
    std::ostringstream oss;
    oss << "CLK_CR:" << clk_CR;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: CLK_CR with " + std::to_string(clk_CR));
    return ok;
}

bool IntanRHS2116::set_state_CR(int state_CR)
{
    std::ostringstream oss;
    oss << "STATE_CR:" << state_CR;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: STATE_CR with " + std::to_string(state_CR));
    return ok;
}

bool IntanRHS2116::set_dsp_enable(int enable)
{
    std::ostringstream oss;
    oss << "DSP_EN:" << enable;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: DSP_EN with value " + std::to_string(enable));
    return ok;
}

bool IntanRHS2116::set_dsp_frequency(float frequency)
{
    std::ostringstream oss;
    oss << "DSP_FREQ:" << std::fixed << std::setprecision(3) << frequency;
    bool ok = sendCommand(oss.str());
    if (ok) log("Command sent: DSP_FREQ with value " + std::to_string(frequency));
    return ok;
}

// -------------------- internals --------------------

bool IntanRHS2116::sendCommand(const std::string& command, bool appendNewline)
{
    std::lock_guard<std::mutex> lock(ioMutex_);

    std::string line = command;
    if (appendNewline)
    {
        if (line.empty() || line.back() != '\n')
            line.push_back('\n');
    }

    const auto* bytes = reinterpret_cast<const uint8_t*>(line.data());
    const std::size_t size = line.size();

    bool ok = false;
    try
    {
        ok = writer_ ? writer_(bytes, size) : false;
    }
    catch (...)
    {
        ok = false;
    }

    // Always wait a bit between commands, like the Python version.
    sleepMs(interDelayMs_);

    return ok;
}

void IntanRHS2116::log(const std::string& msg)
{
    if (logger_)
        logger_(msg);
}

void IntanRHS2116::sleepMs(unsigned ms)
{
    if (ms == 0u) return;
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}
