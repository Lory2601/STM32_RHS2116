/*
 * Project : BRAIN PLUS
 * File    : DataThreadPlugin.cpp
 * Author  : Clerici Lorenzo (ISEA)
 * Created : 2025-09-19
 * Purpose : Data acquisition plugin for Intan RHS2116 over a serial interface.
 *
 * Data-acquisition plugin for the Intan RHS2116 over a serial link.
 * Responsibilities:
 *   - Configure device (rate, bandwidth, DSP) and manage start/stop.
 *   - Serial I/O thread: sync on 0xAA, read fixed-size frames, enqueue.
 *   - Packet pool queue for lock-efficient producer/consumer flow.
 *   - Parse/de-interleave samples, convert to µV, publish to DataBuffer.
 *   - Expose Open Ephys/JUCE DataThread interface and editor stub.
 */


#include "DataThreadPlugin.h"
#include "DataThreadPluginEditor.h"
#include "ofSerial.h"
#include "IntanRHS2116.h"

#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>

#include <CoreServicesHeader.h>
#include "DataThreadPluginEditor.h"
#include <iostream>
#include <cstdio>
#include <EditorHeaders.h>
#include "DataThreadPlugin.h"
#include "ofSerial.h" 
#include <BasicJuceHeader.h> 

#include <filesystem>
namespace fs = std::filesystem;
static juce::File gBaseDir;
static juce::File gSeqFile;
static juce::var  gSeq; // array of preset filenames
static int        gSeqCount = 0;

// =================================================== Command line parser ======================================================
static String handleRhsCommand (IntanRHS2116* rhs, const String& msg)
{
    String cleanMsg = msg.trim();
    cleanMsg = cleanMsg.replaceCharacter (':', ' ');
    cleanMsg = cleanMsg.replaceCharacter (',', ' ');

    StringArray args;
    args.addTokens (cleanMsg, " \t\r\n", "");
    args.removeEmptyStrings();

    if (args.size() == 0)
        return "";

    const String cmd = args[0].toUpperCase();

    auto hasArgs = [&args](int n) { return args.size() == n + 1; };
    auto intArg  = [&args](int i) { return args[i].getIntValue(); };
    auto dblArg  = [&args](int i) { return args[i].getDoubleValue(); };

    const bool knownCommand = (cmd == "SAMPLERATE" || cmd == "LOWERBANDWIDTH" || cmd == "UPPERBANDWIDTH"
                               || cmd == "CONFIG" || cmd == "START" || cmd == "STOP" || cmd == "RESET"
                               || cmd == "STIM" || cmd == "CLK_NEG" || cmd == "CLK_POS" || cmd == "STIM_POL"
                               || cmd == "STIM_TYPE" || cmd == "VOLTAGE" || cmd == "STEP_SIZE"
                               || cmd == "NEG_CURRENT" || cmd == "POS_CURRENT" || cmd == "CONTINOUS_STIM"
                               || cmd == "CONTINUOUS_STIM" || cmd == "CLK_CR" || cmd == "STATE_CR"
                               || cmd == "DSP_EN" || cmd == "DSP_FREQ" || cmd == "TLCMAX" || cmd == "TLCPWM"
                               || cmd == "OPTCLK" || cmd == "OPTSTIM");

    if (! knownCommand)
        return "";

    if (rhs == nullptr)
        return "ERR: device not initialized";

    // Configuration commands
    if (cmd == "SAMPLERATE" && hasArgs (1))        { rhs->setSampleRate (intArg (1));       return "OK: SAMPLERATE sent"; }
    if (cmd == "LOWERBANDWIDTH" && hasArgs (1))    { rhs->setLowerBandwidth (dblArg (1));   return "OK: LOWERBANDWIDTH sent"; }
    if (cmd == "UPPERBANDWIDTH" && hasArgs (1))    { rhs->setUpperBandwidth (dblArg (1));   return "OK: UPPERBANDWIDTH sent"; }
    if (cmd == "CONFIG" && hasArgs (0))            { rhs->configure();                      return "OK: CONFIG sent"; }

    // Acquisition control
    if (cmd == "START" && hasArgs (0))             { rhs->startAcquisition();               return "OK: START sent"; }
    if (cmd == "STOP" && hasArgs (0))              { rhs->stopAcquisition();                return "OK: STOP sent"; }
    if (cmd == "RESET" && hasArgs (0))             { rhs->reset();                          return "OK: RESET sent"; }

    // Stimulation-related commands
    if (cmd == "STIM" && hasArgs (3))              { rhs->stim (intArg (1), intArg (2), intArg (3)); return "OK: STIM sent"; }
    if (cmd == "CLK_NEG" && hasArgs (1))           { rhs->setNumberOfClkNeg (intArg (1));   return "OK: CLK_NEG sent"; }
    if (cmd == "CLK_POS" && hasArgs (1))           { rhs->setNumberOfClkPos (intArg (1));   return "OK: CLK_POS sent"; }
    if (cmd == "STIM_POL" && hasArgs (1))          { rhs->setStimPolarity (intArg (1));     return "OK: STIM_POL sent"; }
    if (cmd == "STIM_TYPE" && hasArgs (1))         { rhs->setStimType (intArg (1));         return "OK: STIM_TYPE sent"; }
    if (cmd == "VOLTAGE" && hasArgs (1))           { rhs->setVoltage (dblArg (1));          return "OK: VOLTAGE sent"; }
    if (cmd == "STEP_SIZE" && hasArgs (1))         { rhs->setStepSize (intArg (1));         return "OK: STEP_SIZE sent"; }
    if (cmd == "NEG_CURRENT" && hasArgs (1))       { rhs->setNegStimCurrent (intArg (1));   return "OK: NEG_CURRENT sent"; }
    if (cmd == "POS_CURRENT" && hasArgs (1))       { rhs->setPosStimCurrent (intArg (1));   return "OK: POS_CURRENT sent"; }
    if ((cmd == "CONTINOUS_STIM" || cmd == "CONTINUOUS_STIM") && hasArgs (1))
                                                     { rhs->setContinuousStim (intArg (1)); return "OK: CONTINOUS_STIM sent"; }
    if (cmd == "CLK_CR" && hasArgs (1))            { rhs->setNumberOfClkCR (intArg (1));    return "OK: CLK_CR sent"; }
    if (cmd == "STATE_CR" && hasArgs (1))          { rhs->setStateCR (intArg (1));          return "OK: STATE_CR sent"; }

    // DSP controls
    if (cmd == "DSP_EN" && hasArgs (1))            { rhs->setDspEnable (intArg (1) != 0);   return "OK: DSP_EN sent"; }
    if (cmd == "DSP_FREQ" && hasArgs (1))          { rhs->setDspFrequency (intArg (1));     return "OK: DSP_FREQ sent"; }

    // TLC LED controls
    if (cmd == "TLCMAX" && hasArgs (0))            { rhs->setTlcAllOnMax();                 return "OK: TLCMAX sent"; }
    if (cmd == "TLCPWM" && hasArgs (1))
    {
        const int pwm = intArg (1);
        if (pwm < 0 || pwm > 255) return "ERR: TLCPWM range is 0-255";
        rhs->setTlcAllOnPwm (pwm);
        return "OK: TLCPWM sent";
    }

    // Optical stimulation controls
    if (cmd == "OPTCLK" && hasArgs (1))
    {
        const int clk = intArg (1);
        if (clk < 0 || clk > 65535) return "ERR: OPTCLK range is 0-65535";
        rhs->setOpticClk (static_cast<uint16_t> (clk));
        return "OK: OPTCLK sent";
    }

    if (cmd == "OPTSTIM" && hasArgs (1))
    {
        const int pattern = intArg (1);
        if (pattern < 0 || pattern > 255) return "ERR: OPTSTIM range is 0-255";
        rhs->optic_stim (static_cast<uint8_t> (pattern));
        return "OK: OPTSTIM sent";
    }

    return "";
}

// =================================================== PacketPoolQueue impl =====================================================
DataThreadPlugin::PacketPoolQueue::PacketPoolQueue(int n)
: storage_(static_cast<size_t>(n))
{
    for (int i = 0; i < n; ++i) free_.push_back(i);
}

int DataThreadPlugin::PacketPoolQueue::acquireFreeBlocking(std::atomic_bool& stopFlag) {
    std::unique_lock<std::mutex> lk(m_);
    cvFree_.wait(lk, [&] { return !free_.empty() || stopFlag.load(); });
    if (stopFlag.load() && free_.empty()) return -1;
    int idx = free_.front();
    free_.pop_front();
    return idx;
}

void DataThreadPlugin::PacketPoolQueue::pushReady(int idx) {
    { std::lock_guard<std::mutex> lk(m_); ready_.push_back(idx); }
    cvReady_.notify_one();
}

bool DataThreadPlugin::PacketPoolQueue::tryPopReady(int& idx) {
    std::lock_guard<std::mutex> lk(m_);
    if (ready_.empty()) return false;
    idx = ready_.front();
    ready_.pop_front();
    return true;
}

void DataThreadPlugin::PacketPoolQueue::releaseFree(int idx) {
    { std::lock_guard<std::mutex> lk(m_); free_.push_back(idx); }
    cvFree_.notify_one();
}

void DataThreadPlugin::PacketPoolQueue::reset() {
    std::lock_guard<std::mutex> lk(m_);
    ready_.clear();
    free_.clear();
    for (int i = 0; i < static_cast<int>(storage_.size()); ++i) free_.push_back(i);
}

int DataThreadPlugin::PacketPoolQueue::readySize() const {
    std::lock_guard<std::mutex> lk(m_);
    return static_cast<int>(ready_.size());
}
// ==============================================================================================================================



// ============================ EnvPoolQueue impl =============================
int DataThreadPlugin::EnvPoolQueue::acquireFreeBlocking(std::atomic_bool& stopFlag) {
    std::unique_lock<std::mutex> lk(m_);
    cvFree_.wait(lk, [&]{ return !free_.empty() || stopFlag.load(); });
    if (stopFlag.load() && free_.empty()) return -1;
    int idx = free_.front(); free_.pop_front(); return idx;
}
void DataThreadPlugin::EnvPoolQueue::pushReady(int idx) {
    { std::lock_guard<std::mutex> lk(m_); ready_.push_back(idx); } cvReady_.notify_one();
}
bool DataThreadPlugin::EnvPoolQueue::tryPopReady(int& idx) {
    std::lock_guard<std::mutex> lk(m_); if (ready_.empty()) return false; idx=ready_.front(); ready_.pop_front(); return true;
}
void DataThreadPlugin::EnvPoolQueue::releaseFree(int idx) {
    { std::lock_guard<std::mutex> lk(m_); free_.push_back(idx); } cvFree_.notify_one();
}
void DataThreadPlugin::EnvPoolQueue::reset() {
    std::lock_guard<std::mutex> lk(m_); ready_.clear(); free_.clear();
    for (int i=0;i<(int)storage_.size();++i) free_.push_back(i);
}

int DataThreadPlugin::SpadPoolQueue::acquireFreeBlocking(std::atomic_bool& stopFlag) {
    std::unique_lock<std::mutex> lk(m_);
    cvFree_.wait(lk, [&]{ return !free_.empty() || stopFlag.load(); });
    if (stopFlag.load() && free_.empty()) return -1;
    int idx = free_.front(); free_.pop_front(); return idx;
}
void DataThreadPlugin::SpadPoolQueue::pushReady(int idx) { { std::lock_guard<std::mutex> lk(m_); ready_.push_back(idx); } cvReady_.notify_one(); }
bool DataThreadPlugin::SpadPoolQueue::tryPopReady(int& idx) { std::lock_guard<std::mutex> lk(m_); if (ready_.empty()) return false; idx=ready_.front(); ready_.pop_front(); return true; }
void DataThreadPlugin::SpadPoolQueue::releaseFree(int idx) { { std::lock_guard<std::mutex> lk(m_); free_.push_back(idx); } cvFree_.notify_one(); }
void DataThreadPlugin::SpadPoolQueue::reset() { std::lock_guard<std::mutex> lk(m_); ready_.clear(); free_.clear(); for (int i=0;i<(int)storage_.size();++i) free_.push_back(i); }
// ==============================================================================================================================




// ================================================= DataThreadPluginEditor impl ================================================
struct PluginSettingsObject { /* reserved */ };

DataThreadPlugin::DataThreadPlugin (SourceNode* sn)
: DataThread (sn)
{
    queue_ = std::make_unique<PacketPoolQueue>(N_BLOCKS);
    envQueue_ = std::make_unique<EnvPoolQueue>(N_ENV_BLOCKS);
    spadQueue_ = std::make_unique<SpadPoolQueue>(N_SPAD_BLOCKS);
}

DataThreadPlugin::~DataThreadPlugin()
{
    serialRunning_.store(false);
    if (serialThread_.joinable()) serialThread_.join();
    sequenceRunning_.store(false);
    if (sequenceThread_.joinable()) sequenceThread_.join();
}

bool DataThreadPlugin::foundInputSource()
{
    return true;
}
// ==============================================================================================================================








// =================================================== update settings ==========================================================
void DataThreadPlugin::updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                                       OwnedArray<EventChannel>* eventChannels,
                                       OwnedArray<SpikeChannel>* /*spikeChannels*/,
                                       OwnedArray<DataStream>* sourceStreams,
                                       OwnedArray<DeviceInfo>* /*devices*/,
                                       OwnedArray<ConfigurationObject>* /*configurationObjects*/)
{
    continuousChannels->clear();
    eventChannels->clear();
    sourceStreams->clear();
    sourceBuffers.clear();
    dataBufferAC_ = nullptr;
    dataBufferDC_ = nullptr;
    streamAC_ = nullptr;
    streamDC_ = nullptr;

    // --- AC stream ---
    {
        DataStream::Settings ds {
            "rhs_AC_stream",
            "RHS2116 AC stream (16 ch, uV)",
            "rhs_ac_stream_id",
            sampleRateHz_,
            true  // generates_timestamps
        };
        streamAC_ = new DataStream(ds);
        sourceStreams->add(streamAC_);
    }

    // --- DC stream ---
    {
        DataStream::Settings ds {
            "rhs_DC_stream",
            "RHS2116 DC stream (16 ch, mV)",
            "rhs_dc_stream_id",
            sampleRateHz_,
            true  // generates_timestamps
        };
        streamDC_ = new DataStream(ds);
        sourceStreams->add(streamDC_);
    }

    // --- ENV stream (2 ch, uV) ---
    {
        DataStream::Settings ds {
            "env_stream",
            "Environmental (Temp °C, Humidity %) in uV",
            "env_stream_id",
            sampleRateHz_,
            true  // generates_timestamps
        };
        streamENV_ = new DataStream(ds);
        sourceStreams->add(streamENV_);
    }

    // --- SPAD stream (8 ch, uV) ---
    {
        DataStream::Settings ds {
            "spad_stream",
            "SPAD stream (8 ch, uV)",
            "spad_stream_id",
            sampleRateHz_,
            true  // generates_timestamps
        };
        streamSPAD_ = new DataStream(ds);
        sourceStreams->add(streamSPAD_);
    }

    // --- allocate 2 internal buffers
    {
        const int internalCapacity = 30000000; // samples per channel
        sourceBuffers.add(new DataBuffer(NUM_CH, internalCapacity)); // AC
        dataBufferAC_ = sourceBuffers.getLast();
        sourceBuffers.add(new DataBuffer(NUM_CH, internalCapacity)); // DC
        dataBufferDC_ = sourceBuffers.getLast();
        sourceBuffers.add(new DataBuffer(ENV_NUM_CH, internalCapacity)); // ENV
        dataBufferENV_ = sourceBuffers.getLast();
        sourceBuffers.add(new DataBuffer(SPAD_NUM_CH, internalCapacity)); // SPAD
        dataBufferSPAD_ = sourceBuffers.getLast();
    }

    // --- 16 canali AC in microvolt ---
    for (int ch = 0; ch < NUM_CH; ++ch)
    {
        ContinuousChannel::Settings cs{
            ContinuousChannel::Type::ELECTRODE,
            "CH" + String(ch + 1),
            "RHS2116 (uV)",
            "rhs_ac_ch_" + String(ch),
            1.0,
            streamAC_
        };
        continuousChannels->add(new ContinuousChannel(cs));
    }

    // --- 16 canali DC in millivolt ---
    for (int ch = 0; ch < NUM_CH; ++ch)
    {
        ContinuousChannel::Settings cs{
            ContinuousChannel::Type::ELECTRODE,
            "CH" + String(ch + 1),
            "RHS2116 (mV)",
            "rhs_dc_ch_" + String(ch),
            1.0,
            streamDC_
        };
        continuousChannels->add(new ContinuousChannel(cs));
    }

    // --- 2 ENV channels in microvolt (1:1 mapping) ---
    {
        // ch 0: Temperature
        ContinuousChannel::Settings csT{
            ContinuousChannel::Type::ELECTRODE,
            "Temp",
            "ENV (uV)",               
            "env_temp_uV",
            1.0,
            streamENV_
        };
        continuousChannels->add(new ContinuousChannel(csT));

        // ch 1: Humidity
        ContinuousChannel::Settings csH{
            ContinuousChannel::Type::ELECTRODE,
            "Hum",
            "ENV (uV)",               
            "env_humidity_uV",
            1.0,
            streamENV_
        };
        continuousChannels->add(new ContinuousChannel(csH));
    }

    // --- SPAD channels in microvolt (binary bits -> uV) ---
    for (int ch = 0; ch < SPAD_NUM_CH; ++ch)
    {
        ContinuousChannel::Settings cs{
            ContinuousChannel::Type::ELECTRODE,
            "SPAD" + String(ch + 1),
            "SPAD (uV)",
            "spad_ch_" + String(ch),
            1.0,
            streamSPAD_
        };
        continuousChannels->add(new ContinuousChannel(cs));
    }

    // reset env sample counter
    envSamples_ = 0;
    spadTotalSamples_ = 0;
    totalSamples_ = 0;
    if (queue_) queue_->reset();
    if (envQueue_) envQueue_->reset();
    if (spadQueue_) spadQueue_->reset();

}
// ==============================================================================================================================








// ===================================================== start acquisition ======================================================
bool DataThreadPlugin::startAcquisition()
{
    totalSamples_ = 0;
    envSamples_ = 0;

    // Open serial port and configure the device
    //serial_.setup(serialPort_, serialBaud_);

    // Open serial port and configure the device (with N retries)
    const int kMaxAttempts = 20;
    const int kDelayMs     = 500;

    std::this_thread::sleep_for(std::chrono::seconds(1));
    bool opened = false;
    for (int attempt = 1; attempt <= kMaxAttempts && !opened; ++attempt) {
        serial_.setup(serialPort_, serialBaud_);
        opened = serial_.isInitialized();
        if (!opened) {
            std::printf("[STM32-RHS2116] serial open failed (%d/%d)\n",
                        attempt, kMaxAttempts);
            std::fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(kDelayMs));
        }
    }
    if (!opened) {
        std::printf("[STM32-RHS2116] ERROR: cannot open serial after %d attempts\n", kMaxAttempts);
        return false;
    }

    // Create RHS2116 interface
    rhs_ = std::make_unique<IntanRHS2116>(serial_);


    // Apply requested settings
    rhs_->setSampleRate(sampleRateHz_);
    rhs_->setLowerBandwidth(lowerBwHz_);
    rhs_->setUpperBandwidth(upperBwHz_);
    rhs_->setDspEnable(dspEnabled_);
    rhs_->setDspFrequency(dspK_);

    // Stimulation requsted settings
    rhs_->setVoltage(stimVoltageV_);
    rhs_->setStepSize(stimStepNa_);
    rhs_->setPosStimCurrent(stimPosCurrent_);
    rhs_->setNegStimCurrent(stimNegCurrent_);
    rhs_->setStimType(stimType_);
    rhs_->setStimPolarity(stimPolarity_);
    rhs_->setNumberOfClkPos(stimClkPos_);
    rhs_->setNumberOfClkNeg(stimClkNeg_);
    rhs_->setContinuousStim(stimContinuous_);
    rhs_->setStateCR(crEnable_);
    rhs_->setNumberOfClkCR(crClk_);

    // TLC LED settings (apply stored preset values)
    if (tlcMaxOn_ == 1) {
        rhs_->setTlcAllOnMax();
    }
    else {
        // set PWM only when not forcing all-on-max
        rhs_->setTlcAllOnPwm(tlcPwm_);
    }

    // Optical stim clock: always set the clock register (safe to call)
    rhs_->setOpticClk(static_cast<uint16_t>(opticClk_));


    // Configure the device
    rhs_->configure();

    // Wait a bit for the device to settle
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start the serial I/O thread
    serialRunning_.store(true);
    serialThread_ = std::thread(&DataThreadPlugin::serialLoop, this);

    // Start the ENV print thread
    envRunning_.store(true);
    envThread_ = std::thread(&DataThreadPlugin::envPrintLoop, this);

    // Start acquisition
    rhs_->startAcquisition();

    // Start the DataThread
    startThread();
    return true;
}
// ==============================================================================================================================








// =================================================== update buffer ============================================================
bool DataThreadPlugin::updateBuffer()
{
    // Parse raw packets and push to DataBuffer. No I/O here.
    if (dataBufferAC_ == nullptr || dataBufferDC_ == nullptr || streamAC_ == nullptr || streamDC_ == nullptr)
        return false;

    int drained = 0, idx = -1;

    // scratch per un blocco (100 sample, 16 canali)
    std::array<float,  NUM_CH * BLOCK_NSAMP> samplesAC{};
    std::array<float,  NUM_CH * BLOCK_NSAMP> samplesDC{};
    std::array<int64,  BLOCK_NSAMP>          sampleNumbers{};
    std::array<double, BLOCK_NSAMP>          timestamps{};
    std::array<uint64, BLOCK_NSAMP>          eventCodes{}; // 0

    while (drained < MAX_DRAIN_PER_CALL && queue_->tryPopReady(idx))
    {
        auto& raw = queue_->at(idx);

        // timestamp base (ticks da 100 us)
        const uint32 ticks  = readLE32(raw.bytes.data());
        const double ts0_s  = (ticks * TS_TICK_US) * 1e-6;
        const double dt_s   = 1.0 / sampleRateHz_;

        // --- de-interleave and convert to µV ---
        for (int i = 0; i < BLOCK_NSAMP; ++i) {
            const int64 snum = totalSamples_ + i;
            sampleNumbers[static_cast<size_t>(i)] = snum;
            timestamps[static_cast<size_t>(i)]    = ts0_s + i * dt_s;
            eventCodes[static_cast<size_t>(i)]    = 0;
        }

        // payload
        const uint8* payload = raw.bytes.data() + TIMESTAMP_BYTES;
        constexpr int BYTES_PER_SAMPLE = TOTAL_HW_CH * 2;

        for (int i = 0; i < BLOCK_NSAMP; ++i)
        {
            const int sampleOffset = i * BYTES_PER_SAMPLE;

            // AC channels (16 ch, µV)
            for (int outCh = 0; outCh < NUM_CH; ++outCh) {
                const int hwCh      = CH_MAP[static_cast<size_t>(outCh)];
                const int byteIndex = sampleOffset + (hwCh * 2);
                const uint16 adc    = readLE16(payload + byteIndex);
                const int centered  = static_cast<int>(adc) - 32768;
                const float uV      = AC_UV_PER_LSB * static_cast<float>(centered);
                samplesAC[static_cast<size_t>(outCh) * BLOCK_NSAMP + static_cast<size_t>(i)] = uV;
            }

            //  DC channels (16 ch, uV)
            for (int outCh = 0; outCh < NUM_CH; ++outCh) {
                const int hwCh      = DC10_MAP[static_cast<size_t>(outCh)];
                const int byteIndex = sampleOffset + (hwCh * 2);
                const uint16 raw16  = readLE16(payload + byteIndex);
                const int adc10     = static_cast<int>(raw16 & DC_MASK_10B);
                const int centered  = adc10 - DC_CENTER_10B;
                const float uV      = DC_UV_PER_LSB * static_cast<float>(centered);
                samplesDC[static_cast<size_t>(outCh) * BLOCK_NSAMP + static_cast<size_t>(i)] = uV;
            }
        }

        // build ENV block with zero-order hold (fill gaps at ~1 Hz)
        static thread_local std::array<float, ENV_NUM_CH * BLOCK_NSAMP> envBlock;

        float vT = envTempUV_.load(std::memory_order_relaxed);
        float vH = envRhUV_.load(std::memory_order_relaxed);
        // default to 0 if not yet received
        if (std::isnan(vT)) vT = 0.0f;
        if (std::isnan(vH)) vH = 0.0f;

        // channel-major layout: ch*NS + i
        for (int i = 0; i < BLOCK_NSAMP; ++i) {
            envBlock[0 * BLOCK_NSAMP + i] = vT; // Temperature (uV)
            envBlock[1 * BLOCK_NSAMP + i] = vH; // Humidity    (uV)
        }

        // publish to DataBuffer
        dataBufferAC_->addToBuffer(samplesAC.data(), sampleNumbers.data(), timestamps.data(), eventCodes.data(), BLOCK_NSAMP); // AC
        dataBufferDC_->addToBuffer(samplesDC.data(), sampleNumbers.data(), timestamps.data(), eventCodes.data(), BLOCK_NSAMP); // DC
        dataBufferENV_->addToBuffer(envBlock.data(), sampleNumbers.data(), timestamps.data(), eventCodes.data(), BLOCK_NSAMP); // ENV

        // advance total sample count
        totalSamples_ += BLOCK_NSAMP;

        // release the raw slot
        queue_->releaseFree(idx);
        ++drained;
    }

    //  SPAD queue and publish (binary bits -> uV) 
    {
        int idx_spad = -1;
        int drained_spad = 0;

        while (drained_spad < MAX_DRAIN_PER_CALL && spadQueue_->tryPopReady(idx_spad)) {
            std::array<float, SPAD_NUM_CH * SPAD_NSAMP> samplesSPAD{};
            std::array<int64, SPAD_NSAMP> sampleNumbersSPAD{};
            std::array<double, SPAD_NSAMP> timestampsSPAD{};
            std::array<uint64, SPAD_NSAMP> eventCodesSPAD{};

            auto& sb = spadQueue_->at(idx_spad);
            const uint32 ticks = readLE32(sb.bytes.data());
            const double ts0_s = (ticks * TS_TICK_US) * 1e-6;
            const double dt_s  = 1.0 / sampleRateHz_;

            // de-interleave and convert to µV
            for (int i = 0; i < SPAD_NSAMP; ++i) {
                const int64 snum = spadTotalSamples_ + i;
                sampleNumbersSPAD[static_cast<size_t>(i)] = snum;
                timestampsSPAD[static_cast<size_t>(i)] = ts0_s + i * dt_s;
                eventCodesSPAD[static_cast<size_t>(i)] = 0;

                // each byte has 8 bits -> 8 SPAD channels
                const uint8 v = sb.bytes[TIMESTAMP_BYTES + i];
                for (int ch = 0; ch < SPAD_NUM_CH; ++ch) {
                    const int bit = (v >> ch) & 1;
                    const float uv = bit ? 3300.0f : 0.0f;
                    samplesSPAD[static_cast<size_t>(ch) * SPAD_NSAMP + static_cast<size_t>(i)] = uv;
                }
            }

            // publish to DataBuffer
            if (dataBufferSPAD_) dataBufferSPAD_->addToBuffer(samplesSPAD.data(), sampleNumbersSPAD.data(), timestampsSPAD.data(), eventCodesSPAD.data(), SPAD_NSAMP);
            spadTotalSamples_ += SPAD_NSAMP;
            spadQueue_->releaseFree(idx_spad);
            ++drained_spad;
        }
    }

    return true;

}
// ==============================================================================================================================








// =================================================== stop acquisition =========================================================
bool DataThreadPlugin::stopAcquisition()
{
    // Stop device
    if (rhs_){
        rhs_->stopAcquisition();
        //rhs_->reset();
    }

    // Stop serial I/O thread
    serialRunning_.store(false);
    if (serialThread_.joinable()) serialThread_.join();

    // Stop DataThread
    if (isThreadRunning())
        signalThreadShouldExit();
    waitForThreadToExit(500);

    // Clear internal buffers
    if (dataBufferAC_ != nullptr)
        dataBufferAC_->clear();

    if (dataBufferDC_ != nullptr)
        dataBufferDC_->clear();

    if (dataBufferENV_ != nullptr)
        dataBufferENV_->clear();

    if (dataBufferSPAD_ != nullptr)
        dataBufferSPAD_->clear();

    // Stop ENV print thread
    envRunning_.store(false);
    if (envThread_.joinable()) envThread_.join();

    // Close serial port
    serial_.flush(true, true);  // input + output
    serial_.close();

    return true;
}
// ==============================================================================================================================








// ===================================================== Serial I/O thread  =====================================================
void DataThreadPlugin::serialLoop()
{
    uint8 b = 0;

    while (serialRunning_.load())
    {
        // find a valid sync
        do {
            serial_.readData(reinterpret_cast<char*>(&b), 1);
        } while (serialRunning_.load() && b != SYNC_BYTE && b != ENV_SYNC && b != SPAD_SYNC);

        if (!serialRunning_.load()) break;

        // read EMG block
        if (b == SYNC_BYTE)
        {
            int idx = queue_->acquireFreeBlocking(serialRunning_); if (idx < 0) break;
            auto& raw = queue_->at(idx);
            size_t got = 0;
            while (got < static_cast<size_t>(BLOCK_BYTES) && serialRunning_.load()) {
                long r = serial_.readData(reinterpret_cast<char*>(raw.bytes.data() + got),
                                          static_cast<size_t>(BLOCK_BYTES) - got);
                if (r > 0) got += static_cast<size_t>(r);
            }

            // push to ready queue
            queue_->pushReady(idx);
        }

        // read ENV block
        else if (b == ENV_SYNC)
        {
            int idx = envQueue_->acquireFreeBlocking(serialRunning_); if (idx < 0) break;
            auto& eb = envQueue_->at(idx);
            size_t got = 0;
            while (got < static_cast<size_t>(ENV_BYTES_AFTER_H) && serialRunning_.load()) {
                long r = serial_.readData(reinterpret_cast<char*>(eb.bytes.data() + got),
                                          static_cast<size_t>(ENV_BYTES_AFTER_H) - got);
                if (r > 0) got += static_cast<size_t>(r);
            }

            // push to ready queue
            envQueue_->pushReady(idx);
        }

        // read SPAD block
        else if (b == SPAD_SYNC)
        {
            int idx = spadQueue_->acquireFreeBlocking(serialRunning_); if (idx < 0) break;
            auto& sb = spadQueue_->at(idx);
            size_t got = 0;
            while (got < static_cast<size_t>(SPAD_BYTES_AFTER_H) && serialRunning_.load()) {
                long r = serial_.readData(reinterpret_cast<char*>(sb.bytes.data() + got),
                                          static_cast<size_t>(SPAD_BYTES_AFTER_H) - got);
                if (r > 0) got += static_cast<size_t>(r);
            }

            // push to ready queue
            spadQueue_->pushReady(idx);
        }
    }
}

// ==============================================================================================================================


// ===================================================== ENV print thread  ======================================================
void DataThreadPlugin::envPrintLoop()
{
    int idx = -1;
    while (envRunning_.load())
    {
        if (envQueue_->tryPopReady(idx))
        {
            auto& eb = envQueue_->at(idx);
            const uint32 ts = readLE32(eb.bytes.data())/10000;
            const int8_t tC = static_cast<int8_t>(eb.bytes[4]);
            const int8_t rh = static_cast<int8_t>(eb.bytes[5]);

            std::printf("[STM32-RHS2116] ts=%u s, temp=%d C, humidity=%d %%\n", ts, (int)tC, (int)rh);
            std::fflush(stdout);


            //   Temp [°C] -> uV with same numeric value (e.g., 37°C -> 37 uV)
            //   Humidity [%] -> uV with same numeric value (e.g., 100% -> 100 uV)

            // store latest values (°C -> uV, % -> uV) for zero-order hold
            envTempUV_.store(static_cast<float>(tC), std::memory_order_relaxed);
            envRhUV_.store(static_cast<float>(rh),   std::memory_order_relaxed);

            // store latest sample metadata
            sampleNumbers_env[0] = envSamples_++;
            timestamps_env[0]    = static_cast<double>(ts);


            envQueue_->releaseFree(idx);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

// ==============================================================================================================================




// ================================================= DataThreadPluginEditor impl ================================================

// set serial port
bool DataThreadPlugin::setSerialPort(const std::string& name)
{
    serialPort_ = name;
    return true;
}

// set serial baudrate
bool DataThreadPlugin::setSampleRate(int hz)
{
    if (hz <= 0) 
        return false;
    sampleRateHz_ = hz;
    return true;
}

// set lower bandwidth
bool DataThreadPlugin::setLowerBandwidthHz(double hz)
{
    if (hz <= 0.0) 
        return false;
    lowerBwHz_ = hz;
    return true;
}

// set upper bandwidth
bool DataThreadPlugin::setUpperBandwidthHz(double hz)
{
    if (hz <= 0.0) 
        return false;
    upperBwHz_ = hz;
    return true;
}

// set DSP enabled
bool DataThreadPlugin::setDspEnabled(bool enabled)
{
    dspEnabled_ = enabled;
    return true;
}

// set DSP k-factor
bool DataThreadPlugin::setDspKFactor(int k)
{
    if (k < 0) 
        return false;
    dspK_ = k;
    return true;
}

// set acquisition time in seconds
bool DataThreadPlugin::setAcquisitionTimeSeconds(int seconds)
{
    if (seconds < 0) 
        return false;
    acquisitionTimeSec_ = seconds;
    return true;
}

// set stimulation enabled
bool DataThreadPlugin::setStimEnabled(bool v)
{ 
    stimEnabled_ = v; 
    return true; 
}

// set stimulation voltage
bool DataThreadPlugin::setStimVoltage(double v)          
{ 
    stimVoltageV_ = v; 
    return true; 
}

// set stimulation step size
bool DataThreadPlugin::setStimStepNa(int v)              
{ 
    if (v<=0) 
        return false; 
    stimStepNa_ = v; 
    return true; 
}

// set positive stimulation current
bool DataThreadPlugin::setStimPosCurrent(int v)          
{ 
    if (v<1||v>255) 
        return false; 
    stimPosCurrent_ = v; 
    return true; 
}

// set negative stimulation current
bool DataThreadPlugin::setStimNegCurrent(int v)          
{ 
    if (v<1||v>255) 
        return false; 
    stimNegCurrent_ = v; 
    return true; 
}

// set stimulation type
bool DataThreadPlugin::setStimType(int v)                
{ 
    if (v<0||v>1) 
        return false; 
    stimType_ = v; 
    return true; 
}

// set stimulation polarity
bool DataThreadPlugin::setStimPolarity(int v)            
{ 
    if (v<0||v>1) 
        return false; 
    stimPolarity_ = v; 
    return true; 
}

// set number of positive stimulation clocks
bool DataThreadPlugin::setStimClkPos(int v)              
{ 
    if (v<0) 
        return false; 
    stimClkPos_ = v; 
    return true; 
}

// set number of negative stimulation clocks
bool DataThreadPlugin::setStimClkNeg(int v)              
{ 
    if (v<0) 
        return false; 
    stimClkNeg_ = v; 
    return true; 
}

// set continuous stimulation
bool DataThreadPlugin::setStimContinuous(int v)          
{ 
    if (v<0||v>1) 
        return false; 
    stimContinuous_ = v; 
    return true; 
}

// set charge recovery enabled
bool DataThreadPlugin::setChargeRecoveryEnable(int v)    
{ 
    if (v<0||v>1) 
        return false; 
    crEnable_ = v; 
    return true; 
}

// set charge recovery clocks
bool DataThreadPlugin::setChargeRecoveryClk(int v)       
{ 
    if (v<0) 
        return false; 
    crClk_ = v; 
    return true; 
}

// set delay after stimulation in seconds
bool DataThreadPlugin::setStimulationTimeMs(int v)       
{ 
    if (v<0) 
        return false; 
    stimTimeMs_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimSequenceRepeatMs(int v)
{
    if (v < 0)
        return false;
    stimSequenceRepeatMs_ = v;
    return true;
}

// set delay after stimulation in seconds
bool DataThreadPlugin::setTlcMaxOn(int v)
{
    if (v != 0 && v != 1) return false;
    tlcMaxOn_ = v;
    return true;
}

// set TLC PWM value
bool DataThreadPlugin::setTlcPwm(int v)
{
    if (v < 0 || v > 255) return false;
    tlcPwm_ = v;
    return true;
}

// set delay after stimulation in seconds
bool DataThreadPlugin::setStimMode(int v)
{
    if (v != 0 && v != 1) return false;
    stimMode_ = v;
    return true;
}

// set optical stimulation clock
bool DataThreadPlugin::setOpticClk(int v)
{
    if (v < 0 || v > 65535) return false;
    opticClk_ = v;
    return true;
}

// set delay after stimulation in seconds
bool DataThreadPlugin::setOpticSequence(const std::vector<uint8_t>& seq)
{
    opticSeq_.clear();
    opticSeq_.reserve(seq.size());
    for (auto b : seq) opticSeq_.push_back(b);
    return true;
}

// set stimulation sequence
bool DataThreadPlugin::setStimSequence(const std::vector<StimCmd>& seq) {
    stimSeq_.clear();

    const size_t cap = (seq.size() < static_cast<size_t>(kMaxStimSeq))
                       ? seq.size()
                       : static_cast<size_t>(kMaxStimSeq);

    stimSeq_.reserve(cap);
    for (size_t i = 0; i < cap; ++i)
        stimSeq_.push_back(seq[i]);

    stimSeqIdx_ = 0;
    return true;
}

// set delay after stimulation in seconds
bool DataThreadPlugin::setPresetFolderPath(const std::string& path)
{
    if (!path.empty() && !fs::is_directory(fs::u8path(path)))
        return false;

    presetFolderPath_ = path;
    return true;
}

// start preset sequence
bool DataThreadPlugin::startSequence()
{
    // if a previous thread object exists, make sure it has finished and been joined
    if (sequenceThread_.joinable()) {
        // this can happen when the previous preset run reaches the end naturally
        // the thread() object is still joinable even though it has completed.
        // assigning to it without joining would call its destructor and terminate.
        sequenceThread_.join();
    }

    if (sequenceRunning_.exchange(true)) return false;         // already running
    if (!prepareSequenceHeader()) { sequenceRunning_.store(false); return false; }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    sequenceThread_ = std::thread(&DataThreadPlugin::presetSequenceThread, this);
    return true;
}

// preset sequence thread
void DataThreadPlugin::presetSequenceThread()
{
    for (int i = 0; i < gSeqCount && sequenceRunning_.load(); ++i)
    {
        if (!loadAndApplyPreset(i)) continue;
        CoreServices::setAcquisitionStatus(true);
        CoreServices::setRecordingStatus(true);

        const int secs = acquisitionTimeSec_;
        const bool hasElectricalSequence = !stimSeq_.empty();
        const bool hasOpticalSequence    = !opticSeq_.empty();
        const bool canStim = (stimEnabled_
                              && stimTimeMs_ > 0
                              && ((stimMode_ == 0 && hasElectricalSequence)
                                  || (stimMode_ != 0 && hasOpticalSequence)));
        bool stimActive = canStim;
        size_t stimIdx = 0;

        auto t0 = std::chrono::steady_clock::now();
        auto nextStim = t0 + std::chrono::milliseconds(stimTimeMs_);
        const bool repeatSequence = (stimSequenceRepeatMs_ > 0);
        std::chrono::steady_clock::time_point nextRepeat = (std::chrono::steady_clock::time_point::max)();
        if (repeatSequence)
            nextRepeat = t0 + std::chrono::milliseconds(stimSequenceRepeatMs_);

        int printed = 0;
        while (sequenceRunning_.load())
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - t0).count();
            if (secs > 0 && elapsed_s >= secs) break;

            if (repeatSequence && canStim && now >= nextRepeat)
            {
                do {
                    nextRepeat += std::chrono::milliseconds(stimSequenceRepeatMs_);
                } while (now >= nextRepeat);

                stimIdx = 0;
                stimActive = true;
                nextStim = now + std::chrono::milliseconds(stimTimeMs_);
            }

            if (elapsed_s > printed) {
                printed = (int)elapsed_s;
                //std::printf("[STM32-RHS2116] preset %d: %d/%d s\n", i+1, printed, secs);
                //std::fflush(stdout);
            }

            if (stimActive && now >= nextStim)
            {
                    if (stimMode_ == 0) {
                        // Electrical stimulation (existing behavior)
                        const auto& c = stimSeq_[stimIdx];
                        if (rhs_) rhs_->stim(c.mode, c.ch1, c.ch2);

                        ++stimIdx;
                        if (stimIdx >= stimSeq_.size()) {
                            stimActive = false;
                        } else {
                            nextStim += std::chrono::milliseconds(stimTimeMs_);
                        }
                    }
                else {
                    // Optical stimulation: use opticSeq_ (8-bit patterns)
                    if (!opticSeq_.empty()) {
                        uint8_t pattern = opticSeq_[stimIdx % opticSeq_.size()];
                        if (rhs_) rhs_->optic_stim(pattern);
                        ++stimIdx;
                        if (stimIdx >= opticSeq_.size()) {
                            stimActive = false;
                        } else {
                            nextStim += std::chrono::milliseconds(stimTimeMs_);
                        }
                    } else {
                        // no optic sequence defined -> disable stimulation
                        stimActive = false;
                    }
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        CoreServices::setRecordingStatus(false);
        CoreServices::setAcquisitionStatus(false);
        std::this_thread::sleep_for(std::chrono::seconds(delayAfterStim_sec_));
    }

    if (editor_) editor_->setStartToggle(false);
    sequenceRunning_.store(false);
}

// stop preset sequence
bool DataThreadPlugin::stopSequence()
{
    sequenceRunning_.store(false);
    if (sequenceThread_.joinable()) sequenceThread_.join();
    CoreServices::setRecordingStatus(false);
    CoreServices::setAcquisitionStatus(false);
    return true;
}

// ==============================================================================================================================




// ================================================== Preset management =========================================================

// Validate preset folder, locate & parse sequence.json, cache results in globals.
bool DataThreadPlugin::prepareSequenceHeader()
{
    if (presetFolderPath_.empty() || !fs::is_directory(fs::u8path(presetFolderPath_))) {
        std::printf("[STM32-RHS2116] Preset folder not set or invalid\n"); return false;
    }

    // Locate sequence.json
    gBaseDir = juce::File{ juce::String(presetFolderPath_) };
    gSeqFile = gBaseDir.getChildFile("sequence.json");
    if (!gSeqFile.existsAsFile()) {
        std::printf("[STM32-RHS2116] sequence.json not found in %s\n", gBaseDir.getFullPathName().toRawUTF8());
        return false;
    }

    // Parse the sequence.json file
    gSeq = juce::var(); auto ok = juce::JSON::parse(gSeqFile.loadFileAsString(), gSeq);
    if (!ok.wasOk() || !gSeq.isArray()) { std::printf("[STM32-RHS2116] Invalid sequence.json\n"); return false; }

    // Count presets
    gSeqCount = gSeq.getArray()->size();
    std::printf("[STM32-RHS2116] sequence.json: %d presets found\n", gSeqCount);
    if (gSeqCount <= 0) return false;

    return true;
}

// ------------------------------- loadAndApplyPreset(index) --------------------------------
// Load preset by index from globals and apply to the editor.
bool DataThreadPlugin::loadAndApplyPreset(int index)
{
    // Basic bounds/structure checks
    if (!gSeq.isArray()) { std::printf("[STM32-RHS2116] Sequence not parsed\n"); return false; }
    if (index < 0 || index >= gSeqCount) {
        std::printf("[STM32-RHS2116] Preset index out of range (%d of %d)\n", index, gSeqCount);
        return false;
    }

    // Get preset filename
    const juce::var& entry = gSeq.getArray()->getReference(index);
    if (!entry.isString()) { std::printf("[STM32-RHS2116] Preset entry is not a string\n"); return false; }

    juce::File preset = gBaseDir.getChildFile(entry.toString());
    if (!preset.existsAsFile()) {
        std::printf("[STM32-RHS2116] Preset file not found: %s\n", preset.getFullPathName().toRawUTF8());
        return false;
    }

    // Parse preset JSON
    juce::var root;
    if (!juce::JSON::parse(preset.loadFileAsString(), root).wasOk() || !root.isObject()) {
        std::printf("[STM32-RHS2116] Invalid preset JSON: %s\n", preset.getFileName().toRawUTF8());
        return false;
    }

    // Log which preset is being applied
    std::printf("\n\n\n[STM32-RHS2116] ============== Applying preset #%d / %d: %s ==============\n\n", index+1, gSeqCount, preset.getFileName().toRawUTF8());

    if (!editor_) { std::printf("[STM32-RHS2116] Editor not attached\n"); return false; }
    if (!editor_->applyPresetObject(root)) return false;

    return true;
}
// ==============================================================================================================================




// ================================================= other DataThread methods ===================================================
void DataThreadPlugin::resizeBuffers() { /* no-op */ }

std::unique_ptr<GenericEditor> DataThreadPlugin::createEditor (SourceNode* sn)
{
    std::unique_ptr<DataThreadPluginEditor> editor = std::make_unique<DataThreadPluginEditor>(sn, this);
    editor_ = editor.get();
    return editor;
}

void DataThreadPlugin::handleBroadcastMessage (const String& msg, const int64 /*messageTimeMilliseconds*/)
{
    handleRhsCommand (rhs_.get(), msg);
}

String DataThreadPlugin::handleConfigMessage (const String& msg)
{
    return handleRhsCommand (rhs_.get(), msg);
}
void DataThreadPlugin::registerParameters() {}
void DataThreadPlugin::parameterValueChanged (Parameter* /*parameter*/) {}
// ==============================================================================================================================
