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
// ==============================================================================================================================




// ================================================= DataThreadPluginEditor impl ================================================
struct PluginSettingsObject { /* reserved */ };

DataThreadPlugin::DataThreadPlugin (SourceNode* sn)
: DataThread (sn)
{
    queue_ = std::make_unique<PacketPoolQueue>(N_BLOCKS);
    envQueue_ = std::make_unique<EnvPoolQueue>(N_ENV_BLOCKS);
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
            sampleRateHz_
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
            sampleRateHz_
        };
        streamDC_ = new DataStream(ds);
        sourceStreams->add(streamDC_);
    }

    // --- allocate 2 internal buffers (capienza ampia) ---
    {
        const int internalCapacity = 30000000; // samples per channel
        sourceBuffers.add(new DataBuffer(NUM_CH, internalCapacity)); // AC
        dataBufferAC_ = sourceBuffers.getLast();
        sourceBuffers.add(new DataBuffer(NUM_CH, internalCapacity)); // DC
        dataBufferDC_ = sourceBuffers.getLast();
    }

    // --- 16 canali AC in microvolt ---
    for (int ch = 0; ch < NUM_CH; ++ch)
    {
        ContinuousChannel::Settings cs{
            ContinuousChannel::Type::ELECTRODE,
            "RHS AC CH" + String(ch + 1),
            "RHS2116 (uV)",
            "rhs_ac_ch_" + String(ch + 1),
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
            "RHS DC CH" + String(ch + 1),
            "RHS2116 (mV)",
            "rhs_dc_ch_" + String(ch + 1),
            1.0,
            streamDC_
        };
        continuousChannels->add(new ContinuousChannel(cs));
    }

    totalSamples_ = 0;
    if (queue_) queue_->reset();
    if (envQueue_) envQueue_->reset();

}
// ==============================================================================================================================








// ===================================================== start acquisition ======================================================
bool DataThreadPlugin::startAcquisition()
{
    totalSamples_ = 0;

    // Open serial port and configure the device
    serial_.setup(serialPort_, serialBaud_);
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


    // Configure the device
    rhs_->configure();

    // Wait a bit for the device to settle
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start acquisition
    rhs_->startAcquisition();

    // Start the serial I/O thread
    serialRunning_.store(true);
    serialThread_ = std::thread(&DataThreadPlugin::serialLoop, this);

    envRunning_.store(true);
    envThread_ = std::thread(&DataThreadPlugin::envPrintLoop, this);

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

        // publish to DataBuffer
        dataBufferAC_->addToBuffer(samplesAC.data(), sampleNumbers.data(), timestamps.data(), eventCodes.data(), BLOCK_NSAMP);
        dataBufferDC_->addToBuffer(samplesDC.data(), sampleNumbers.data(), timestamps.data(), eventCodes.data(), BLOCK_NSAMP);

        // advance total sample count
        totalSamples_ += BLOCK_NSAMP;

        // release the raw slot
        queue_->releaseFree(idx);
        ++drained;
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
        rhs_->reset();
    } 

    serialRunning_.store(false);
    if (serialThread_.joinable()) serialThread_.join();

    if (isThreadRunning())
        signalThreadShouldExit();
    waitForThreadToExit(500);

    if (dataBufferAC_ != nullptr)
        dataBufferAC_->clear();

    if (dataBufferDC_ != nullptr)
        dataBufferDC_->clear();

    envRunning_.store(false);
    if (envThread_.joinable()) envThread_.join();
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
        } while (serialRunning_.load() && b != SYNC_BYTE && b != ENV_SYNC);

        if (!serialRunning_.load()) break;

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
            queue_->pushReady(idx);
        }
        else // ENV_SYNC
        {
            int idx = envQueue_->acquireFreeBlocking(serialRunning_); if (idx < 0) break;
            auto& eb = envQueue_->at(idx);
            size_t got = 0;
            while (got < static_cast<size_t>(ENV_BYTES_AFTER_H) && serialRunning_.load()) {
                long r = serial_.readData(reinterpret_cast<char*>(eb.bytes.data() + got),
                                          static_cast<size_t>(ENV_BYTES_AFTER_H) - got);
                if (r > 0) got += static_cast<size_t>(r);
            }
            envQueue_->pushReady(idx);
        }
    }
}

// ==============================================================================================================================


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

            envQueue_->releaseFree(idx);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}






// ================================================= DataThreadPluginEditor impl ================================================
bool DataThreadPlugin::setSerialPort(const std::string& name)
{
    serialPort_ = name;
    return true;
}

bool DataThreadPlugin::setSampleRate(int hz)
{
    if (hz <= 0) 
        return false;
    sampleRateHz_ = hz;
    return true;
}

bool DataThreadPlugin::setLowerBandwidthHz(double hz)
{
    if (hz <= 0.0) 
        return false;
    lowerBwHz_ = hz;
    return true;
}

bool DataThreadPlugin::setUpperBandwidthHz(double hz)
{
    if (hz <= 0.0) 
        return false;
    upperBwHz_ = hz;
    return true;
}

bool DataThreadPlugin::setDspEnabled(bool enabled)
{
    dspEnabled_ = enabled;
    return true;
}

bool DataThreadPlugin::setDspKFactor(int k)
{
    if (k < 0) 
        return false;
    dspK_ = k;
    return true;
}

bool DataThreadPlugin::setAcquisitionTimeSeconds(int seconds)
{
    if (seconds < 0) 
        return false;
    acquisitionTimeSec_ = seconds;
    return true;
}

bool DataThreadPlugin::setStimEnabled(bool v)
{ 
    stimEnabled_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimVoltage(double v)          
{ 
    stimVoltageV_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimStepNa(int v)              
{ 
    if (v<=0) 
        return false; 
    stimStepNa_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimPosCurrent(int v)          
{ 
    if (v<1||v>255) 
        return false; 
    stimPosCurrent_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimNegCurrent(int v)          
{ 
    if (v<1||v>255) 
        return false; 
    stimNegCurrent_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimType(int v)                
{ 
    if (v<0||v>1) 
        return false; 
    stimType_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimPolarity(int v)            
{ 
    if (v<0||v>1) 
        return false; 
    stimPolarity_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimClkPos(int v)              
{ 
    if (v<0) 
        return false; 
    stimClkPos_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimClkNeg(int v)              
{ 
    if (v<0) 
        return false; 
    stimClkNeg_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimContinuous(int v)          
{ 
    if (v<0||v>1) 
        return false; 
    stimContinuous_ = v; 
    return true; 
}

bool DataThreadPlugin::setChargeRecoveryEnable(int v)    
{ 
    if (v<0||v>1) 
        return false; 
    crEnable_ = v; 
    return true; 
}

bool DataThreadPlugin::setChargeRecoveryClk(int v)       
{ 
    if (v<0) 
        return false; 
    crClk_ = v; 
    return true; 
}

bool DataThreadPlugin::setStimulationTimeMs(int v)       
{ 
    if (v<0) 
        return false; 
    stimTimeMs_ = v; 
    return true; 
}

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



bool DataThreadPlugin::setPresetFolderPath(const std::string& path)
{
    if (!path.empty() && !fs::is_directory(fs::u8path(path)))
        return false;

    presetFolderPath_ = path;
    return true;
}

bool DataThreadPlugin::startSequence()
{
    if (sequenceRunning_.exchange(true)) return false;         // already running
    if (!prepareSequenceHeader()) { sequenceRunning_.store(false); return false; }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    sequenceThread_ = std::thread(&DataThreadPlugin::presetSequenceThread, this);
    return true;
}

void DataThreadPlugin::presetSequenceThread()
{
    for (int i = 0; i < gSeqCount && sequenceRunning_.load(); ++i)
    {
        if (!loadAndApplyPreset(i)) continue;
        CoreServices::setRecordingStatus(true);

        const int secs = acquisitionTimeSec_;
        bool stimActive = (stimEnabled_ && stimTimeMs_ > 0 && !stimSeq_.empty());
        size_t stimIdx = 0;

        auto t0 = std::chrono::steady_clock::now();
        auto nextStim = t0 + std::chrono::milliseconds(stimTimeMs_);

        int printed = 0;
        while (sequenceRunning_.load())
        {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - t0).count();
            if (secs > 0 && elapsed_s >= secs) break;

            if (elapsed_s > printed) {
                printed = (int)elapsed_s;
                //std::printf("[STM32-RHS2116] preset %d: %d/%d s\n", i+1, printed, secs);
                //std::fflush(stdout);
            }

            if (stimActive && now >= nextStim)
            {
                const auto& c = stimSeq_[stimIdx];
                if (rhs_) rhs_->stim(c.mode, c.ch1, c.ch2);

                ++stimIdx;
                if (stimIdx >= stimSeq_.size()) {
                    stimActive = false;
                } else {
                    nextStim += std::chrono::milliseconds(stimTimeMs_);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        CoreServices::setAcquisitionStatus(false);
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }

    if (editor_) editor_->setStartToggle(false);
    sequenceRunning_.store(false);
}




bool DataThreadPlugin::stopSequence()
{
    sequenceRunning_.store(false);
    if (sequenceThread_.joinable()) sequenceThread_.join();
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
    std::printf("\n\n\n[STM32-RHS2116] ============== Applying preset #%d: %s ==============\n\n", index+1, preset.getFileName().toRawUTF8());

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

void DataThreadPlugin::handleBroadcastMessage (const String& /*msg*/, const int64 /*messageTimeMilliseconds*/) {}
String DataThreadPlugin::handleConfigMessage (const String& /*msg*/) { return ""; }
void DataThreadPlugin::registerParameters() {}
void DataThreadPlugin::parameterValueChanged (Parameter* /*parameter*/) {}
// ==============================================================================================================================
