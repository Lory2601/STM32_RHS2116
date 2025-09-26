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








// ================================================= DataThreadPluginEditor impl ================================================
struct PluginSettingsObject { /* reserved */ };

DataThreadPlugin::DataThreadPlugin (SourceNode* sn)
: DataThread (sn)
{
    queue_ = std::make_unique<PacketPoolQueue>(N_BLOCKS);
}

DataThreadPlugin::~DataThreadPlugin()
{
    serialRunning_.store(false);
    if (serialThread_.joinable()) serialThread_.join();
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

    // Create RHS stream
    {
        DataStream::Settings ds {
            "rhs_stream",
            "RHS2116 hardware stream (16 channels)",
            "rhs_stream_id",
            sampleRateHz_
        };
        stream_ = new DataStream(ds);
        sourceStreams->add(stream_);
    }

    // Allocate internal buffer (ample headroom)
    {
        const int internalCapacity = 30000000; // samples per channel
        sourceBuffers.add(new DataBuffer(NUM_CH, internalCapacity));
        dataBuffer_ = sourceBuffers.getLast();
    }

    // Register 16 channels (bitVolts = 1.0 since we push microvolts)
    for (int ch = 0; ch < NUM_CH; ++ch)
    {
        ContinuousChannel::Settings cs{
            ContinuousChannel::Type::ELECTRODE,
            "RHS CH" + String(ch + 1),
            "RHS2116 (uV)",
            "rhs_ch_" + String(ch + 1),
            1.0,
            stream_
        };
        continuousChannels->add(new ContinuousChannel(cs));
    }

    totalSamples_ = 0;
    if (queue_) queue_->reset();
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

    // Configure the device
    rhs_->configure();

    // Wait a bit for the device to settle
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Start acquisition
    rhs_->startAcquisition();

    serialRunning_.store(true);
    serialThread_ = std::thread(&DataThreadPlugin::serialLoop, this);

    startThread(); // kicks the updateBuffer() loop
    return true;
}
// ==============================================================================================================================








// =================================================== update buffer ============================================================
bool DataThreadPlugin::updateBuffer()
{
    // Parse raw packets and push to DataBuffer. No I/O here.
    if (dataBuffer_ == nullptr || stream_ == nullptr) return false;

    int drained = 0;
    int idx = -1;

    // Scratch buffers for a single device packet (100 samples, 16 channels)
    std::array<float,  NUM_CH * BLOCK_NSAMP> samples{};
    std::array<int64,  BLOCK_NSAMP>          sampleNumbers{};
    std::array<double, BLOCK_NSAMP>          timestamps{};
    std::array<uint64, BLOCK_NSAMP>          eventCodes{}; // zeros

    while (drained < MAX_DRAIN_PER_CALL && queue_->tryPopReady(idx))
    {
        auto& raw = queue_->at(idx);

        // 1) Timestamp (u32 LE), device tick = 100 us
        const uint32 ticks  = readLE32(raw.bytes.data());
        const double ts0_s  = (ticks * TS_TICK_US) * 1e-6;  // seconds
        const double dt_s   = 1.0 / sampleRateHz_;

        for (int i = 0; i < BLOCK_NSAMP; ++i) {
            const int64 snum = totalSamples_ + i;
            sampleNumbers[static_cast<size_t>(i)] = snum;
            timestamps[static_cast<size_t>(i)]    = ts0_s + i * dt_s;
            eventCodes[static_cast<size_t>(i)]    = 0;
        }

        // 2) De-interleave 40ch x int16 LE -> pick 16 channels -> convert to uV
        const uint8* payload = raw.bytes.data() + TIMESTAMP_BYTES;
        constexpr int BYTES_PER_SAMPLE = TOTAL_HW_CH * 2;

        for (int i = 0; i < BLOCK_NSAMP; ++i) {
            const int sampleOffset = i * BYTES_PER_SAMPLE;
            for (int outCh = 0; outCh < NUM_CH; ++outCh) {
                const int hwCh      = CH_MAP[static_cast<size_t>(outCh)];
                const int byteIndex = sampleOffset + (hwCh * 2);
                const uint16 adc    = readLE16(payload + byteIndex);

                // Velec(AC) = 0.195 µV × (ADC – 32768)
                const int   centered = static_cast<int>(adc) - 32768;
                const float uV       = 0.195f * static_cast<float>(centered);

                samples[static_cast<size_t>(outCh) * BLOCK_NSAMP + static_cast<size_t>(i)] = uV;
            }
        }

        // 3) Publish to OE
        dataBuffer_->addToBuffer(samples.data(),
                                 sampleNumbers.data(),
                                 timestamps.data(),
                                 eventCodes.data(),
                                 BLOCK_NSAMP);

        totalSamples_ += BLOCK_NSAMP;
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

    if (dataBuffer_ != nullptr)
        dataBuffer_->clear();

    return true;
}
// ==============================================================================================================================








// ===================================================== Serial I/O thread  =====================================================
void DataThreadPlugin::serialLoop()
{
    // Seek 0xAA, then read 8004 bytes and push to queue.
    uint8 b = 0;

    while (serialRunning_.load())
    {
        // 1) Find sync byte 0xAA (read 1 byte at a time)
        do {
            serial_.readData(reinterpret_cast<char*>(&b), 1);
        } while (b != SYNC_BYTE && serialRunning_.load());

        if (!serialRunning_.load()) break;

        // 2) Acquire a raw slot, read 8004 bytes (timestamp + payload), enqueue
        int idx = queue_->acquireFreeBlocking(serialRunning_);
        if (idx < 0) break;
        auto& raw = queue_->at(idx);
        size_t got = 0;
        while (got < static_cast<size_t>(BLOCK_BYTES) && serialRunning_.load()) {
            long r = serial_.readData(reinterpret_cast<char*>(raw.bytes.data() + got),
                                     static_cast<size_t>(BLOCK_BYTES) - got);
            if (r > 0) got += static_cast<size_t>(r);
        }

        //push to ready queue if we got a full packet
        queue_->pushReady(idx);
    }
}
// ==============================================================================================================================








// ================================================= DataThreadPluginEditor impl ================================================
bool DataThreadPlugin::setSerialPort(const std::string& name)
{
    serialPort_ = name;
    return true;
}

bool DataThreadPlugin::setSampleRate(int hz)
{
    if (hz <= 0) return false;
    sampleRateHz_ = hz;
    return true;
}

bool DataThreadPlugin::setLowerBandwidthHz(double hz)
{
    if (hz <= 0.0) return false;
    lowerBwHz_ = hz;
    return true;
}

bool DataThreadPlugin::setUpperBandwidthHz(double hz)
{
    if (hz <= 0.0) return false;
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
    if (k < 0) return false;
    dspK_ = k;
    return true;
}

bool DataThreadPlugin::setAcquisitionTimeSeconds(int seconds)
{
    if (seconds < 0) return false;
    acquisitionTimeSec_ = seconds;
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
    if (!prepareSequenceHeader())  return false;   // folder + sequence.json + parse + count
    if (!loadAndApplyPreset(1))    return false;   // load + validate + apply preset #0

    //CoreServices::setAcquisitionStatus(true);
    CoreServices::setRecordingStatus(true);
    return true;
}


bool DataThreadPlugin::stopSequence()
{
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
    std::printf("\n\n\n[STM32-RHS2116] ============== Applying preset #%d: %s ==============\n\n", index, preset.getFileName().toRawUTF8());

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
