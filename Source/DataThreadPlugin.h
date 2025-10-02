/*
 * Project : BRAIN PLUS
 * File    : DataThreadPlugin.h
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

#ifndef DATATHREADPLUGIN_H_DEFINED
#define DATATHREADPLUGIN_H_DEFINED

#include <DataThreadHeaders.h>
#include <array>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>
#include <string>
#include "ofSerial.h"
#include "IntanRHS2116.h"

class ofSerial;
class DataThreadPluginEditor;

class DataThreadPlugin : public DataThread
{
public:
    DataThreadPlugin (SourceNode* sn);
    ~DataThreadPlugin() override;

    bool foundInputSource() override;

    void updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                         OwnedArray<EventChannel>* eventChannels,
                         OwnedArray<SpikeChannel>* spikeChannels,
                         OwnedArray<DataStream>* sourceStreams,
                         OwnedArray<DeviceInfo>* devices,
                         OwnedArray<ConfigurationObject>* configurationObjects) override;

    bool startAcquisition() override;
    bool updateBuffer() override;
    bool stopAcquisition() override;

    void resizeBuffers() override;
    std::unique_ptr<GenericEditor> createEditor (SourceNode* sn) override;
    void handleBroadcastMessage (const String& msg, const int64 messageTimeMilliseconds) override;
    String handleConfigMessage (const String& msg) override;
    void registerParameters() override;
    void parameterValueChanged (Parameter* parameter) override;
    bool setSerialPort(const std::string& name);

    // -------- Runtime configuration setters (store-only; applied on start) --------
    bool setSampleRate(int hz);
    bool setLowerBandwidthHz(double hz);
    bool setUpperBandwidthHz(double hz);
    bool setDspEnabled(bool enabled);
    bool setDspKFactor(int k);
    bool setAcquisitionTimeSeconds(int seconds);
    bool setPresetFolderPath(const std::string& path);
    bool startSequence();
    bool stopSequence();
    bool setStimEnabled(bool v);
    bool setStimVoltage(double v);
    bool setStimStepNa(int v);
    bool setStimPosCurrent(int v);
    bool setStimNegCurrent(int v);
    bool setStimType(int v);
    bool setStimPolarity(int v);
    bool setStimClkPos(int v);
    bool setStimClkNeg(int v);
    bool setStimContinuous(int v);
    bool setChargeRecoveryEnable(int v);
    bool setChargeRecoveryClk(int v);
    bool setStimulationTimeMs(int v);
    struct StimCmd { int mode; int ch1; int ch2; };
    bool setStimSequence(const std::vector<StimCmd>& seq);


private:
    // ===================== Hardware / packet layout =========================
    static constexpr double FS_HZ           = 30000.0;  // device sampling rate (Hz)
    static constexpr int    BLOCK_NSAMP     = 100;      // samples per device packet
    static constexpr int    TOTAL_HW_CH     = 40;       // 40 × 16-bit channels per sample
    static constexpr int    TIMESTAMP_BYTES = 4;        // u32 little-endian
    static constexpr int    PAYLOAD_BYTES   = 8000;     // 100 * 40 * 2
    static constexpr int    BLOCK_BYTES     = TIMESTAMP_BYTES + PAYLOAD_BYTES; // 8004 (no sync)
    static constexpr uint8  SYNC_BYTE       = 0xAA;     // start-of-packet marker
    static constexpr double TS_TICK_US      = 100.0;    // device timestamp resolution (100 us)
    static constexpr int MAX_DRAIN_PER_CALL = 8; // parse/push up to N raw blocks per updateBuffer()
    static constexpr int N_BLOCKS = 256;         // pool size for raw blocks

    // ===================== Producer–consumer data structures =================
    struct RawBlock {
        std::array<uint8, BLOCK_BYTES> bytes{};
    };

    class PacketPoolQueue {
    public:
        explicit PacketPoolQueue(int n);
        int  acquireFreeBlocking(std::atomic_bool& stopFlag);
        void pushReady(int idx);
        bool tryPopReady(int& idx);
        void releaseFree(int idx);
        RawBlock& at(int idx) { return storage_[static_cast<size_t>(idx)]; }
        void reset();
        int  capacity() const { return static_cast<int>(storage_.size()); }
        int  readySize() const;
    private:
        std::vector<RawBlock> storage_;
        mutable std::mutex m_;
        std::condition_variable cvFree_, cvReady_;
        std::deque<int> free_, ready_;
    };

    // ===================== Threads =====================
    void serialLoop();                // I/O thread: frame on 0xAA and enqueue 8004B raw
    std::thread serialThread_;
    std::atomic_bool serialRunning_{false};

    std::unique_ptr<PacketPoolQueue> queue_;

    // ===================== Open Ephys integration =====================
    DataBuffer* dataBufferAC_  = nullptr;
    DataBuffer* dataBufferDC_  = nullptr;
    DataStream* streamAC_      = nullptr;
    DataStream* streamDC_      = nullptr;

    // AC scale: we push microvolts (already in code)
    static constexpr float AC_UV_PER_LSB = 0.195f; // uV

    // DC (10-bit) scale: we push millivolts
    static constexpr int   DC_CENTER_10B = 512;
    static constexpr uint16 DC_MASK_10B  = 0x03FF;
    static constexpr float DC_UV_PER_LSB = -19230.0f; 
    
    // ===================== Channel mapping =====================
    static constexpr int NUM_CH = 16;

    // 16-bit AC channel indices
    static constexpr std::array<int, NUM_CH> CH_MAP = {
        5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35
    };

    // 10-bit DC channel indices
    static constexpr std::array<int, NUM_CH> DC10_MAP = {
        4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34
    };


    int64 totalSamples_ = 0;          // monotonic sample counter for sampleNumbers[]

    int         serialBaud_ = 115200;
    // --------------------- Runtime config (applied on start) ---------------------
    std::string serialPort_ = "COM3";
    int    sampleRateHz_ = 0;
    double lowerBwHz_    = 0.0;
    double upperBwHz_    = 0.0;
    bool   dspEnabled_   = false;
    int    dspK_         = 0;
    // --- Stimulation config ---
    bool  stimEnabled_     = false;
    double stimVoltageV_   = 5.0;
    int    stimStepNa_     = 10;
    int    stimPosCurrent_ = 128;
    int    stimNegCurrent_ = 128;
    int    stimType_       = 0;   // 0/1
    int    stimPolarity_   = 1;   // 0/1
    int    stimClkPos_     = 6;
    int    stimClkNeg_     = 6;
    int    stimContinuous_ = 0;   // 0/1
    int    crEnable_       = 0;   // 0/1
    int    crClk_          = 1;

    int    stimTimeMs_     = 0;   // 0 = off

    
    static constexpr int kMaxStimSeq = 64;
    std::vector<StimCmd> stimSeq_;   // bounded to kMaxStimSeq
    size_t stimSeqIdx_ = 0;


    int  acquisitionTimeSec_ = 0;
    std::string presetFolderPath_;
    std::atomic_bool sequenceRunning_{false};
    std::thread sequenceThread_;

    // ===================== Helpers: little-endian readers =====================
    static inline uint16 readLE16(const uint8* p) {
        return static_cast<uint16>(p[0]) | (static_cast<uint16>(p[1]) << 8);
    }
    static inline uint32 readLE32(const uint8* p) {
        return  static_cast<uint32>(p[0])
              | (static_cast<uint32>(p[1]) << 8)
              | (static_cast<uint32>(p[2]) << 16)
              | (static_cast<uint32>(p[3]) << 24);
    }

    // ===================== Low-level serial I/O =====================
    ofSerial serial_;
    std::unique_ptr<IntanRHS2116> rhs_;

    // ===================== Editor =====================
    DataThreadPluginEditor* editor_ = nullptr;

    // --- sequence helpers ---
    bool prepareSequenceHeader();       
    bool loadAndApplyPreset(int index); 
    void presetSequenceThread();
};

#endif // DATATHREADPLUGIN_H_DEFINED
