/*
 ------------------------------------------------------------------

 This file is part of the Open Ephys GUI
 Copyright (C) 2022 Open Ephys

 ------------------------------------------------------------------

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

#ifndef DATATHREADPLUGIN_H_DEFINED
#define DATATHREADPLUGIN_H_DEFINED

#include <DataThreadHeaders.h>

// added by lorenzo clerici
#include <array>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <chrono>

class DataThreadPlugin : public DataThread
{
public:
    /** The class constructor, used to initialize any members. */
    DataThreadPlugin (SourceNode* sn);

    /** The class destructor, used to deallocate memory */
    ~DataThreadPlugin();

    // ------------------------------------------------------------
    //                  PURE VIRTUAL METHODS
    //     (must be implemented by all DataThreads)
    // ------------------------------------------------------------

    /** Returns true if the data source is connected, false otherwise.*/
    bool foundInputSource();

    /** Passes the processor's info objects to DataThread, to allow them to be configured */
    void updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                         OwnedArray<EventChannel>* eventChannels,
                         OwnedArray<SpikeChannel>* spikeChannels,
                         OwnedArray<DataStream>* sourceStreams,
                         OwnedArray<DeviceInfo>* devices,
                         OwnedArray<ConfigurationObject>* configurationObjects);

    /** Initializes data transfer.*/
    bool startAcquisition();

    /** Called repeatedly to add any available data to the buffer */
    bool updateBuffer();

    /** Stops data transfer.*/
    bool stopAcquisition();

    // ------------------------------------------------------------
    //                   VIRTUAL METHODS
    //       (can optionally be overriden by sub-classes)
    // ------------------------------------------------------------

    /** Called when the chain updates, to add, remove or resize the sourceBuffers' DataBuffers as needed */
    void resizeBuffers() override;

    /** Create the DataThread custom editor */
    std::unique_ptr<GenericEditor> createEditor (SourceNode* sn) override;

    /** Allows the DataThread plugin to respond to messages sent by other processors */
    void handleBroadcastMessage (const String& msg, const int64 messageTimeMilliseconds) override;

    /** Allows the DataThread plugin to handle a config message while acquisition is not active */
    String handleConfigMessage (const String& msg) override;

    /** Registers parameters to the DataThread */
    void registerParameters() override;

    /** Called when a parameter value is updated, to allow plugin-specific responses */    
    void parameterValueChanged (Parameter* parameter) override;

private:
    // added by lorenzo clerici
    static constexpr int   NUM_CH     = 16;        // numero di canali da generare
    static constexpr int   SAMPLES_CB = 256;       // campioni per callback
    static constexpr double FS_HZ     = 30000.0;   // sample rate dello stream
    static constexpr float AMP_UV     = 100.0f;    // ampiezza sinusoidi (µV)


    static constexpr int   N_BLOCKS   = 256;


    static constexpr int   MAX_DRAIN_PER_CALL = 8; // limita quanti blocchi drenare per chiamata di updateBuffer 


    std::array<double, NUM_CH> freqHz_ {};         // frequenze per canale (Hz)
    std::array<double, NUM_CH> phase_  {};         // fase corrente per canale (rad)
    int64 totalSamples_ = 0;                       // sample counter


    // —— nuova struttura dati per producer–consumer ——
    // Pacchetto di dati pronto per la GUI 
    struct DataPacket {
        std::array<float,  NUM_CH * SAMPLES_CB> samples; 
        std::array<int64,  SAMPLES_CB>          sampleNumbers; 
        std::array<double, SAMPLES_CB>          timestamps; 
        std::array<uint64, SAMPLES_CB>          eventCodes; 
    }; 


    // Coda con pool pre-allocato (bounded) 
    class PacketPoolQueue { 
    public: 
        explicit PacketPoolQueue(int n)
            : storage_(n)
        {
            for (int i = 0; i < n; ++i) free_.push_back(i);
        }

        int acquireFreeBlocking(std::atomic_bool& stopFlag) {
            std::unique_lock<std::mutex> lk(m_);
            cvFree_.wait(lk, [&]{ return !free_.empty() || stopFlag.load(); });
            if (stopFlag.load() && free_.empty()) return -1;
            int idx = free_.front();
            free_.pop_front();
            return idx;
        }

        void pushReady(int idx) {
            { std::lock_guard<std::mutex> lk(m_); ready_.push_back(idx); }
            cvReady_.notify_one();
        }

        bool tryPopReady(int& idx) {
            std::lock_guard<std::mutex> lk(m_);
            if (ready_.empty()) return false;
            idx = ready_.front();
            ready_.pop_front();
            return true;
        }

        void releaseFree(int idx) {
            { std::lock_guard<std::mutex> lk(m_); free_.push_back(idx); }
            cvFree_.notify_one();
        }

        DataPacket& at(int idx) { return storage_[idx]; } 

        void reset() {
            std::lock_guard<std::mutex> lk(m_);
            ready_.clear();
            free_.clear();
            for (int i = 0; i < (int)storage_.size(); ++i) free_.push_back(i);
        }

        int capacity() const { return (int)storage_.size(); }
        int readySize() const { std::lock_guard<std::mutex> lk(m_); return (int)ready_.size(); }

    private:
        std::vector<DataPacket> storage_;
        mutable std::mutex m_;
        std::condition_variable cvFree_;
        std::condition_variable cvReady_;
        std::deque<int> free_;
        std::deque<int> ready_;
    };

    // Thread del producer
    void producerLoop();

    // Thread di lettura seriale
    void serialLoop();
    std::thread serialThread_;
    std::atomic<bool> serialRunning_{false};

    std::unique_ptr<PacketPoolQueue> queue_;
    std::thread producerThread_;
    std::atomic_bool producerRunning_{false};

    DataBuffer* dataBuffer_ = nullptr;             // buffer associato allo stream
    DataStream* stream_     = nullptr;             // stream sorgente
};

#endif