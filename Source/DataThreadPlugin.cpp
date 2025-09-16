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

#include "DataThreadPlugin.h"
#include "DataThreadPluginEditor.h"
#include "ofSerial.h"
// added by lorenzo clerici
#include <cmath>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

// ------------------------------------------------------------

struct PluginSettingsObject
{
    // Store settings for the plugin here
};

DataThreadPlugin::DataThreadPlugin (SourceNode* sn) : DataThread (sn)
{
// added by lorenzo clerici
for (int ch = 0; ch < NUM_CH; ++ch)
{
    freqHz_[ch] = 10.0;
    phase_[ch]  = 0.0;
}

    // added by lorenzo clerici
    // inizializza la coda pre-allocata
    queue_ = std::make_unique<PacketPoolQueue>(N_BLOCKS);
}

DataThreadPlugin::~DataThreadPlugin()
{
    // added by lorenzo clerici
    // assicura lo stop del producer
    producerRunning_.store(false);
    if (producerThread_.joinable()) producerThread_.join();
}

bool DataThreadPlugin::foundInputSource()
{
    return true;
}

void DataThreadPlugin::updateSettings (OwnedArray<ContinuousChannel>* continuousChannels,
                                       OwnedArray<EventChannel>* eventChannels,
                                       OwnedArray<SpikeChannel>* /*spikeChannels*/,
                                       OwnedArray<DataStream>* sourceStreams,
                                       OwnedArray<DeviceInfo>* /*devices*/,
                                       OwnedArray<ConfigurationObject>* /*configurationObjects*/)
{
    // added by lorenzo clerici
    // Pulisce vecchia configurazione
    continuousChannels->clear();
    eventChannels->clear();
    sourceStreams->clear();
    sourceBuffers.clear();     // membro protetto di DataThread (buffer interni)

    // added by lorenzo clerici
    // Crea DataStream a FS_HZ
    {
        DataStream::Settings ds {
            "sine_stream",
            "16 synthetic sine waves",
            "sine_stream_id",
            FS_HZ
        };
        stream_ = new DataStream(ds);
        sourceStreams->add(stream_);
    }

    // added by lorenzo clerici
    // Alloca DataBuffer associato allo stream (capienza ~ 1.6s @30 kHz)
    {
        const int internalCapacity = 30000000;
        sourceBuffers.add(new DataBuffer(NUM_CH, internalCapacity));
        dataBuffer_ = sourceBuffers.getLast();
    }

    // added by lorenzo clerici
    // Registra 16 canali continui (bitVolts=1: forniremo già µV)
    for (int ch = 0; ch < NUM_CH; ++ch)
    {
        ContinuousChannel::Settings cs{
            ContinuousChannel::Type::ELECTRODE,
            "CH" + String(ch + 1),
            "synthetic sine",
            "ch" + String(ch + 1),
            1.0,        // bitVolts
            stream_
        };
        continuousChannels->add(new ContinuousChannel(cs));
    }

    // added by lorenzo clerici
    // (Opzionale) un canale eventi TTL
    {
        EventChannel::Settings es{
            EventChannel::Type::TTL,
            "TTL",
            "synthetic events",
            "ttl0",
            stream_,
            8
        };
        eventChannels->add(new EventChannel(es));
    }

    // added by lorenzo clerici
    // reset generatori
    totalSamples_ = 0;
    std::fill(phase_.begin(), phase_.end(), 0.0);
    if (queue_) queue_->reset();
}

bool DataThreadPlugin::startAcquisition()
{
    // added by lorenzo clerici
    // reset contatori
    totalSamples_ = 0;

    // added by lorenzo clerici
    // avvia il producer dedicato
    producerRunning_.store(true);
    producerThread_ = std::thread(&DataThreadPlugin::producerLoop, this);

    // avvia il thread di lettura seriale
    serialRunning_.store(true);
    serialThread_ = std::thread(&DataThreadPlugin::serialLoop, this);

    // added by lorenzo clerici
    // avvia il thread del DataThread (consumer)
    startThread();  // <--- fondamentale
    return true;
}

bool DataThreadPlugin::updateBuffer()
{
    // added by lorenzo clerici
    if (dataBuffer_ == nullptr || stream_ == nullptr)
        return false; 

    // added by lorenzo clerici
    // Legge SOLO dalla coda e invia alla GUI
    int drained = 0;
    int idx = -1;
    while (drained < MAX_DRAIN_PER_CALL && queue_->tryPopReady(idx))
    {
        auto& pkt = queue_->at(idx);
        dataBuffer_->addToBuffer(pkt.samples.data(),
                                 pkt.sampleNumbers.data(),
                                 pkt.timestamps.data(),
                                 pkt.eventCodes.data(),
                                 SAMPLES_CB);
        queue_->releaseFree(idx);
        ++drained;
    }
    return true;
}

bool DataThreadPlugin::stopAcquisition()
{
    // added by lorenzo clerici
    // stop producer
    producerRunning_.store(false);
    if (producerThread_.joinable()) producerThread_.join();

    // stop serial reader
    serialRunning_.store(false);
    if (serialThread_.joinable()) serialThread_.join();


    // added by lorenzo clerici
    // stop consumer (DataThread)
    if (isThreadRunning())
        signalThreadShouldExit();
    waitForThreadToExit(500);

    if (dataBuffer_ != nullptr)
        dataBuffer_->clear();

    return true;
}

void DataThreadPlugin::resizeBuffers()
{
    // (nessuna azione richiesta per questo generatore)
}

std::unique_ptr<GenericEditor> DataThreadPlugin::createEditor (SourceNode* sn)
{
    std::unique_ptr<DataThreadPluginEditor> editor = std::make_unique<DataThreadPluginEditor> (sn, this);
    return editor;
}

void DataThreadPlugin::handleBroadcastMessage (const String& /*msg*/, const int64 /*messageTimestmpMilliseconds*/)
{
    // (non utilizzato)
}

String DataThreadPlugin::handleConfigMessage (const String& /*msg*/)
{
    return "";
}

void DataThreadPlugin::registerParameters()
{
    // Register parameters for the plugin here (e.g. addParameter())
}

void DataThreadPlugin::parameterValueChanged (Parameter* /*parameter*/)
{
    // Handle parameter value changes here (e.g. update settings)
}


// added by lorenzo clerici
// ---------------- Producer thread ----------------
void DataThreadPlugin::producerLoop()
{ 
    using clock = std::chrono::steady_clock;
    const auto blockPeriod = std::chrono::duration<double>(SAMPLES_CB / FS_HZ);
    auto nextT = clock::now();

    while (producerRunning_.load())
    {
        // acquisisce uno slot libero del pool (bloccante ma uscibile)
        int idx = queue_->acquireFreeBlocking(producerRunning_);
        if (idx < 0) break;

        auto& pkt = queue_->at(idx);

        // prepara sampleNumbers e timestamps
        for (int i = 0; i < SAMPLES_CB; ++i) {
            const int64 snum = totalSamples_ + i;
            pkt.sampleNumbers[i] = snum;
            pkt.timestamps[i]    = static_cast<double>(snum) / FS_HZ;
            pkt.eventCodes[i]    = 0;
        }

        // genera sinusoidi in formato channel-major 
        for (int ch = 0; ch < NUM_CH; ++ch) {
            const double dphi = 2.0 * M_PI * freqHz_[ch] / FS_HZ;
            double phi = phase_[ch];
            const int base = ch * SAMPLES_CB;
            for (int i = 0; i < SAMPLES_CB; ++i) {
                pkt.samples[base + i] = AMP_UV * std::sin(phi);
                phi += dphi;
                if (phi >= 2.0 * M_PI) phi -= 2.0 * M_PI;
            }
            phase_[ch] = phi;
        } 

        totalSamples_ += SAMPLES_CB; 

        // pubblica il blocco pronto
        queue_->pushReady(idx);

        // pacing temporale per evitare overflow della coda
        nextT += std::chrono::duration_cast<clock::duration>(blockPeriod);
        std::this_thread::sleep_until(nextT);
    }
}

void DataThreadPlugin::serialLoop()
{
    ofSerial serial;
    serial.setup("COM12", 115200);


    const char* buf1 = "SAMPLERATE:30000\n";
    serial.writeData(buf1, strlen(buf1));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    const char* buf2 = "CONFIG\n";
    serial.writeData(buf2, strlen(buf2));
    std::this_thread::sleep_for(std::chrono::seconds(1));

    const char* buf3 = "START\n";
    serial.writeData(buf3, strlen(buf3));

    const int BUFSIZE = 8005;
    char buffer[BUFSIZE];

    while (true) {
        serial.readData(buffer, BUFSIZE);
    }
}