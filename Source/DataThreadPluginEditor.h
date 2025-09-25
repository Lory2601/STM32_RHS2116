/*
 * Project : BRAIN PLUS
 * File    : DataThreadPluginEditor.h
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

#ifndef DATATHREADPLUGINEDITOR_H_DEFINED
#define DATATHREADPLUGINEDITOR_H_DEFINED

#pragma once

#include <EditorHeaders.h>
#include "DataThreadPlugin.h"
#include "ofSerial.h" 


// ================= Default values =================
// Default sample rate [Hz]
#define DEFAULT_SAMPLE_RATE  30000  

// Default upper bandwidth [Hz]
#define DEFAULT_UPPER_BW     3  

// Default lower bandwidth [Hz]
#define DEFAULT_LOWER_BW     15

// Default DSP enable (0=off, 1=on)
#define DSP_DEFAULT_ENABLE  1

// === Add after existing defaults ===
#define DEFAULT_DSP_N       12

// kfreq lookup table (index = N; 0 is differentiator)
static constexpr double DSP_K_TABLE[16] = {
    0.0,        // N=0 -> differentiator (no real cutoff)
    0.1103,     // 1
    0.04579,    // 2
    0.02125,    // 3
    0.01027,    // 4
    0.005053,   // 5
    0.002506,   // 6
    0.001248,   // 7
    0.0006229,  // 8
    0.0003112,  // 9
    0.0001555,  // 10
    0.00007773, // 11
    0.00003886, // 12
    0.00001943, // 13
    0.000009714,// 14
    0.000004857 // 15
};


class DataThreadPluginEditor : public GenericEditor
{
public:
    DataThreadPluginEditor (GenericProcessor* parentNode, DataThreadPlugin* plugin);
    ~DataThreadPluginEditor() override = default;

    void resized() override;

private:
    DataThreadPlugin* thread = nullptr;
    struct RefreshingComboBox : public juce::ComboBox
    {
        std::function<void()> beforePopup;

        // Refresh the items on click
        void mouseDown (const juce::MouseEvent& e) override
        {
            if (beforePopup) beforePopup();
            juce::ComboBox::mouseDown(e);
        }
    } comBox;

    // Serial port handler
    ofSerial serial;

    // Serial port label
    juce::Label portLabel;

    // Sample rate controls
    juce::Label sampleRateLabel;
    juce::ComboBox sampleRateBox;

    // upper Bandwidth controls
    juce::Label upperBwLabel;
    juce::ComboBox upperBwBox;

    // lower Bandwidth controls
    juce::Label lowerBwLabel;
    juce::ComboBox lowerBwBox;

    // DSP enable controls
    juce::Label     dspEnableLabel;
    juce::TextButton dspEnableButton;

    // DSP cutoff controls
    juce::Label   dspFreqLabel;
    juce::ComboBox dspFreqBox;

    // --- Preset folder UI ---
    juce::Label     presetFolderLabel;
    juce::TextButton presetFolderBox;  
    juce::File      presetBaseDir;    
    
    
    // --- Preset import helpers ---
    bool loadPresetFile (const juce::File& f);
    bool applyPresetObject (const juce::var& root);
    bool loadSequenceFirst (const juce::File& baseDir);

    // Rebuilds the DSP frequency dropdown according to the current sample rate
    void rebuildDspFreqItems (int fsample);
};


#endif