/*
 * Project : BRAIN PLUS
 * File    : DataThreadPluginEditor.cpp
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

#include "DataThreadPluginEditor.h"
#include <iostream>
#include <cstdio>

// ========================================== DataThreadPluginEditor impl ==========================================
DataThreadPluginEditor::DataThreadPluginEditor (GenericProcessor* parentNode, DataThreadPlugin* plugin)
    : GenericEditor (parentNode), thread(plugin)
{
    // Set size of the editor window
    desiredWidth = 430;

    //-------------------------------------------------- serial port -----------------------------------------------
    // Port label
    portLabel.setText("Serial port:", juce::dontSendNotification);
    portLabel.setFont(juce::Font(12.0f));
    portLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(portLabel);

    // ComboBox for serial ports
    comBox.setTextWhenNoChoicesAvailable("No serial ports found");
    comBox.setTextWhenNothingSelected("Select");

    // Auto-refresh ports when the user clicks the drop-down
    comBox.beforePopup = [this]()
    {
        comBox.clear(juce::dontSendNotification);
        auto devices = serial.getDeviceList(); // requires ofSerial.h/.cpp
        int id = 1;
        for (auto& d : devices)
        {
            comBox.addItem (juce::String(d.getDevicePath().c_str()), id++);
        }
    };

    addAndMakeVisible(comBox);

    comBox.onChange = [this]()
    {
        auto selected = comBox.getText();
        if (selected.isNotEmpty())
        {
            thread->setSerialPort(selected.toStdString());
            std::cout << "[STM32-RHS2116] Selected port: " << selected << "\n";
        }
    };
    //--------------------------------------------------------------------------------------------------------------





    //-------------------------------------------------- sample rate -----------------------------------------------
    sampleRateLabel.setText("Sample rate [Hz]:", juce::dontSendNotification);
    sampleRateLabel.setFont(juce::Font(12.0f));
    sampleRateLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(sampleRateLabel);

    // ComboBox for sample rates
    for (int rate = 1000; rate <= 30000; rate += 1000)
    {
        sampleRateBox.addItem(juce::String(rate), rate);
    }
    sampleRateBox.setSelectedId(1000, juce::dontSendNotification);

    addAndMakeVisible(sampleRateBox);

    sampleRateBox.onChange = [this]()
    {
        int selected = sampleRateBox.getText().getIntValue();
        if (selected > 0)
        {
            std::cout << "[STM32-RHS2116] Selected sample rate: " << selected << " Hz\n";

            thread->setSampleRate(selected);

            // Rebuild DSP frequency list to reflect new fs
            const int prevN = dspFreqBox.getSelectedId();
            rebuildDspFreqItems(selected);
            if (prevN >= 1 && prevN <= 15)
                dspFreqBox.setSelectedId(prevN, juce::dontSendNotification);
            else
                dspFreqBox.setSelectedId(DEFAULT_DSP_N, juce::dontSendNotification);
        }
    };
    // Set default sample rate
    sampleRateBox.setSelectedId(DEFAULT_SAMPLE_RATE, juce::dontSendNotification);
    //--------------------------------------------------------------------------------------------------------------





    //------------------------------------------------- lower bandwidth --------------------------------------------
    // Label for lower bandwidth
    lowerBwLabel.setText("Lower BW [Hz]:", juce::dontSendNotification);
    lowerBwLabel.setFont(juce::Font(12.0f));
    lowerBwLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(lowerBwLabel);

    // ComboBox for lower bandwidth
    {
        int id = 1;
        const char* LOWER_BW_VALUES[] = {
            "1000", "500", "300", "250", "200", "150", "100", "75", "50", "30",
            "25", "20", "15", "10", "7.5", "5.0", "3.0", "2.5", "2.0", "1.5", "1.0",
            "0.75", "0.50", "0.30", "0.25", "0.10"
        };
        for (const char* v : LOWER_BW_VALUES)
            lowerBwBox.addItem(juce::String(v), id++);
    }
    addAndMakeVisible(lowerBwBox);

    lowerBwBox.onChange = [this]()
    {
        const double hz = lowerBwBox.getText().getDoubleValue();
        if (hz > 0.0)
        {
            std::cout << "[STM32-RHS2116] Selected lower bandwidth: " << hz << " Hz\n";

            thread->setLowerBandwidthHz(hz);
        }
    };

    // Set a default (first item in the list)
    lowerBwBox.setSelectedItemIndex(DEFAULT_LOWER_BW, juce::dontSendNotification);
    //--------------------------------------------------------------------------------------------------------------






    //------------------------------------------------- upper bandwidth --------------------------------------------
    // Label for upper bandwidth
    upperBwLabel.setText("Upper BW [Hz]:", juce::dontSendNotification);
    upperBwLabel.setFont(juce::Font(12.0f));
    upperBwLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(upperBwLabel);

    // ComboBox for upper bandwidth
    {
        int id = 1; 
        const char* UPPER_BW_VALUES[] = {
            "20000", "15000", "10000", "7500", "5000", "3000", "2500", "2000",
            "1500", "1000", "750", "500", "300", "250", "200", "150", "100"
        };
        for (const char* v : UPPER_BW_VALUES)
            upperBwBox.addItem(juce::String(v), id++);
    }
    addAndMakeVisible(upperBwBox);

    // When selection changes, parse the text as double and (optionally) pass it to the thread
    upperBwBox.onChange = [this]()
    {
        const double hz = upperBwBox.getText().getDoubleValue();
        if (hz > 0.0)
        {
            std::cout << "[STM32-RHS2116] Selected upper bandwidth: " << hz << " Hz\n";

            thread->setUpperBandwidthHz(hz);
        }
    };

    // Set a default (first item in the list)
    upperBwBox.setSelectedItemIndex(DEFAULT_UPPER_BW, juce::dontSendNotification);
    //--------------------------------------------------------------------------------------------------------------



    //---------------------------------------------------- dsp enable ----------------------------------------------
    // Label
    dspEnableLabel.setText("DSP enable:", juce::dontSendNotification);
    dspEnableLabel.setFont(juce::Font(12.0f));
    dspEnableLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(dspEnableLabel);

    // Pulsante
    dspEnableButton.setButtonText("DSP EN");
    dspEnableButton.setClickingTogglesState(true);
    dspEnableButton.setToggleState(DSP_DEFAULT_ENABLE, juce::dontSendNotification);

    // Arancione quando abilitato
    dspEnableButton.setColour(juce::TextButton::buttonOnColourId, juce::Colours::orange);

    addAndMakeVisible(dspEnableButton);

    dspEnableButton.onClick = [this]()
    {
        const bool enabled = dspEnableButton.getToggleState();
        if (thread != nullptr){
        thread->setDspEnabled(enabled);
        std::cout << "[STM32-RHS2116] DSP " << (enabled ? "ENABLED" : "DISABLED") << "\n";
        }
    };
    //--------------------------------------------------------------------------------------------------------------



    //--------------------------------------------------- dsp frequency --------------------------------------------
    // Label
    dspFreqLabel.setText("DSP freq [Hz]:", juce::dontSendNotification);
    dspFreqLabel.setFont(juce::Font(12.0f));
    dspFreqLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(dspFreqLabel);

    // Combo for computed cutoff frequencies (depends on sample rate)
    addAndMakeVisible(dspFreqBox);

    // Populate now based on current sample rate
    {
        const int fs = sampleRateBox.getText().getIntValue();
        rebuildDspFreqItems(fs);
        dspFreqBox.setSelectedId(DEFAULT_DSP_N, juce::dontSendNotification);
    }

    // When user changes selection, print fc and k, and pass only K to the thread
    dspFreqBox.onChange = [this]()
    {
        const int fs  = sampleRateBox.getText().getIntValue();
        const int N   = dspFreqBox.getSelectedId();
        if (N >= 1 && N <= 15)
        {
            const double k  = DSP_K_TABLE[N];
            const double fc = k * static_cast<double>(fs);

            std::cout << "[STM32-RHS2116] Selected DSP cutoff: " << fc << " Hz (k=" << k << ", N=" << N << ")\n";

            if (thread != nullptr)
            {
                // Pass only the K factor to the processing thread
                thread->setDspKFactor(N);
            }
        }
    };
    //--------------------------------------------------------------------------------------------------------------



    //--------------------------------------------------- preset folder --------------------------------------------

    presetFolderLabel.setText("Load preset folder:", juce::dontSendNotification);
    presetFolderLabel.setFont(juce::Font(12.0f));
    presetFolderLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(presetFolderLabel);

    presetFolderBox.setButtonText("Select");
    presetFolderBox.setColour(juce::TextButton::buttonColourId, juce::Colours::lightgrey);
    presetFolderBox.setColour(juce::TextButton::textColourOffId, juce::Colours::black);
    presetFolderBox.setTriggeredOnMouseDown(false);
    addAndMakeVisible(presetFolderBox);

    presetFolderBox.onClick = [this]()
    {
        juce::File startDir = presetBaseDir.exists() ? presetBaseDir
                                                    : juce::File::getSpecialLocation(juce::File::userDocumentsDirectory);

        juce::FileChooser chooser ("Select preset folder", startDir, juce::String());
        if (chooser.browseForDirectory())
        {
            presetBaseDir = chooser.getResult();

            // Show only the folder name on the 80x20 box; full path can go to a tooltip
            presetFolderBox.setButtonText(presetBaseDir.getFileName());
            presetFolderBox.setTooltip(presetBaseDir.getFullPathName());

            // Try to load the first sequence file in the folder
            loadSequenceFirst(presetBaseDir);

            //printf
            std::cout << "[STM32-RHS2116] Selected preset folder: " << presetBaseDir.getFullPathName() << "\n";

            //if (thread != nullptr)
                //thread->setPresetFolderPath(presetBaseDir.getFullPathName().toStdString());
        }
    };

//--------------------------------------------------------------------------------------------------------------

    // ---- Sync plugin with GUI defaults (avoid any mismatch at startup) ----
    if (thread != nullptr)
    {
        const int    fs     = sampleRateBox.getText().getIntValue();
        const double lbwHz  = lowerBwBox.getText().getDoubleValue();
        const double ubwHz  = upperBwBox.getText().getDoubleValue();
        const bool   dspOn  = dspEnableButton.getToggleState();
        const int N_as_K = dspFreqBox.getSelectedId();

        thread->setSampleRate(fs);
        thread->setLowerBandwidthHz(lbwHz);
        thread->setUpperBandwidthHz(ubwHz);
        thread->setDspEnabled(dspOn);
        thread->setDspKFactor(N_as_K);
    }

}
// =================================================================================================================



void DataThreadPluginEditor::rebuildDspFreqItems (int fsample)
{
    // Rebuild the dropdown with frequencies -> kfreq * fsample.
    dspFreqBox.clear(juce::dontSendNotification);

    // Simple formatting rule: more decimals for lower frequencies
    auto fmtHz = [] (double f) -> juce::String
    {
        if      (f < 10.0)  return juce::String(f, 3);
        else if (f < 100.0) return juce::String(f, 2);
        else                return juce::String(f, 0);
    };

    for (int N = 1; N <= 15; ++N)
    {
        const double k  = DSP_K_TABLE[N];
        const double fc = k * static_cast<double>(fsample);
        // Show only the frequency as requested; internally we keep N as the item ID
        dspFreqBox.addItem(fmtHz(fc), N);
    }
}

// ================================ Preset import implementation =================================

// Load and parse a JSON preset file, then apply it.
bool DataThreadPluginEditor::loadPresetFile (const juce::File& f)
{
    if (!f.existsAsFile()) {
        std::printf("[STM32-RHS2116] Preset file not found: %s\n", f.getFullPathName().toRawUTF8());
        return false;
    }

    juce::String content = f.loadFileAsString();
    juce::var root;
    if (!juce::JSON::parse(content, root).wasOk() || !root.isObject())
    {
        std::printf("[STM32-RHS2116] Invalid preset JSON: %s\n", f.getFileName().toRawUTF8());
        return false;
    }

    const bool ok = applyPresetObject(root);
    if (ok) {
        std::printf("[STM32-RHS2116] Preset applied: %s\n", f.getFileName().toRawUTF8());
        std::fflush(stdout);
    }
    return ok;
}

// Apply a parsed JSON object to GUI and backend.
// Expected keys: samplerate, lowerbandwidth, upperbandwidth, dspenabled, dspn
bool DataThreadPluginEditor::applyPresetObject (const juce::var& root)
{
    auto getNum = [&root](const char* key, double& out)->bool {
        if (auto* obj = root.getDynamicObject())
        {
            juce::var v = obj->getProperty(key);
            if (v.isDouble() || v.isInt()) { out = static_cast<double>(v); return true; }
        }
        return false;
    };
    auto getBool = [&root](const char* key, bool& out)->bool {
        if (auto* obj = root.getDynamicObject())
        {
            juce::var v = obj->getProperty(key);
            if (v.isBool()) { out = static_cast<bool>(v); return true; }
            if (v.isInt())  { out = (static_cast<int>(v) != 0); return true; }
        }
        return false;
    };

    double fs = 0.0, lbw = 0.0, ubw = 0.0; bool dspEn = false; double dspN_d = 0.0;

    const bool hasFs   = getNum("samplerate",     fs);
    const bool hasLbw  = getNum("lowerbandwidth", lbw);
    const bool hasUbw  = getNum("upperbandwidth", ubw);
    const bool hasDspE = getBool("dspenabled",    dspEn);
    const bool hasDspN = getNum("dspn",           dspN_d);

    if (!(hasFs && hasLbw && hasUbw)) {
        std::printf("Preset missing required keys\n");
        return false;
    }

    // 1) Sample rate
    const int fs_i = static_cast<int>(fs);
    sampleRateBox.setSelectedId(fs_i, juce::dontSendNotification); // id==rate in your setup
    if (thread) thread->setSampleRate(fs_i);

    // Rebuild DSP freq list because it depends on fs
    rebuildDspFreqItems(fs_i);

    // 2) Bandwidths
    auto round2 = [](double v) noexcept { return std::round(v * 100.0) / 100.0; };
    auto round0 = [](double v) noexcept { return std::round(v); };

    const double lbw2 = round2(lbw);
    const double ubw0 = round0(ubw);

    // UI: lower with 2 decimals, upper with no decimals
    lowerBwBox.setText(juce::String(lbw2, 2), juce::dontSendNotification);
    upperBwBox.setText(juce::String(ubw0, 0), juce::dontSendNotification);

    // Backend: send the rounded values for consistency
    if (thread)
    {
        thread->setLowerBandwidthHz(lbw2);
        thread->setUpperBandwidthHz(ubw0);
    }

    // 3) DSP enable
    if (hasDspE) {
        dspEnableButton.setToggleState(dspEn, juce::dontSendNotification);
        if (thread) thread->setDspEnabled(dspEn);
    }

    // 4) DSP N
    if (hasDspN) {
        const int N = juce::jlimit(1, 15, static_cast<int>(dspN_d));
        dspFreqBox.setSelectedId(N, juce::dontSendNotification);
        if (thread) thread->setDspKFactor(N);
        // Optional debug print of computed cutoff
        const double k  = DSP_K_TABLE[N];
        const double fc = k * static_cast<double>(fs_i);
        std::printf("[STM32-RHS2116] DSP cutoff from preset: %.6f Hz (k=%.6g, N=%d)\n", fc, k, N);
    }


    // Force a repaint
    repaint();
    return true;
}

// Load sequence.json
bool DataThreadPluginEditor::loadSequenceFirst (const juce::File& baseDir)
{
    const juce::File seq = baseDir.getChildFile("sequence.json");
    if (!seq.existsAsFile()) {
        std::printf("No sequence.json in %s\n", baseDir.getFullPathName().toRawUTF8());
        return false;
    }

    juce::String content = seq.loadFileAsString();
    juce::var root;
    auto res = juce::JSON::parse(content, root);
    if (!res.wasOk()) {
        std::printf("Invalid sequence.json\n");
        return false;
    }

    // ["preset1.json", "preset2.json", ...]
    if (root.isArray() && root.getArray()->size() > 0)
    {
        const juce::var& first = root.getArray()->getReference(0);
        if (first.isString())
        {
            const juce::File preset = baseDir.getChildFile(first.toString());
            return loadPresetFile(preset);
        }
    }
    std::printf("sequence.json has no valid preset entries\n");
    return false;
}


// ======================================== DataThreadPluginEditor::resized() ======================================
void DataThreadPluginEditor::resized()
{
    // x, y, larghezza, altezza

    //serial port
    portLabel.setBounds(5, 20, 120, 40);
    comBox.setBounds(125, 30, 80, 20);

    // sample rate
    sampleRateLabel.setBounds(215, 30, 125, 20);
    sampleRateBox.setBounds(340, 30, 80, 20);

    // lower bandwidth
    lowerBwLabel.setBounds(5, 45, 120, 40);
    lowerBwBox.setBounds(125, 55, 80, 20);

    // upper bandwidth
    upperBwLabel.setBounds(215, 45, 120, 40);
    upperBwBox.setBounds(340, 55, 80, 20);

    // dsp enable
    dspEnableLabel.setBounds(5, 70, 120, 40);
    dspEnableButton.setBounds(125, 80, 80, 20);

    // dsp freq
    dspFreqLabel.setBounds(215, 70, 120, 40);
    dspFreqBox.setBounds(340, 80, 80, 20);

    // preset folder (new row)
    presetFolderLabel.setBounds(5, 95, 120, 40);
    presetFolderBox.setBounds(125, 105, 80, 20);

}
// =================================================================================================================