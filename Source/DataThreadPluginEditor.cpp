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

            //printf
            std::cout << "[STM32-RHS2116] Selected preset folder: " << presetBaseDir.getFullPathName() << "\n";

            if (thread != nullptr)
                thread->setPresetFolderPath(presetBaseDir.getFullPathName().toStdString());

        }
    };
    //--------------------------------------------------------------------------------------------------------------

    // ----------------------------------------------- start sequence -------------------------------------------------
    startSeqLabel.setText("Start sequence:", juce::dontSendNotification);
    startSeqLabel.setFont(juce::Font(12.0f));
    startSeqLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(startSeqLabel);

    startSeqButton.setButtonText("START");
    startSeqButton.setClickingTogglesState(true);

    // grey when off, green when on
    startSeqButton.setColour(juce::TextButton::buttonColourId, juce::Colours::lightgrey);
    startSeqButton.setColour(juce::TextButton::buttonOnColourId, juce::Colours::green);
    startSeqButton.setColour(juce::TextButton::textColourOffId, juce::Colours::black);
    startSeqButton.setColour(juce::TextButton::textColourOnId, juce::Colours::white);

    addAndMakeVisible(startSeqButton);

    startSeqButton.onClick = [this]()
    {
        if (startSeqButton.getToggleState())
        {
            if (thread) thread->startSequence();
            std::printf("[STM32-RHS2116] Sequence started\n");
        }
        else
        {
            if (thread) thread->stopSequence();
            std::printf("[STM32-RHS2116] Sequence stopped\n");
        }
        std::fflush(stdout);
    };
    // ---------------------------------------------------------------------------------------------------------------




    // ------------------------ Sync plugin with GUI defaults (avoid any mismatch at startup) -----------------------
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
    //--------------------------------------------------------------------------------------------------------------

}
// =================================================================================================================


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

    // preset folder
    presetFolderLabel.setBounds(5, 95, 120, 40);
    presetFolderBox.setBounds(125, 105, 80, 20);

    // start sequence
    startSeqLabel.setBounds(215, 95, 120, 40);
    startSeqButton.setBounds(340, 105, 80, 20);

}
// =================================================================================================================




// ==================================Rebuilds the DSP frequency dropdown ==========================================
void DataThreadPluginEditor::rebuildDspFreqItems (int fsample)
{
    // Rebuild the dropdown with frequencies -> kfreq * fsample.
    dspFreqBox.clear(juce::dontSendNotification);

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
        dspFreqBox.addItem(fmtHz(fc), N);
    }
}
// ==================================================================================================================





// ========================================= Preset import implementation ===========================================

bool DataThreadPluginEditor::applyPresetObject (const juce::var& root)
{
    // Get double value from JSON
    auto getNum = [&root](const char* key, double& out)->bool {
        if (auto* obj = root.getDynamicObject())
        {
            juce::var v = obj->getProperty(key);
            if (v.isDouble() || v.isInt()) { out = static_cast<double>(v); return true; }
        }
        return false;
    };

    // Get boolean value from JSON
    auto getBool = [&root](const char* key, bool& out)->bool {
        if (auto* obj = root.getDynamicObject())
        {
            juce::var v = obj->getProperty(key);
            if (v.isBool()) { out = static_cast<bool>(v); return true; }
            if (v.isInt())  { out = (static_cast<int>(v) != 0); return true; }
        }
        return false;
    };

    // Get integer value from JSON
    auto getInt = [&root](const char* key, int& out)->bool {
        if (auto* obj = root.getDynamicObject())
        {
            juce::var v = obj->getProperty(key);
            if (v.isInt()) { out = static_cast<int>(v); return true; }
        }
        return false;
    };

    // Get array<array<int,3>> from JSON
    auto getStimSeq = [&root](std::vector<DataThreadPlugin::StimCmd>& out)->bool {
        if (auto* obj = root.getDynamicObject()) {
            juce::var v = obj->getProperty("stim_sequence");
            if (! v.isArray()) return false;
            auto* arr = v.getArray();
            out.clear();
            for (auto& row : *arr)
            {
                if (!row.isArray() || row.getArray()->size() < 3) continue;
                auto& a = *row.getArray();
                DataThreadPlugin::StimCmd c;
                c.mode = (int)a[0]; c.ch1 = (int)a[1]; c.ch2 = (int)a[2];
                out.push_back(c);
            }
            return true;
        }
        return false;
    };


    // --- acquisition
    double fs = 0.0, lbw = 0.0, ubw = 0.0; 
    bool dspEn = false; 
    double dspN_d = 0.0;
    int acquisitionTimeSec = 0;

    // --- stimulation
    int stim_enable      = 0;
    double stim_voltage  = 0;
    int stim_step_na     = 0;
    int stim_pos_current = 0;
    int stim_neg_current = 0;
    int stim_type        = 0;
    int stim_polarity    = 0;
    int stim_clk_pos     = 0;
    int stim_clk_neg     = 0;
    int stim_continuous  = 0;
    int cr_enable        = 0;
    int cr_clk           = 0;
    int stim_time_ms     = 0;

    const bool hasFs            = getNum("samplerate",     fs);
    const bool hasLbw           = getNum("lowerbandwidth", lbw);
    const bool hasUbw           = getNum("upperbandwidth", ubw);
    const bool hasDspE          = getBool("dspenabled",    dspEn);
    const bool hasDspN          = getNum("dspn",           dspN_d);
    const bool hasAcqT          = getInt("acquisitiontime", acquisitionTimeSec);
    const bool hasStimE         = getInt("stim_enable",       stim_enable);
    const bool hasStimV         = getNum("stim_voltage",      stim_voltage);
    const bool hasStimStep      = getInt("stim_step_size_na", stim_step_na);
    const bool hasStimPos       = getInt("stim_pos_current",  stim_pos_current);
    const bool hasStimNeg       = getInt("stim_neg_current",  stim_neg_current);
    const bool hasStimType      = getInt("stim_type",         stim_type);
    const bool hasStimPol       = getInt("stim_polarity",     stim_polarity);
    const bool hasStimClkPos    = getInt("stim_clk_pos",      stim_clk_pos);
    const bool hasStimClkNeg    = getInt("stim_clk_neg",      stim_clk_neg);
    const bool hasStimCont      = getInt("stim_continuous",   stim_continuous);
    const bool hasCrEnable      = getInt("cr_enable",         cr_enable);
    const bool hasCrClk         = getInt("cr_clk",            cr_clk);
    const bool hasStimTime      = getInt("stimulation_time_ms", stim_time_ms);

    // sequence
    std::vector<DataThreadPlugin::StimCmd> seq;
    getStimSeq(seq);

    if (!(hasFs && hasLbw && hasUbw && hasDspE && hasDspN && hasAcqT && hasStimE 
            && hasStimV && hasStimStep && hasStimPos && hasStimNeg && hasStimType 
            && hasStimPol && hasStimClkPos && hasStimClkNeg && hasStimCont && hasCrEnable 
            && hasCrClk && hasStimTime)) {
        std::printf("[STM32-RHS2116] Preset missing required keys\n");
        return false;
    }

    // 1) Sample rate
    const int fs_i = static_cast<int>(fs);
    sampleRateBox.setSelectedId(fs_i, juce::dontSendNotification);
    rebuildDspFreqItems(fs_i);

    // 2) Bandwidths
    auto round2 = [](double v) noexcept { return std::round(v * 100.0) / 100.0; };
    auto round0 = [](double v) noexcept { return std::round(v); };
    const double lbw2 = round2(lbw);
    const double ubw0 = round0(ubw);
    lowerBwBox.setText(juce::String(lbw2, 2), juce::dontSendNotification);
    upperBwBox.setText(juce::String(ubw0, 0), juce::dontSendNotification);


    // 3) DSP enable
    dspEnableButton.setToggleState(dspEn, juce::dontSendNotification);


    // 4) DSP N
    const int N = juce::jlimit(1, 15, static_cast<int>(dspN_d));
    dspFreqBox.setSelectedId(N, juce::dontSendNotification);
    const double k  = DSP_K_TABLE[N];
    const double fc = k * static_cast<double>(fs_i);
    std::printf("[STM32-RHS2116] DSP cutoff from preset: %.6f Hz (k=%.6g, N=%d)\n", fc, k, N);

    // 5) Acquisition time
    std::printf("[STM32-RHS2116] Acquisition time from preset: %d seconds\n", acquisitionTimeSec);

    // Validate stimulation current step size (in nA)
    {
        static const int kAllowedSteps[] = { 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000 };
        bool ok = false;
        for (int v : kAllowedSteps) { if (stim_step_na == v) { ok = true; break; } }
        if (!ok) {
            std::printf("[STM32-RHS2116] Invalid stim_step_size_na: %d (allowed: 10 20 50 100 200 500 1000 2000 5000 10000)\n",
                        stim_step_na);
            return false;
        }
    }

    // Selected stimulation currents: step (nA) * multiplier (1..255)
    const int posCurrentNa = stim_step_na * stim_pos_current;
    const int negCurrentNa = stim_step_na * stim_neg_current;
    std::printf("[STM32-RHS2116] Positive current: %d nA (step=%d nA * mult=%d)\n",
                posCurrentNa, stim_step_na, stim_pos_current);
    std::printf("[STM32-RHS2116] Negative current: %d nA (step=%d nA * mult=%d)\n",
                negCurrentNa, stim_step_na, stim_neg_current);



    // push to thread
    if (thread)
    {
        // acquisition
        thread->setSampleRate(fs_i);
        thread->setLowerBandwidthHz(lbw2);
        thread->setUpperBandwidthHz(ubw0);
        thread->setDspEnabled(dspEn);
        thread->setDspKFactor(N);
        thread->setAcquisitionTimeSeconds(acquisitionTimeSec);
        
        // stimulation
        thread->setStimEnabled(stim_enable != 0);
        thread->setStimVoltage(stim_voltage);
        thread->setStimStepNa(stim_step_na);
        thread->setStimPosCurrent(stim_pos_current);
        thread->setStimNegCurrent(stim_neg_current);
        thread->setStimType(stim_type);
        thread->setStimPolarity(stim_polarity);
        thread->setStimClkPos(stim_clk_pos);
        thread->setStimClkNeg(stim_clk_neg);
        thread->setStimContinuous(stim_continuous);
        thread->setChargeRecoveryEnable(cr_enable);
        thread->setChargeRecoveryClk(cr_clk);
        thread->setStimulationTimeMs(stim_time_ms);
        if (!seq.empty()) 
            thread->setStimSequence(seq);
    }    

    // Force a repaint
    repaint();
    return true;
}

// ==================================================================================================================





// ================================ Toggle the start/stop button state (used by the thread) ==========================
void DataThreadPluginEditor::setStartToggle (bool on)
{
    startSeqButton.setToggleState(on, juce::dontSendNotification); // UI only
}
// ==================================================================================================================