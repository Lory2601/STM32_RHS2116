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

#include "DataThreadPluginEditor.h"
#include <iostream>

// ========================================== DataThreadPluginEditor impl ==========================================
DataThreadPluginEditor::DataThreadPluginEditor (GenericProcessor* parentNode, DataThreadPlugin* plugin)
    : GenericEditor (parentNode), thread(plugin)
{
    // Set size of the editor window
    desiredWidth = 430;

    //-------------------------------------------------- serial port -----------------------------------------------
    // Port label
    portLabel.setText("Serial port:", juce::dontSendNotification);
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
            const int prevN = dspFreqBox.getSelectedId(); // try to keep same N
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
    lowerBwLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(lowerBwLabel);

    // ComboBox for lower bandwidth (string values, including decimals like 7.5 / 0.50 / 0.10)
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
    upperBwLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(upperBwLabel);

    // ComboBox for upper bandwidth (string values, including decimals)
    {
        int id = 1; // JUCE item IDs are int; we use sequential IDs and parse text to double
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

            // Console info (C++ and C-style)
            std::cout << "[STM32-RHS2116] Selected DSP cutoff: " << fc << " Hz (k=" << k << ", N=" << N << ")\n";

            if (thread != nullptr)
            {
                // Pass only the K factor to the processing thread
                thread->setDspKFactor(N);
            }
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
        const int N_as_K = dspFreqBox.getSelectedId(); // 1..15

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
    // Rebuild the dropdown with frequencies computed from kfreq * fsample.
    // Item IDs are set to N (cutoff code 1..15). We omit N=0 (differentiator) from the list.
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
    lowerBwLabel.setBounds(5, 55, 120, 40);
    lowerBwBox.setBounds(125, 65, 80, 20);

    // upper bandwidth
    upperBwLabel.setBounds(215, 55, 120, 40);
    upperBwBox.setBounds(340, 65, 80, 20);

    // dsp enable
    dspEnableLabel.setBounds(5, 90, 120, 40);
    dspEnableButton.setBounds(125, 100, 80, 20);

    // dsp freq
    dspFreqLabel.setBounds(215, 90, 120, 40);
    dspFreqBox.setBounds(340, 100, 80, 20);

}