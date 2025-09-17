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
    desiredWidth = 300;

    // Port label
    portLabel.setText("Port:", juce::dontSendNotification);
    portLabel.setJustificationType(juce::Justification::centredLeft);
    addAndMakeVisible(portLabel);

    // ComboBox for serial ports
    comBox.setTextWhenNoChoicesAvailable("No serial ports found");
    comBox.setTextWhenNothingSelected("Select");

    // Auto-refresh ports when the user clicks the drop-down
    comBox.beforePopup = [this]()
    {
        // Rebuild the list just-in-time
        comBox.clear(juce::dontSendNotification);

        auto devices = serial.getDeviceList(); // requires ofSerial.h/.cpp

        int id = 1;
        for (auto& d : devices)
        {
            // You can switch to d.getDevicePath() if you prefer strictly "COMx" / "/dev/tty*".
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
            std::cout << "Selected port: " << selected << "\n";
        }
    };
}
// =================================================================================================================


void DataThreadPluginEditor::resized()
{
    portLabel.setBounds(10, 20, 40, 40);   // x, y, larghezza, altezza
    comBox.setBounds(60, 30, 100, 20);     // x, y, larghezza, altezza

}
