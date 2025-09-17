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

#ifndef DATATHREADPLUGINEDITOR_H_DEFINED
#define DATATHREADPLUGINEDITOR_H_DEFINED

#pragma once

#include <EditorHeaders.h>
#include "DataThreadPlugin.h"
#include "ofSerial.h" // uses getDeviceList() to enumerate serial ports

class DataThreadPluginEditor : public GenericEditor
{
public:
    /** Minimal editor: only a ComboBox that lists serial ports. */
    DataThreadPluginEditor (GenericProcessor* parentNode, DataThreadPlugin* plugin);
    ~DataThreadPluginEditor() override = default;

    void resized() override;

private:
    DataThreadPlugin* thread = nullptr;

    /** ComboBox that refreshes itself right before the popup is shown. */
    struct RefreshingComboBox : public juce::ComboBox
    {
        std::function<void()> beforePopup;

        /** Refresh the items on click, then open the popup as usual. */
        void mouseDown (const juce::MouseEvent& e) override
        {
            if (beforePopup) beforePopup();
            juce::ComboBox::mouseDown(e);
        }
    } comBox;

    ofSerial serial; // used only to enumerate ports with getDeviceList()

    juce::Label portLabel;
};


#endif