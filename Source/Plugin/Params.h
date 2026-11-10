// Params.h
// Parameter table shared by the plugin, the editor and the preset store.
//
// Front-panel controls first (the three pots and the S1 footswitch), then the
// plugin-only controls in the editor's bottom strip.

#ifndef TBCC_PARAMS_H
#define TBCC_PARAMS_H

namespace tbcc
{

enum Param
{
    // Front panel
    kFrequency = 0, kDepth, kResonance, kEffect,
    // Plugin only (bottom strip of the editor)
    kInput, kOutput,
    kNumParams
};

enum Kind { kKnob, kRateHz, kSwitch, kDb12, kDb24 };

const char* const kParamNames[kNumParams] = {
    "Freq", "Depth", "Reso", "Effect",
    "Input", "Output"
};

const int kParamKind[kNumParams] = {
    kRateHz, kKnob, kKnob, kSwitch,
    kDb12, kDb24
};

const float kDefaults[kNumParams] = {
    0.4f, 0.7f, 0.5f, 1.0f,
    0.5f, 0.5f
};

const double kInputRangeDb  = 12.0;
const double kOutputRangeDb = 24.0;

inline bool isOn (float v) { return v >= 0.5f; }

} // namespace tbcc

#endif
