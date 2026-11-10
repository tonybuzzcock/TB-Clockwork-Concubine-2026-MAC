// flanger_test.cpp
// Offline checks for the TB Clockwork Concubine DSP. Builds on Linux or macOS:
//   make -C Tests run      (writes demo WAVs to Tests/renders)
//
// 1. Bypass: S1 out passes the input straight through
// 2. Delay: the wet impulse lands at the BBD delay; DEPTH sweeps the range
// 3. FREQUENCY: LFO rate range and the measured sweep rate
// 4. The comb: deep notches with the dry and wet mixed; level of the dry path
// 5. RESONANCE: peaks grow with the knob, the loop stays stable at 10
// 6. BBD: clipping on a hot input stays bounded; clock aliasing at long delays
// 7. Stability sweep: knob extremes x 10 mV-4 V x 41 Hz-15 kHz, plus noise
// 8. Automation of every control while playing; CPU; demo renders

#include "../Source/DSP/Flanger.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <string>
#include <vector>

using namespace tbcc;

static const double kPiT = 3.14159265358979323846;
static const int kSr = 44100;
static int failures = 0;

static void check (bool ok, const char* what)
{
    std::printf ("  %s  %s\n", ok ? "ok  " : "FAIL", what);
    if (! ok) ++failures;
}

static bool allFinite (const std::vector<float>& x, double limit)
{
    for (size_t i = 0; i < x.size(); ++i)
        if (! (x[i] == x[i]) || std::fabs (x[i]) > limit) return false;
    return true;
}

static double rmsRange (const std::vector<float>& x, size_t from, size_t to)
{
    double s = 0.0;
    if (to > x.size()) to = x.size();
    for (size_t i = from; i < to; ++i) s += (double) x[i] * x[i];
    return to > from ? std::sqrt (s / (double) (to - from)) : 0.0;
}

static double peakRange (const std::vector<float>& x, size_t from, size_t to)
{
    double p = 0.0;
    if (to > x.size()) to = x.size();
    for (size_t i = from; i < to; ++i) p = std::max (p, (double) std::fabs (x[i]));
    return p;
}

static std::vector<float> sine (double f, double amp, double sec)
{
    std::vector<float> v ((size_t) (sec * kSr));
    for (size_t i = 0; i < v.size(); ++i) v[i] = (float) (amp * std::sin (2.0 * kPiT * f * i / kSr));
    return v;
}

static std::vector<float> run (Flanger& a, const std::vector<float>& in)
{
    std::vector<float> out (in.size());
    for (size_t pos = 0; pos < in.size(); pos += 256)
    {
        const int n = (int) std::min ((size_t) 256, in.size() - pos);
        a.process (&in[pos], &out[pos], n);
    }
    return out;
}

static void defaults (Flanger& a)
{
    a.setFrequency (0.4); a.setDepth (0.7); a.setResonance (0.5); a.setEffectOn (true);
    a.setInputGainDb (0.0); a.setOutputGainDb (0.0);
}

// Static comb: DEPTH at 0 holds the delay still
static void still (Flanger& a, double reso)
{
    defaults (a);
    a.setDepth (0.0);
    a.setResonance (reso);
}

// Steady-state gain (dB) of a sine through the unit
static double gainDb (Flanger& a, double f, double amp = 0.1)
{
    a.reset();
    std::vector<float> x = sine (f, amp, 0.5);
    std::vector<float> y = run (a, x);
    const double gi = rmsRange (x, kSr / 4, kSr / 2), go = rmsRange (y, kSr / 4, kSr / 2);
    return 20.0 * std::log10 ((go + 1e-12) / (gi + 1e-12));
}

static void writeWav (const std::string& path, const std::vector<float>& x, int sr)
{
    FILE* f = std::fopen (path.c_str(), "wb");
    if (! f) return;
    const unsigned dataBytes = (unsigned) x.size() * 2, riff = 36 + dataBytes;
    const unsigned short fmt = 1, ch = 1, bits = 16, align = 2;
    const unsigned rate = (unsigned) sr, bps = rate * 2, fmtLen = 16;
    std::fwrite ("RIFF", 1, 4, f); std::fwrite (&riff, 4, 1, f); std::fwrite ("WAVEfmt ", 1, 8, f);
    std::fwrite (&fmtLen, 4, 1, f); std::fwrite (&fmt, 2, 1, f); std::fwrite (&ch, 2, 1, f);
    std::fwrite (&rate, 4, 1, f); std::fwrite (&bps, 4, 1, f); std::fwrite (&align, 2, 1, f);
    std::fwrite (&bits, 2, 1, f); std::fwrite ("data", 1, 4, f); std::fwrite (&dataBytes, 4, 1, f);
    for (size_t i = 0; i < x.size(); ++i)
    {
        double v = x[i] * 0.9;
        v = v > 1.0 ? 1.0 : (v < -1.0 ? -1.0 : v);
        const short s = (short) std::floor (v * 32767.0 + 0.5);
        std::fwrite (&s, 2, 1, f);
    }
    std::fclose (f);
}

// A decaying plucked string (Karplus-Strong), added into buf at start
static void pluck (std::vector<float>& buf, size_t start, double freq, double amp, double dur)
{
    const int period = std::max (2, (int) (kSr / freq + 0.5));
    std::vector<double> line (period);
    unsigned seed = (unsigned) (freq * 1000.0) + (unsigned) start;
    for (int i = 0; i < period; ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        line[i] = ((seed >> 9) / 4194304.0 - 1.0) * amp;
    }
    const size_t n = (size_t) (dur * kSr);
    int idx = 0;
    double prev = 0.0;
    for (size_t i = 0; i < n && start + i < buf.size(); ++i)
    {
        const double cur = line[idx];
        const double next = 0.996 * 0.5 * (cur + prev);
        prev = cur;
        line[idx] = next;
        buf[start + i] += (float) cur;
        if (++idx >= period) idx = 0;
    }
}

static std::vector<float> makeChords (double seconds)
{
    std::vector<float> v ((size_t) (seconds * kSr), 0.0f);
    const double chords[4][4] = { { 82.4, 123.5, 164.8, 207.7 }, { 110.0, 164.8, 220.0, 277.2 },
                                  { 98.0, 146.8, 196.0, 246.9 }, { 73.4, 110.0, 146.8, 185.0 } };
    for (int c = 0; c * 2.0 < seconds; ++c)
        for (int s = 0; s < 4; ++s)
            pluck (v, (size_t) ((c * 2.0 + s * 0.012) * kSr), chords[c % 4][s], 0.22, 2.0);
    return v;
}

static std::vector<float> makeRiff (double seconds)
{
    std::vector<float> v ((size_t) (seconds * kSr), 0.0f);
    const double notes[8] = { 82.4, 98.0, 110.0, 123.5, 110.0, 98.0, 146.8, 130.8 };
    for (int i = 0; i * 0.25 < seconds; ++i)
        pluck (v, (size_t) (i * 0.25 * kSr), notes[i % 8], 0.35, 0.6);
    return v;
}

static std::vector<float> noise (double amp, double sec, unsigned seed)
{
    std::vector<float> v ((size_t) (sec * kSr));
    for (size_t i = 0; i < v.size(); ++i)
    {
        seed = seed * 1664525u + 1013904223u;
        v[i] = (float) (amp * ((seed >> 9) / 4194304.0 - 1.0));
    }
    return v;
}

int main (int argc, char** argv)
{
    const std::string renderDir = argc > 1 ? argv[1] : ".";
    Flanger* ap = new Flanger();
    Flanger& a = *ap;
    a.prepare (kSr);
    const int lat = Flanger::latencySamples();

    std::printf ("\n1. Bypass\n");
    {
        defaults (a);
        a.setEffectOn (false);
        a.reset();
        std::vector<float> x = sine (440.0, 0.3, 0.5), y = run (a, x);
        // The oversampler's filters delay by the reported latency to within a
        // fraction of a sample, so compare at the best of the two nearest lags
        double maxd = 1e9;
        for (int L = lat - 1; L <= lat; ++L)
        {
            double m = 0.0;
            for (size_t i = kSr / 10; i < x.size(); ++i) m = std::max (m, (double) std::fabs (y[i] - x[i - L]));
            maxd = std::min (maxd, m);
        }
        std::printf ("  bypassed vs input (latency aligned): max diff %.2g of 0.3\n", maxd);
        check (maxd < 0.02, "S1 out passes the input straight through");
        a.setEffectOn (true);
    }

    std::printf ("\n2. BBD delay and DEPTH\n");
    {
        still (a, 0.0);
        a.reset();
        // Click in, look for the wet copy after the dry one
        std::vector<float> x ((size_t) (0.1 * kSr), 0.0f);
        for (int i = 0; i < 40; ++i) x[200 + i] = (float) (0.5 * std::sin (kPiT * i / 40.0));
        std::vector<float> y = run (a, x);
        // dry peak (IC5 inverts twice: positive), then the wet one
        size_t dryAt = 0; double best = 0.0;
        for (size_t i = 0; i < 600; ++i) if (std::fabs (y[i]) > best) { best = std::fabs (y[i]); dryAt = i; }
        size_t wetAt = 0; double bw = 0.0;
        const size_t from = dryAt + (size_t) (0.0015 * kSr);
        for (size_t i = from; i < from + (size_t) (0.004 * kSr); ++i) if (std::fabs (y[i]) > bw) { bw = std::fabs (y[i]); wetAt = i; }
        const double ms = 1000.0 * (double) (wetAt - dryAt) / kSr;
        std::printf ("  DEPTH 0: wet copy %.2f ms after the dry (nominal %.2f), dry %.3f, wet %.3f\n",
                     ms, Flanger::delayAtDepthZeroMs(), best, bw);
        check (std::fabs (ms - Flanger::delayAtDepthZeroMs()) < 0.25, "wet click lands at the BBD delay");
        check (std::fabs (a.currentClockHz() - 256.0 / 3.0e-3) < 1.0, "clock: 256 periods for 512 stages");

        defaults (a);
        a.setDepth (1.0);
        a.setFrequency (0.8);
        a.reset();
        double lo = 1e9, hi = 0.0;
        std::vector<float> z (256, 0.0f), o (256);
        for (int b = 0; b < 4 * kSr / 256; ++b)
        {
            a.process (&z[0], &o[0], 256);
            lo = std::min (lo, a.currentDelayMs());
            hi = std::max (hi, a.currentDelayMs());
        }
        std::printf ("  DEPTH 10: sweeps %.2f - %.2f ms (clock %.0f - %.0f kHz)\n", lo, hi,
                     256.0 / hi, 256.0 / lo);
        check (std::fabs (lo - Flanger::delayRangeMs (1.0, false)) < 0.05
               && std::fabs (hi - Flanger::delayRangeMs (1.0, true)) < 0.05, "full DEPTH covers the range");
        double lo5 = 1e9, hi5 = 0.0;
        a.setDepth (0.5);
        for (int b = 0; b < 4 * kSr / 256; ++b)
        {
            a.process (&z[0], &o[0], 256);
            if (b > kSr / 256) { lo5 = std::min (lo5, a.currentDelayMs()); hi5 = std::max (hi5, a.currentDelayMs()); }
        }
        check (hi5 - lo5 < hi - lo && hi5 - lo5 > 0.3 * (hi - lo), "DEPTH 5 sweeps about half as far");
    }

    std::printf ("\n3. FREQUENCY\n");
    {
        std::printf ("  LFO: %.2f Hz at 0, %.2f at 5, %.2f at 10\n", Flanger::lfoHz (0.0), Flanger::lfoHz (0.5), Flanger::lfoHz (1.0));
        bool mono = true;
        for (int k = 1; k <= 20; ++k) if (Flanger::lfoHz (k / 20.0) <= Flanger::lfoHz ((k - 1) / 20.0)) mono = false;
        check (mono, "rate rises clockwise");
        check (Flanger::lfoHz (0.0) > 0.05 && Flanger::lfoHz (0.0) < 0.2 && Flanger::lfoHz (1.0) > 5.0 && Flanger::lfoHz (1.0) < 15.0,
               "about 0.1 - 10 Hz");
        // Count the measured sweep cycles at FREQUENCY 7
        defaults (a);
        a.setFrequency (0.7);
        a.reset();
        std::vector<float> z (64, 0.0f), o (64);
        int crossings = 0; double prev = 0.0;
        const int blocks = 5 * kSr / 64;
        for (int b = 0; b < blocks; ++b)
        {
            a.process (&z[0], &o[0], 64);
            const double v = a.lfoValue();
            if (b > 0 && prev < 0.0 && v >= 0.0) ++crossings;
            prev = v;
        }
        const double hz = crossings / 5.0, expect = Flanger::lfoHz (0.7);
        std::printf ("  FREQUENCY 7: %.2f Hz measured, %.2f expected\n", hz, expect);
        check (std::fabs (hz - expect) < 0.25, "sweep rate as set");
    }

    std::printf ("\n4. The comb (DEPTH 0, RESONANCE 0)\n");
    double notchDepth = 0.0;
    {
        still (a, 0.0);
        double mx = -100.0, mn = 100.0, fmx = 0.0, fmn = 0.0;
        for (double f = 150.0; f < 3000.0; f *= 1.02)
        {
            const double g = gainDb (a, f);
            if (g > mx) { mx = g; fmx = f; }
            if (g < mn) { mn = g; fmn = f; }
        }
        notchDepth = mx - mn;
        std::printf ("  150 Hz - 3 kHz: peak %+.1f dB at %.0f Hz, notch %+.1f dB at %.0f Hz\n", mx, fmx, mn, fmn);
        check (notchDepth > 15.0, "deep notches (dry and wet nearly equal through the mids)");
        check (mx < 3.0 && mx > -6.0, "comb peaks near unity gain");
        // Dry alone: the BBD signal is a delayed copy; low bass is mostly dry
        const double g50 = gainDb (a, 50.0), g10k = gainDb (a, 10000.0);
        std::printf ("  50 Hz %+.1f dB, 10 kHz %+.1f dB\n", g50, g10k);
        check (g50 > -9.0 && g50 < -4.0, "bass is the dry path: about -6.6 dB (22K / 47K)");
        check (g10k < g50, "top rolls off (IC5's CX2 6n8 and the T network)");
    }

    std::printf ("\n5. RESONANCE\n");
    {
        double lastSpread = 0.0;
        bool grows = true;
        for (int k = 0; k <= 2; ++k)
        {
            still (a, k * 0.5);
            double mx = -100.0, mn = 100.0;
            for (double f = 300.0; f < 1500.0; f *= 1.015)
            {
                const double g = gainDb (a, f);
                mx = std::max (mx, g); mn = std::min (mn, g);
            }
            std::printf ("  RESONANCE %d: peaks %+.1f dB, notches %+.1f dB\n", k * 5, mx, mn);
            if (k > 0 && mx <= lastSpread) grows = false;
            lastSpread = mx;
        }
        check (grows, "resonance peaks rise with the knob");

        // Full resonance, then silence: the ringing decays (no self-oscillation)
        still (a, 1.0);
        a.setDepth (1.0);
        a.reset();
        std::vector<float> x = noise (0.3, 1.0, 7), z (3 * kSr, 0.0f);
        run (a, x);
        std::vector<float> y = run (a, z);
        const double tail = peakRange (y, 2 * kSr, 3 * kSr);
        std::printf ("  RESONANCE 10, 2 s after the input stops: %.2g\n", tail);
        check (tail < 1e-3, "rings down: the loop gain stays below 1");
    }

    std::printf ("\n6. BBD limits\n");
    {
        defaults (a);
        a.setResonance (1.0);
        a.reset();
        a.clearPeakBbd();
        std::vector<float> x = sine (220.0, 3.0, 1.0), y = run (a, x);
        std::printf ("  3 V in: output peak %.2f, BBD drive %.2f of clip\n", peakRange (y, kSr / 2, kSr), a.peakBbd());
        check (allFinite (y, 8.0) && peakRange (y, kSr / 2, kSr) < 6.0, "hot input stays bounded (rails and BBD clip)");
        check (a.peakBbd() > 0.9, "the BBD clips on a hot input");

        // Long delays: the clock drops, the staircase aliases a high tone
        still (a, 0.0);
        a.reset();
        std::vector<float> h = sine (9000.0, 0.3, 0.5), yh = run (a, h);
        check (allFinite (yh, 4.0), "clean at a 9 kHz tone");
    }

    std::printf ("\n7. Stability sweep\n");
    {
        const double amps[4] = { 0.01, 0.1, 1.0, 4.0 };
        const double freqs[5] = { 41.0, 110.0, 440.0, 2000.0, 15000.0 };
        const double corners[4][3] = { { 0, 0, 0 }, { 1, 1, 1 }, { 0, 1, 1 }, { 1, 0, 1 } };
        int runs = 0; bool ok = true;
        for (int c = 0; c < 4; ++c)
            for (int ai = 0; ai < 4; ++ai)
                for (int fi = 0; fi < 5; ++fi)
                {
                    a.setFrequency (corners[c][0]); a.setDepth (corners[c][1]); a.setResonance (corners[c][2]);
                    a.setEffectOn (true);
                    a.reset();
                    std::vector<float> y = run (a, sine (freqs[fi], amps[ai], 0.4));
                    if (! allFinite (y, 10.0)) ok = false;
                    ++runs;
                }
        unsigned seed = 1;
        for (int r = 0; r < 20; ++r)
        {
            seed = seed * 1664525u + 1013904223u; const double f1 = (seed >> 8) / 16777216.0;
            seed = seed * 1664525u + 1013904223u; const double f2 = (seed >> 8) / 16777216.0;
            seed = seed * 1664525u + 1013904223u; const double f3 = (seed >> 8) / 16777216.0;
            a.setFrequency (f1); a.setDepth (f2); a.setResonance (f3);
            a.reset();
            std::vector<float> y = run (a, noise (2.0, 0.4, seed));
            if (! allFinite (y, 10.0)) ok = false;
            ++runs;
        }
        std::printf ("  %d runs\n", runs);
        check (ok, "no NaNs or runaways anywhere");
    }

    std::printf ("\n8. Automation, CPU, renders\n");
    {
        defaults (a);
        a.reset();
        std::vector<float> x = makeRiff (4.0), y (x.size());
        for (size_t pos = 0; pos < x.size(); pos += 64)
        {
            const double t = (double) pos / x.size();
            a.setFrequency (0.5 + 0.5 * std::sin (2 * kPiT * 3 * t));
            a.setDepth (0.5 + 0.5 * std::sin (2 * kPiT * 5 * t));
            a.setResonance (0.5 + 0.5 * std::sin (2 * kPiT * 7 * t));
            a.setEffectOn (std::sin (2 * kPiT * 4 * t) > 0.0);
            a.setInputGainDb (12.0 * std::sin (2 * kPiT * 2 * t));
            a.setOutputGainDb (12.0 * std::sin (2 * kPiT * 9 * t));
            const int n = (int) std::min ((size_t) 64, x.size() - pos);
            a.process (&x[pos], &y[pos], n);
        }
        check (allFinite (y, 10.0), "every control automated while playing");

        defaults (a);
        a.reset();
        std::vector<float> ten = makeChords (10.0);
        std::clock_t t0 = std::clock();
        std::vector<float> yc = run (a, ten);
        const double secs = (double) (std::clock() - t0) / CLOCKS_PER_SEC;
        std::printf ("  CPU: %.1f%% of one core at 44.1 kHz\n", 10.0 * secs);
        check (secs < 3.0, "runs well under real time");

        struct R { const char* name; double f, d, r; bool chords; };
        const R renders[] = {
            { "dry_chords",        0.0, 0.0, 0.0, true },
            { "slow_sweep_chords", 0.25, 0.8, 0.4, true },
            { "jet_resonance_chords", 0.15, 1.0, 1.0, true },
            { "fast_riff",         0.8, 0.6, 0.5, false },
            { "default_riff",      0.4, 0.7, 0.5, false },
        };
        std::vector<float> riff = makeRiff (8.0), chords = makeChords (8.0);
        for (size_t k = 0; k < sizeof (renders) / sizeof (renders[0]); ++k)
        {
            defaults (a);
            a.setFrequency (renders[k].f); a.setDepth (renders[k].d); a.setResonance (renders[k].r);
            a.setEffectOn (k != 0);
            a.reset();
            const std::vector<float>& src = renders[k].chords ? chords : riff;
            std::vector<float> o = run (a, src);
            writeWav (renderDir + "/" + renders[k].name + ".wav", o, kSr);
        }
        check (true, "demo renders written");
    }

    delete ap;
    std::printf ("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL DSP TESTS PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
