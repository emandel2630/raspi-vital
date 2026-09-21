// alsa_host.cpp — RaspiSynth-Vital: a live headless ALSA host for the real
// Vital synthesis engine, replacing Vital's offline render main().
//
//   EWI (USB MIDI) -> ALSA seq thread -> lock-free ring -> RT audio thread
//                                                          HeadlessSynth (vital::SoundEngine)
//                                                          -> ALSA hw (S16 + TPDF dither)
//
// Patches: real .vital files in patch_dir, sorted by name. MIDI Program
// Change selects; loading runs on a worker thread (wavetable rendering takes
// hundreds of ms) while the RT thread emits silence via tryEnter().
// Breath CC drives macro_control_1 through a cached vital::Value* (RT-safe).
#include "JuceHeader.h"
#include "synth_base.h"
#include "sound_engine.h"
#include "synth_constants.h"
#include "poly_values.h"

#include <alsa/asoundlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <dirent.h>
#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include <climits>
#include <string>
#include <thread>
#include <vector>

// c++14 build: no std::clamp
template <typename T>
static inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ------------------------------------------------------------------ config
struct HostConfig {
    int breath_cc = 74;              // EWI breath -> macro_control_1
    int breath_to_at = 1;            // also mirror breath to channel aftertouch
    float breath_smooth_ms = 8.0f;
    int patch_cc = -1;               // optional CC that cycles patches
    std::string patch_dir = "/etc/vitalsynth.d";
    std::string audio_device = "auto";
    std::string audio_prefer = "HA3,ATR2xUSB";  // USB DAC preference, best first
    // MIDI input preference, best first, matched case-insensitively as a
    // substring of the ALSA client name. "*" may be used as the slot for any
    // source matching no other entry; with no "*" (as here) an unlisted device
    // sorts last, so a stray USB gadget cannot steal the input from the
    // instrument. U2MIDI Pro is the USB-MIDI adapter carrying the midiBeam
    // RX02 wireless receiver; NuEVI/Teensyduino is the EWI's own USB port.
    std::string midi_prefer = "U2MIDI Pro,midiBeam,RX02,NuEVI,Teensyduino";
    int sample_rate = 48000;
    int period_frames = 128;
    int periods = 2;
    float master_gain_db = 0.0f;
    float limiter_thresh = 0.95f;
    int log_midi = 1;
    int oversampling = -1;           // -1 = keep patch value; 0/1/2 = 1x/2x/4x

    void set(const char* key, const char* val) {
        auto f = [&](const char* k, float& dst) {
            if (!std::strcmp(key, k)) { dst = std::strtof(val, nullptr); return true; }
            return false;
        };
        auto ii = [&](const char* k, int& dst) {
            if (!std::strcmp(key, k)) { dst = std::atoi(val); return true; }
            return false;
        };
        if (ii("breath_cc", breath_cc)) return;
        if (ii("breath_to_at", breath_to_at)) return;
        if (f("breath_smooth_ms", breath_smooth_ms)) return;
        if (ii("patch_cc", patch_cc)) return;
        if (ii("sample_rate", sample_rate)) return;
        if (ii("period_frames", period_frames)) return;
        if (ii("periods", periods)) return;
        if (f("master_gain_db", master_gain_db)) return;
        if (f("limiter_thresh", limiter_thresh)) return;
        if (ii("log_midi", log_midi)) return;
        if (ii("oversampling", oversampling)) return;
        if (!std::strcmp(key, "patch_dir")) { patch_dir = val; return; }
        if (!std::strcmp(key, "audio_device")) { audio_device = val; return; }
        if (!std::strcmp(key, "audio_prefer")) { audio_prefer = val; return; }
        if (!std::strcmp(key, "midi_prefer")) { midi_prefer = val; return; }
        std::fprintf(stderr, "config: unknown key '%s' (ignored)\n", key);
    }
    bool loadFile(const char* path) {
        FILE* fp = std::fopen(path, "r");
        if (!fp) return false;
        char line[512];
        while (std::fgets(line, sizeof line, fp)) {
            char* hash = std::strchr(line, '#');
            if (hash) *hash = 0;
            char key[128], val[256];
            // Take the rest of the line, not %s: a value may contain spaces
            // ("midi_prefer = midiBeam RX02, *, NuEVI"), and %s silently kept
            // only the first word. Comments are already cut above, so trailing
            // whitespace is all that needs trimming.
            if (std::sscanf(line, " %127[A-Za-z0-9_] = %255[^\n]", key, val) == 2) {
                size_t n = std::strlen(val);
                while (n > 0 && std::isspace((unsigned char)val[n - 1])) val[--n] = 0;
                if (n > 0) set(key, val);
            }
        }
        std::fclose(fp);
        return true;
    }
    void sanitize() {
        auto cf = [](float& v, float lo, float hi, float d) {
            if (!std::isfinite(v)) v = d;
            v = clampv(v, lo, hi);
        };
        auto ci = [](int& v, int lo, int hi) { v = clampv(v, lo, hi); };
        ci(breath_cc, 0, 127); ci(breath_to_at, 0, 1); ci(patch_cc, -1, 127);
        cf(breath_smooth_ms, 0.01f, 1000.0f, 8.0f);
        ci(sample_rate, 8000, 192000); ci(period_frames, 16, 4096); ci(periods, 2, 8);
        cf(master_gain_db, -60.0f, 24.0f, 0.0f);
        cf(limiter_thresh, 0.1f, 1.0f, 0.95f);
        ci(log_midi, 0, 1); ci(oversampling, -1, 2);
    }
};

// ------------------------------------------------------------- midi message
struct MidiMsg {
    enum Type : uint8_t { NoteOff = 0, NoteOn = 1, CC = 2, PitchBend = 3, AllOff = 4,
                          Patch = 5, SoundsOff = 6, ChanPressure = 7, PolyPressure = 8 };
    uint8_t type;
    int16_t a;
    int16_t b;
};

template <size_t N>
class SpscRing {
    static_assert((N & (N - 1)) == 0, "power of two");
public:
    bool push(const MidiMsg& m) {
        const size_t w = write_.load(std::memory_order_relaxed);
        const size_t r = read_.load(std::memory_order_acquire);
        if (w - r >= N) return false;
        buf_[w & (N - 1)] = m;
        write_.store(w + 1, std::memory_order_release);
        return true;
    }
    bool pop(MidiMsg& out) {
        const size_t r = read_.load(std::memory_order_relaxed);
        const size_t w = write_.load(std::memory_order_acquire);
        if (r == w) return false;
        out = buf_[r & (N - 1)];
        read_.store(r + 1, std::memory_order_release);
        return true;
    }
private:
    MidiMsg buf_[N];
    std::atomic<size_t> write_{0};
    std::atomic<size_t> read_{0};
};

// -------------------------------------------------------------------- synth
// Re-export the protected bits of SynthBase our audio loop needs.
class HostSynth : public HeadlessSynth {
public:
    using SynthBase::processModulationChanges;
};

// --------------------------------------------------------------------- glue
static std::atomic<bool> g_running{true};
static void onSignal(int) { g_running.store(false); }

static std::atomic<int> g_requestedPatch{-1};
static std::atomic<int> g_loadedPatch{-1};
static std::atomic<float> g_patchGain{1.0f};   // loudness-normalization gain (linear)

struct PatchList {
    std::vector<std::string> paths;
    std::vector<std::string> names;
    std::vector<float> gains;                  // linear, 1.0 = no adjustment
};

static PatchList scanPatches(const std::string& dir) {
    PatchList out;
    std::vector<std::string> files;
    if (DIR* d = opendir(dir.c_str())) {
        while (struct dirent* e = readdir(d)) {
            const std::string f = e->d_name;
            if (f.size() > 6 && f.rfind(".vital") == f.size() - 6)
                files.push_back(f);
        }
        closedir(d);
    }
    std::sort(files.begin(), files.end());
    for (const auto& f : files) {
        out.paths.push_back(dir + "/" + f);
        out.names.push_back(f.substr(0, f.size() - 6));
        out.gains.push_back(1.0f);
    }
    // Apply loudness-normalization gains written by --normalize (name dB pairs),
    // then user taste adjustments from trims.txt (same format, added on top).
    auto applyDbFile = [&](const std::string& path, float limit) {
        FILE* g = std::fopen(path.c_str(), "r");
        if (!g) return;
        char line[512];
        while (std::fgets(line, sizeof line, g)) {   // line-by-line: comments/blank
            char* hash = std::strchr(line, '#');     // lines must not stop parsing
            if (hash) *hash = 0;
            // The dB value is the LAST token; everything before it is the name.
            // Splitting on the first space instead would silently drop every
            // patch whose filename contains one, and those arrive routinely
            // from the Samba share ("Soft Wind.vital").
            std::string text(line);
            const size_t end = text.find_last_not_of(" \t\r\n");
            if (end == std::string::npos) continue;
            text.resize(end + 1);
            const size_t split = text.find_last_of(" \t");
            if (split == std::string::npos) continue;
            char* parseEnd = nullptr;
            const double db = std::strtod(text.c_str() + split + 1, &parseEnd);
            if (parseEnd == text.c_str() + split + 1) continue;   // not a number
            std::string name = text.substr(0, split);
            const size_t begin = name.find_first_not_of(" \t");
            if (begin == std::string::npos) continue;
            name = name.substr(begin, name.find_last_not_of(" \t") - begin + 1);
            for (size_t i = 0; i < out.names.size(); ++i)
                if (out.names[i] == name)
                    out.gains[i] *= std::pow(10.0f, clampv(float(db), -limit, limit) * 0.05f);
        }
        std::fclose(g);
    };
    applyDbFile(dir + "/.gains", 15.0f);
    applyDbFile(dir + "/trims.txt", 18.0f);
    return out;
}

// Loader thread: applies patch changes off the RT path. loadFromFile()
// internally takes the synth CriticalSection (pauseProcessing), which the
// audio thread only tryEnter()s — so audio falls back to silence, never blocks.
static void loaderLoop(HostSynth& synth, const HostConfig& cfg, const PatchList& patches) {
    while (g_running.load(std::memory_order_relaxed)) {
        const int want = g_requestedPatch.load(std::memory_order_acquire);
        if (want >= 0 && want != g_loadedPatch.load(std::memory_order_relaxed) &&
            want < int(patches.paths.size())) {
            const auto t0 = std::chrono::steady_clock::now();
            std::string error;
            juce::File file(juce::String(patches.paths[size_t(want)]));
            if (!synth.loadFromFile(file, error)) {
                std::fprintf(stderr, "patch: FAILED to load %s: %s\n",
                             patches.paths[size_t(want)].c_str(), error.c_str());
                g_loadedPatch.store(want, std::memory_order_release); // don't retry-loop
                continue;
            }
            // Host overrides while we still may touch controls safely.
            synth.pauseProcessing(true);
            if (cfg.oversampling >= 0) {
                auto& controls = synth.getControls();
                auto it = controls.find("oversampling");
                if (it != controls.end()) it->second->set(vital::mono_float(cfg.oversampling));
            }
            synth.getEngine()->checkOversampling();
            synth.pauseProcessing(false);
            const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            const float gain = patches.gains[size_t(want)];
            g_patchGain.store(gain, std::memory_order_release);
            g_loadedPatch.store(want, std::memory_order_release);
            std::fprintf(stderr, "patch %d/%zu: %s (%lld ms, norm %+.1f dB)\n", want + 1,
                         patches.names.size(), patches.names[size_t(want)].c_str(),
                         (long long)ms, 20.0f * std::log10(gain));
        }
        struct timespec ts{0, 50 * 1000 * 1000};
        nanosleep(&ts, nullptr);
    }
}

// Comma-separated preference list -> trimmed tokens, best first. Only the
// whitespace AROUND an entry is stripped: device names have internal spaces
// ("Cubilux HA-3", "midiBeam RX02"), and eating those made such an entry match
// nothing at all.
static std::vector<std::string> splitPrefs(const std::string& s) {
    std::vector<std::string> out;
    std::string cur;
    auto flush = [&out, &cur]() {
        const size_t b = cur.find_first_not_of(" \t");
        if (b != std::string::npos) {
            const size_t e = cur.find_last_not_of(" \t");
            out.push_back(cur.substr(b, e - b + 1));
        }
        cur.clear();
    };
    for (char c : s) {
        if (c == ',') flush();
        else cur.push_back(c);
    }
    flush();
    return out;
}

// --------------------------------------------------------------- midi input
namespace midi_in {

bool isHardwareSource(snd_seq_t* seq, int client, int port) {
    if (client == SND_SEQ_CLIENT_SYSTEM) return false;
    snd_seq_client_info_t* cinfo = nullptr;
    snd_seq_client_info_alloca(&cinfo);
    if (snd_seq_get_any_client_info(seq, client, cinfo) < 0) return false;
    if (snd_seq_client_info_get_type(cinfo) != SND_SEQ_KERNEL_CLIENT) return false;
    if (std::strstr(snd_seq_client_info_get_name(cinfo), "Midi Through")) return false;
    snd_seq_port_info_t* pinfo = nullptr;
    snd_seq_port_info_alloca(&pinfo);
    if (snd_seq_get_any_port_info(seq, client, port, pinfo) < 0) return false;
    const unsigned caps = snd_seq_port_info_get_capability(pinfo);
    const unsigned need = SND_SEQ_PORT_CAP_READ | SND_SEQ_PORT_CAP_SUBS_READ;
    if ((caps & need) != need) return false;
    if (caps & SND_SEQ_PORT_CAP_NO_EXPORT) return false;
    return true;
}

// The rig has two possible controllers: the EWI on its own USB port, and a
// wireless receiver arriving through a generic USB-MIDI adapter. Only one may
// be subscribed at a time -- both at once would double every note -- so sources
// are ranked and the best one wins, re-evaluated on every hot-plug event.
struct MidiSource {
    int client = -1;
    int port = -1;
    int rank = INT_MAX;
    std::string name;
};

// Explicit preference entries match by case-insensitive substring and rank by
// position. A source matching no explicit entry takes the position of the "*"
// wildcard, or sorts last when the list has none. So "*,NuEVI" means "any other
// controller outranks the EWI", which needs no name for the adapter.
static int rankSource(const std::vector<std::string>& prefs, const std::string& name) {
    int star = int(prefs.size());
    for (size_t i = 0; i < prefs.size(); ++i) {
        if (prefs[i] == "*") { star = int(i); continue; }
        if (strcasestr(name.c_str(), prefs[i].c_str())) return int(i);
    }
    return star;
}

static std::string clientName(snd_seq_t* seq, int client) {
    snd_seq_client_info_t* cinfo = nullptr;
    snd_seq_client_info_alloca(&cinfo);
    if (snd_seq_get_any_client_info(seq, client, cinfo) < 0) return std::string();
    return std::string(snd_seq_client_info_get_name(cinfo));
}

// Subscribe to the highest-ranked hardware source available, dropping whatever
// was subscribed before. Silences the engine across the switch: a controller
// unplugged mid-note would otherwise leave that note sounding at full breath.
void selectSource(snd_seq_t* seq, int myPort, const std::vector<std::string>& prefs,
                  MidiSource& cur, SpscRing<1024>& ring, const HostConfig& cfg) {
    snd_seq_client_info_t* cinfo = nullptr;
    snd_seq_port_info_t* pinfo = nullptr;
    snd_seq_client_info_alloca(&cinfo);
    snd_seq_port_info_alloca(&pinfo);
    MidiSource best;
    snd_seq_client_info_set_client(cinfo, -1);
    while (snd_seq_query_next_client(seq, cinfo) == 0) {
        const int client = snd_seq_client_info_get_client(cinfo);
        snd_seq_port_info_set_client(pinfo, client);
        snd_seq_port_info_set_port(pinfo, -1);
        while (snd_seq_query_next_port(seq, pinfo) == 0) {
            const int port = snd_seq_port_info_get_port(pinfo);
            if (!isHardwareSource(seq, client, port)) continue;
            const std::string nm = clientName(seq, client);
            const int r = rankSource(prefs, nm);
            if (r < best.rank) { best.client = client; best.port = port; best.rank = r; best.name = nm; }
        }
    }
    if (best.client < 0) {
        if (cur.client >= 0) {
            std::fprintf(stderr, "midi: input gone (%s) — waiting for a controller\n",
                         cur.name.c_str());
            cur = MidiSource();
        }
        return;
    }
    if (best.client == cur.client && best.port == cur.port) return;

    if (cur.client >= 0)
        snd_seq_disconnect_from(seq, myPort, cur.client, cur.port);
    if (snd_seq_connect_from(seq, myPort, best.client, best.port) != 0) {
        std::fprintf(stderr, "midi: cannot subscribe %d:%d (%s)\n",
                     best.client, best.port, best.name.c_str());
        return;
    }
    // Drop any note/breath the outgoing controller left raised.
    ring.push({MidiMsg::CC, int16_t(cfg.breath_cc), 0});
    ring.push({MidiMsg::CC, 123, 0});
    if (cur.client >= 0)
        std::fprintf(stderr, "midi: input -> %d:%d (%s), replacing %s\n",
                     best.client, best.port, best.name.c_str(), cur.name.c_str());
    else
        std::fprintf(stderr, "midi: input -> %d:%d (%s)\n",
                     best.client, best.port, best.name.c_str());
    cur = best;
}

double nowSec() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return double(ts.tv_sec) + double(ts.tv_nsec) * 1e-9;
}

const char* noteName(int n, char* buf) { // buf >= 16 chars
    static const char* names[12] = {"C","C#","D","D#","E","F","F#","G","G#","A","A#","B"};
    n &= 127;
    std::snprintf(buf, 16, "%s%d", names[n % 12], n / 12 - 1);
    return buf;
}

void logCC(int cc, int value, bool isBreath) {
    static double last[128] = {};
    static int lastVal[128];
    static bool init = false;
    if (!init) { for (int i = 0; i < 128; ++i) lastVal[i] = -1; init = true; }
    cc &= 127;
    const double t = nowSec();
    if (value == lastVal[cc]) return;
    if (value != 0 && value != 127 && t - last[cc] < 0.2) return;
    last[cc] = t; lastVal[cc] = value;
    if (isBreath) {
        const int bars = value * 20 / 127;
        char meter[24];
        for (int i = 0; i < 20; ++i) meter[i] = i < bars ? '#' : '-';
        meter[20] = 0;
        std::fprintf(stderr, "midi: breath cc%-3d %3d [%s]\n", cc, value, meter);
    } else {
        std::fprintf(stderr, "midi: cc%d = %d\n", cc, value);
    }
}

void logBend(int value) {
    static double last = 0.0;
    static int lastShown = 0;
    const int v = (value > -100 && value < 100) ? 0 : value;
    if (v == 0) {
        if (lastShown != 0) { std::fprintf(stderr, "midi: bend +0\n"); lastShown = 0; }
        return;
    }
    const double t = nowSec();
    if (t - last < 0.2) return;
    last = t; lastShown = v;
    std::fprintf(stderr, "midi: bend %+d\n", v);
}

void logAftertouch(int value) { // throttled like breath, 0/127 edges always shown
    static double last = 0.0;
    static int lastVal = -1;
    const double t = nowSec();
    if (value == lastVal) return;
    if (value != 0 && value != 127 && t - last < 0.2) return;
    last = t; lastVal = value;
    std::fprintf(stderr, "midi: aftertouch %3d\n", value);
}

int run(const HostConfig& cfg, const PatchList& patches, SpscRing<1024>& ring) {
    snd_seq_t* seq = nullptr;
    if (snd_seq_open(&seq, "default", SND_SEQ_OPEN_DUPLEX, 0) < 0) {
        std::fprintf(stderr, "midi: cannot open ALSA sequencer\n");
        return 1;
    }
    snd_seq_set_client_name(seq, "VitalSynth");
    snd_seq_nonblock(seq, 1);
    const int myPort = snd_seq_create_simple_port(
        seq, "input", SND_SEQ_PORT_CAP_WRITE | SND_SEQ_PORT_CAP_SUBS_WRITE,
        SND_SEQ_PORT_TYPE_MIDI_GENERIC | SND_SEQ_PORT_TYPE_SYNTHESIZER |
            SND_SEQ_PORT_TYPE_APPLICATION);
    if (myPort < 0) { snd_seq_close(seq); return 1; }
    if (snd_seq_connect_from(seq, myPort, SND_SEQ_CLIENT_SYSTEM,
                             SND_SEQ_PORT_SYSTEM_ANNOUNCE) < 0)
        std::fprintf(stderr, "midi: warning: no announce subscription (no hot-plug)\n");
    const std::vector<std::string> midiPrefs = splitPrefs(cfg.midi_prefer);
    MidiSource current;
    selectSource(seq, myPort, midiPrefs, current, ring, cfg);
    if (current.client < 0)
        std::fprintf(stderr, "midi: no controller connected yet — will attach on plug-in\n");
    std::fprintf(stderr, "midi: ready — breath CC%d -> macro1, %zu patches\n",
                 cfg.breath_cc, patches.names.size());

    long dropped = 0;
    int patchIdx = 0;
    int lastPatchCcVal = 0;
    const int patchCount = int(patches.names.size());
    auto selectPatch = [&](int idx) {
        if (patchCount <= 0) return;
        patchIdx = ((idx % patchCount) + patchCount) % patchCount;
        ring.push({MidiMsg::Patch, int16_t(patchIdx), 0});
    };

    const int npfd = snd_seq_poll_descriptors_count(seq, POLLIN);
    struct pollfd pfds[8];
    while (g_running.load(std::memory_order_relaxed)) {
        const int count = snd_seq_poll_descriptors(seq, pfds, npfd > 8 ? 8 : (unsigned)npfd, POLLIN);
        if (poll(pfds, nfds_t(count), 250) <= 0) continue;
        snd_seq_event_t* ev = nullptr;
        for (;;) {
            const int r = snd_seq_event_input(seq, &ev);
            if (r == -EAGAIN) break;
            if (r == -ENOSPC) {
                std::fprintf(stderr, "midi: input overrun — all notes off\n");
                ring.push({MidiMsg::AllOff, 0, 0});
                continue;
            }
            if (r < 0 || !ev) break;
            MidiMsg m{};
            bool send = false;
            switch (ev->type) {
            case SND_SEQ_EVENT_NOTEON:
                if (ev->data.note.velocity == 0)
                    m = {MidiMsg::NoteOff, int16_t(ev->data.note.note), 0};
                else
                    m = {MidiMsg::NoteOn, int16_t(ev->data.note.note),
                         int16_t(ev->data.note.velocity)};
                send = true;
                break;
            case SND_SEQ_EVENT_NOTEOFF:
                m = {MidiMsg::NoteOff, int16_t(ev->data.note.note), 0};
                send = true;
                break;
            case SND_SEQ_EVENT_CONTROLLER: {
                const int cc = int(ev->data.control.param);
                const int v = int(ev->data.control.value);
                if (cc == cfg.patch_cc) {
                    if (v >= 64 && lastPatchCcVal < 64) selectPatch(patchIdx + 1);
                    lastPatchCcVal = v;
                    break;
                }
                m = {MidiMsg::CC, int16_t(cc), int16_t(v)};
                send = true;
                break;
            }
            case SND_SEQ_EVENT_PITCHBEND:
                m = {MidiMsg::PitchBend, int16_t(ev->data.control.value), 0};
                send = true;
                break;
            case SND_SEQ_EVENT_CHANPRESS:      // channel aftertouch (breath-as-AT)
                m = {MidiMsg::ChanPressure, int16_t(ev->data.control.value), 0};
                send = true;
                break;
            case SND_SEQ_EVENT_KEYPRESS:       // poly aftertouch
                m = {MidiMsg::PolyPressure, int16_t(ev->data.note.note),
                     int16_t(ev->data.note.velocity)};
                send = true;
                break;
            case SND_SEQ_EVENT_PGMCHANGE:
                selectPatch(int(ev->data.control.value));
                break;
            case SND_SEQ_EVENT_PORT_START:
                // A newly plugged controller may outrank the current one.
                selectSource(seq, myPort, midiPrefs, current, ring, cfg);
                break;
            case SND_SEQ_EVENT_PORT_EXIT:
                // The subscription is already gone; forget it before re-ranking
                // so the fallback controller can be picked up.
                if (ev->data.addr.client == current.client &&
                    ev->data.addr.port == current.port)
                    current = MidiSource();
                selectSource(seq, myPort, midiPrefs, current, ring, cfg);
                break;
            default:
                break;
            }
            // Hand the event to the audio thread BEFORE logging it: the log writes
            // unbuffered into journald, and if that ever stalls the event in hand
            // would wait behind it. push() is wait-free and copies m, so the log
            // below still sees identical data.
            if (send && !ring.push(m) && ++dropped % 256 == 1)
                std::fprintf(stderr, "midi: ring full, dropped %ld events\n", dropped);
            if (send && cfg.log_midi) {
                char nb[16];
                switch (m.type) {
                case MidiMsg::NoteOn:
                    std::fprintf(stderr, "midi: note on  %-4s (%d) vel %d\n",
                                 noteName(m.a, nb), m.a, m.b);
                    break;
                case MidiMsg::NoteOff:
                    std::fprintf(stderr, "midi: note off %-4s (%d)\n", noteName(m.a, nb), m.a);
                    break;
                case MidiMsg::CC:
                    logCC(m.a, m.b, int(m.a) == cfg.breath_cc);
                    break;
                case MidiMsg::PitchBend:
                    logBend(m.a);
                    break;
                case MidiMsg::ChanPressure:
                    logAftertouch(m.a);
                    break;
                default: break;
                }
            }
        }
    }
    snd_seq_close(seq);
    return 0;
}

} // namespace midi_in

// --------------------------------------------------------------- audio out
namespace audio_out {

bool hasPlayback(snd_ctl_t* ctl) {
    snd_pcm_info_t* pinfo = nullptr;
    snd_pcm_info_alloca(&pinfo);
    int dev = -1;
    while (snd_ctl_pcm_next_device(ctl, &dev) == 0 && dev >= 0) {
        snd_pcm_info_set_device(pinfo, unsigned(dev));
        snd_pcm_info_set_subdevice(pinfo, 0);
        snd_pcm_info_set_stream(pinfo, SND_PCM_STREAM_PLAYBACK);
        if (snd_ctl_pcm_info(ctl, pinfo) == 0) return true;
    }
    return false;
}

std::string pickDevice(const HostConfig& cfg, bool verbose) {
    if (cfg.audio_device != "auto") return cfg.audio_device;
    struct Card { int index; std::string id, name, driver; };
    std::vector<Card> cards;
    int card = -1;
    while (snd_card_next(&card) == 0 && card >= 0) {
        char ctlName[32];
        std::snprintf(ctlName, sizeof ctlName, "hw:%d", card);
        snd_ctl_t* ctl = nullptr;
        if (snd_ctl_open(&ctl, ctlName, 0) < 0) continue;
        snd_ctl_card_info_t* info = nullptr;
        snd_ctl_card_info_alloca(&info);
        if (snd_ctl_card_info(ctl, info) == 0 && hasPlayback(ctl))
            cards.push_back({card,
                             snd_ctl_card_info_get_id(info),
                             snd_ctl_card_info_get_name(info),
                             snd_ctl_card_info_get_driver(info)});
        snd_ctl_close(ctl);
    }
    auto contains = [](const std::string& hay, const char* needle) {
        return strcasestr(hay.c_str(), needle) != nullptr;
    };

    // A USB DAC is identified by its ALSA driver ("USB-Audio"), never by whether
    // the vendor happened to put "USB" in the product name -- most don't, and a
    // name-substring test silently skips them (e.g. "Cubilux HA-3").
    const std::vector<std::string> prefs = splitPrefs(cfg.audio_prefer);
    const Card* best = nullptr;
    const char* why = nullptr;
    int bestRank = INT_MAX;
    for (const auto& c : cards) {
        if (!contains(c.driver, "usb")) continue;
        int rank = int(prefs.size());          // unlisted USB DACs sort last, but still win
        for (size_t i = 0; i < prefs.size(); ++i)
            if (contains(c.id, prefs[i].c_str()) || contains(c.name, prefs[i].c_str())) {
                rank = int(i);
                break;
            }
        if (rank < bestRank) { bestRank = rank; best = &c; why = "USB DAC"; }
    }
    if (!best)
        for (const auto& c : cards)
            if (contains(c.name, "headphone") || contains(c.driver, "bcm2835")) {
                best = &c; why = "headphone jack"; break;
            }
    if (!best)
        for (const auto& c : cards)
            if (contains(c.name, "hdmi") || contains(c.driver, "hdmi")) {
                best = &c; why = "HDMI"; break;
            }
    if (!best && !cards.empty()) { best = &cards.front(); why = "first available"; }
    if (!best) return "default";

    // Address the card by ID, not index: USB enumeration order changes between
    // boots and hot-plugs, and plughw:N,0 would then point at the wrong card.
    char dev[64];
    std::snprintf(dev, sizeof dev, "plughw:CARD=%s,DEV=0", best->id.c_str());
    if (verbose)
        std::fprintf(stderr, "audio: auto-selected %s [%s] (%s)\n",
                     dev, best->name.c_str(), why);
    return dev;
}

int run(const HostConfig& cfg, HostSynth& synth, SpscRing<1024>& ring) {
    snd_pcm_t* pcm = nullptr;
    std::string device;
    int err = -1;
    for (int attempt = 0; g_running.load() && attempt < 150; ++attempt) {
        device = pickDevice(cfg, attempt == 0);
        err = snd_pcm_open(&pcm, device.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
        if (err == 0) break;
        if (attempt == 0)
            std::fprintf(stderr, "audio: open '%s' failed (%s), retrying...\n",
                         device.c_str(), snd_strerror(err));
        struct timespec ts{2, 0};
        nanosleep(&ts, nullptr);
    }
    if (err < 0 || !pcm) return 1;

    unsigned rate = unsigned(cfg.sample_rate);
    snd_pcm_uframes_t period = snd_pcm_uframes_t(cfg.period_frames);
    unsigned periods = unsigned(cfg.periods);
    snd_pcm_uframes_t bufferSize = 0;

    snd_pcm_hw_params_t* hw = nullptr;
    snd_pcm_hw_params_alloca(&hw);
    const char* stage = nullptr;
    if ((err = snd_pcm_hw_params_any(pcm, hw)) < 0) stage = "any";
    else if ((err = snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) stage = "access";
    else if ((err = snd_pcm_hw_params_set_format(pcm, hw, SND_PCM_FORMAT_S16_LE)) < 0) stage = "format";
    else if ((err = snd_pcm_hw_params_set_channels(pcm, hw, 2)) < 0) stage = "channels";
    else if ((err = snd_pcm_hw_params_set_rate_near(pcm, hw, &rate, nullptr)) < 0) stage = "rate";
    else if ((err = snd_pcm_hw_params_set_period_size_near(pcm, hw, &period, nullptr)) < 0) stage = "period";
    else if ((err = snd_pcm_hw_params_set_periods_near(pcm, hw, &periods, nullptr)) < 0) stage = "periods";
    else if ((err = snd_pcm_hw_params(pcm, hw)) < 0) stage = "commit";
    if (stage) {
        std::fprintf(stderr, "audio: hw_params (%s): %s\n", stage, snd_strerror(err));
        snd_pcm_close(pcm);
        return 1;
    }
    snd_pcm_hw_params_get_rate(hw, &rate, nullptr);
    snd_pcm_hw_params_get_period_size(hw, &period, nullptr);
    snd_pcm_hw_params_get_buffer_size(hw, &bufferSize);

    // ALSA's post-hw_params default is start_threshold = 1, so playback restarts
    // with as little as one period queued -- including after every recover()
    // below, which is how a single xrun cascades. Start on a full buffer instead.
    // Placed after the get_*() calls above: with bufferSize still 0 this would set
    // stop_threshold = 0, and writei would then return -EPIPE forever.
    // stop_threshold is left at buffer_size deliberately: snd-usb-audio disables
    // its low-latency playback path when stop_threshold > buffer_size.
    {
        snd_pcm_sw_params_t* sw = nullptr;
        snd_pcm_sw_params_alloca(&sw);
        if (snd_pcm_sw_params_current(pcm, sw) == 0) {
            snd_pcm_sw_params_set_start_threshold(pcm, sw, bufferSize);
            snd_pcm_sw_params_set_avail_min(pcm, sw, period);
            snd_pcm_sw_params_set_stop_threshold(pcm, sw, bufferSize);
            if (snd_pcm_sw_params(pcm, sw) < 0)
                std::fprintf(stderr, "audio: sw_params failed (using defaults)\n");
        }
    }
    std::fprintf(stderr, "audio: %s @ %u Hz, period %lu x %lu buffer (%.1f ms)\n",
                 device.c_str(), rate, (unsigned long)period, (unsigned long)bufferSize,
                 1000.0 * double(bufferSize) / double(rate));

    // Engine setup for the negotiated rate (audio thread not yet RT-hot).
    vital::SoundEngine* eng = synth.getEngine();
    synth.pauseProcessing(true);
    eng->setSampleRate(int(rate));
    eng->updateAllModulationSwitches();
    eng->setBpm(120.0f);
    synth.pauseProcessing(false);

    vital::Value* macro1 = nullptr;
    vital::Value* modWheelCtl = nullptr;
    vital::Value* pitchWheelCtl = nullptr;
    {
        auto& controls = synth.getControls();
        auto it = controls.find("macro_control_1");
        if (it != controls.end()) macro1 = it->second;
        it = controls.find("mod_wheel");
        if (it != controls.end()) modWheelCtl = it->second;
        it = controls.find("pitch_wheel");
        if (it != controls.end()) pitchWheelCtl = it->second;
    }

    struct sched_param sp{};
    sp.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0)
        std::fprintf(stderr, "audio: warning: SCHED_FIFO unavailable\n");

    const int n = int(period);
    std::vector<int16_t> frames(size_t(n) * 2);
    uint32_t dither = 0x12345678u;
    long xruns = 0;
    int fatalErr = 0;

    const float masterGain = std::pow(10.0f, cfg.master_gain_db * 0.05f);
    const float limiterThresh = cfg.limiter_thresh;
    float limiterGain = 1.0f;
    const float limiterRelease = 1.0f - std::exp(-1.0f / (0.050f * float(rate)));
    float breathTarget = 0.0f, breathSm = 0.0f;
    // Exact one-pole coefficient per chunk. The old form used the Euler
    // linearisation coef = chunk / (tau * rate), whose error grows as tau
    // shrinks -- 8.6% at 8 ms but 24% at 3 ms, reaching total bypass at
    // tau == chunk -- so the knob got less honest exactly where it is tuned.
    // It also silently tracked period_frames. chunk takes at most two values
    // for a negotiated period, so precompute both and keep expf() off the RT path.
    const int chunkFull = std::min(n, vital::kMaxBufferSize);
    const int chunkTail = n % vital::kMaxBufferSize;      // 0 when n divides evenly
    const float breathTauFrames = cfg.breath_smooth_ms * 0.001f * float(rate);
    const float breathCoefFull = 1.0f - std::exp(-float(chunkFull) / breathTauFrames);
    const float breathCoefTail = chunkTail
        ? 1.0f - std::exp(-float(chunkTail) / breathTauFrames) : breathCoefFull;
    double timeSec = 0.0;
    const double invRate = 1.0 / double(rate);
    const int stride = vital::poly_float::kSize;

    { MidiMsg m; while (ring.pop(m)) {} } // discard boot-window backlog

    while (g_running.load(std::memory_order_relaxed)) {
        MidiMsg m;
        while (ring.pop(m)) {
            switch (m.type) {
            case MidiMsg::NoteOn:  eng->noteOn(m.a, float(m.b) * (1.0f / 127.0f), 0, 0); break;
            case MidiMsg::NoteOff: eng->noteOff(m.a, 0.0f, 0, 0); break;
            case MidiMsg::CC:
                if (int(m.a) == cfg.breath_cc) breathTarget = float(m.b) * (1.0f / 127.0f);
                else if (m.a == 1) {
                    // Vital feeds mod wheel through BOTH the per-channel engine
                    // call and the "mod_wheel" control (the mod-matrix source).
                    const float v = float(m.b) * (1.0f / 127.0f);
                    eng->setModWheelAllChannels(v);
                    if (modWheelCtl) modWheelCtl->set(v);
                } else if (m.a == 64) { if (m.b >= 64) eng->sustainOn(0); else eng->sustainOff(0, 0); }
                else if (m.a == 120) eng->allSoundsOff();
                else if (m.a == 123) eng->allNotesOff(0, 0);
                break;
            case MidiMsg::PitchBend: {
                const float v = float(m.a) * (1.0f / 8192.0f);
                eng->setZonedPitchWheel(v, 0, 0);
                if (pitchWheelCtl) pitchWheelCtl->set(v); // "pitch_wheel" mod source
                break;
            }
            case MidiMsg::ChanPressure:
                eng->setChannelAftertouch(0, float(m.a) * (1.0f / 127.0f), 0);
                break;
            case MidiMsg::PolyPressure:
                eng->setAftertouch(float(m.a), float(m.b) * (1.0f / 127.0f), 0, 0);
                break;
            case MidiMsg::AllOff: eng->allNotesOff(0, 0); break;
            case MidiMsg::SoundsOff: eng->allSoundsOff(); break;
            case MidiMsg::Patch:
                g_requestedPatch.store(m.a, std::memory_order_release);
                break;
            }
        }

        // Render the period in engine-sized chunks (kMaxBufferSize = 128).
        bool rendered = false;
        if (synth.getCriticalSection().tryEnter()) {
            synth.processModulationChanges();
            const float patchGain = g_patchGain.load(std::memory_order_acquire);
            int done = 0;
            while (done < n) {
                const int chunk = std::min(n - done, vital::kMaxBufferSize);
                // smooth breath -> macro1 at chunk rate
                {
                    // 1 - exp(-x) is in (0,1] by construction, so the recursion below
                    // cannot overshoot and needs no clamp.
                    const float coef = (chunk == chunkFull) ? breathCoefFull : breathCoefTail;
                    breathSm += coef * (breathTarget - breathSm);
                    if (macro1) macro1->set(breathSm);
                    // Replicate a DAW rig where breath also drove channel
                    // aftertouch — many patches modulate from AT, not macro1.
                    if (cfg.breath_to_at) eng->setChannelAftertouch(0, breathSm, 0);
                }
                eng->correctToTime(timeSec);
                eng->process(chunk);
                const float* out = reinterpret_cast<const float*>(eng->output(0)->buffer);
                for (int i = 0; i < chunk; ++i) {
                    float l = out[size_t(stride) * size_t(i)] * masterGain * patchGain;
                    float r = out[size_t(stride) * size_t(i) + 1] * masterGain * patchGain;
                    // instant-attack peak limiter shared across channels
                    const float peak = std::max(std::fabs(l), std::fabs(r));
                    if (peak * limiterGain > limiterThresh) limiterGain = limiterThresh / peak;
                    else limiterGain += limiterRelease * (1.0f - limiterGain);
                    l *= limiterGain; r *= limiterGain;
                    dither = dither * 1664525u + 1013904223u;
                    const float r1 = float(dither >> 9) * (1.0f / 8388608.0f);
                    dither = dither * 1664525u + 1013904223u;
                    const float r2 = float(dither >> 9) * (1.0f / 8388608.0f);
                    const float tpdf = r1 - r2;
                    const int fi = (done + i) * 2;
                    frames[size_t(fi)] = int16_t(std::lrintf(
                        clampv(l * 32767.0f + tpdf, -32768.0f, 32767.0f)));
                    frames[size_t(fi) + 1] = int16_t(std::lrintf(
                        clampv(r * 32767.0f + tpdf, -32768.0f, 32767.0f)));
                }
                timeSec += chunk * invRate;
                done += chunk;
            }
            synth.getCriticalSection().exit();
            rendered = true;
        }
        if (!rendered) {
            std::fill(frames.begin(), frames.end(), int16_t(0));
            timeSec += n * invRate;
        }

        int done = 0;
        while (done < n && g_running.load(std::memory_order_relaxed)) {
            snd_pcm_sframes_t w = snd_pcm_writei(pcm, frames.data() + size_t(done) * 2,
                                                 snd_pcm_uframes_t(n - done));
            if (w >= 0) { done += int(w); continue; }
            ++xruns;
            // Report the first one and then every power of two, so a steady
            // trickle stays visible without ever flooding the RT path.
            if ((xruns & (xruns - 1)) == 0)
                std::fprintf(stderr, "audio: xrun (%ld so far)\n", xruns);
            w = snd_pcm_recover(pcm, int(w), 1);
            if (w < 0) { fatalErr = int(w); break; }
        }
        if (fatalErr) break;
    }

    if (xruns > 0) std::fprintf(stderr, "audio: %ld xrun(s) total\n", xruns);
    if (fatalErr) std::fprintf(stderr, "audio: unrecoverable: %s\n", snd_strerror(fatalErr));
    snd_pcm_drain(pcm);
    snd_pcm_close(pcm);
    return fatalErr ? 1 : 0;
}

} // namespace audio_out

// ------------------------------------------------------------- --normalize
// Render each patch offline across the EWI's playing range and write per-patch
// gain corrections to <patch_dir>/.gains. Runs as a separate process; no audio
// device.
//
// WHY THIS IS NOT PLAIN BS.1770. A single K-weighted energy integral says a
// sine and a dense saw at the same energy are equally loud. Ears disagree by
// up to 15 dB, because loudness sums across critical bands: energy spread over
// many bands is far louder than the same energy in one. Measured over this
// patch bank, K-weighted energy ranked patches against perceived (ISO 532-1
// Zwicker) loudness at rho=0.26 and left a 21.6 dB spread in apparent volume
// after "normalization". Splitting the same K-weighted signal into 6 octave
// bands, spreading each band into its neighbours, and summing E^0.20 — the
// compressive band-summation that underlies every loudness model — tracks the
// Zwicker solution at rho=0.98 with 0.61 dB residual. The exponent, band edges,
// spreading slope and note grid were fit against Zwicker loudness over 1232
// offline renders and cross-validated leave-one-out (held-out std 0.79 dB).
struct Biquad {
    double b0, b1, b2, a1, a2, z1 = 0.0, z2 = 0.0;
    inline double process(double x) {
        const double y = b0 * x + z1;
        z1 = b1 * x - a1 * y + z2;
        z2 = b2 * x - a2 * y;
        return y;
    }
    void reset() { z1 = z2 = 0.0; }
};

// Canonical BS.1770 K-weighting coefficients for 48 kHz.
static Biquad makeKShelf() { return {1.53512485958697, -2.69169618940638, 1.19839281085285,
                                     -1.69065929318241, 0.73248077421585}; }
static Biquad makeKHighpass() { return {1.0, -2.0, 1.0,
                                        -1.99004745483398, 0.99007225036621}; }

// Loudness model constants, all fit offline against ISO 532-1 Zwicker loudness
// (see tools/ in the analysis run). Changing any of them invalidates that fit.
namespace loudness {
constexpr int kFftOrder = 13;                       // 8192-point analysis
constexpr int kFftSize = 1 << kFftOrder;
constexpr int kHop = kFftSize / 2;                  // 50% overlap (Welch)
constexpr double kExponent = 0.20;                  // band-summation compression
// 6 octave-ish band edges in Hz; the top band deliberately runs to Nyquist.
constexpr double kBandEdges[7] = {0.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 24000.0};
constexpr int kBandCount = 6;
// Excitation spreads across neighbouring auditory filters, so a pure tone is
// not confined to one band the way a rectangular split implies. Without this,
// the model treated a sine as maximally narrowband and over-boosted it by 6.5 dB
// (24-theremin-whistle). 20 dB per band, symmetric; leave-one-out picked this
// configuration in 25 of 26 folds and it halves held-out error (1.55 -> 0.79 dB
// std, worst case 6.8 -> 2.5 dB).
constexpr double kSpreadDbPerBand = 20.0;
// Playing grid: 5 registers spanning the EWI's usable range (D#2..D#6 in
// concert terms) x 4 breath positions, weighted toward where playing happens.
constexpr int kNotes[5] = {39, 51, 63, 75, 87};
constexpr float kNoteWeights[5] = {0.7f, 1.0f, 1.0f, 1.0f, 0.6f};
constexpr float kBreaths[4] = {0.30f, 0.55f, 0.80f, 1.00f};
constexpr float kBreathWeights[4] = {0.6f, 1.0f, 1.0f, 0.8f};
constexpr float kSettleSec = 0.4f;                  // skipped: attack + filter settle
constexpr float kMeasureSec = 1.2f;                 // measured sustain window
// Headroom is budgeted against two separate ceilings, because the live limiter
// (0.95, instant attack, 50 ms release) treats sustained and transient level
// very differently. Sustained content above the threshold means CONTINUOUS gain
// reduction — pumping, and the whole patch ducked. A note attack above it is a
// few milliseconds of limiting, which is what a limiter is for.
constexpr double kSustainCeilingDb = -1.0;          // sustained level ceiling
// The grid samples 5 notes but the player can hold any note. Sustained level
// between grid points measured at most 1.15 dB above the sampled notes over
// this bank (mean 0.23, p90 0.89), so a small margin covers the gap.
constexpr double kSustainGridMarginDb = 1.5;
// Transient ceiling: attacks may cross the limiter threshold, but only so far
// before the attack is audibly blunted and the 50 ms release ducks the note
// behind it. The grid also renders isolated notes, while real playing sums a
// release tail into the next attack (measured +1.3 dB mean, +4.3 dB worst).
constexpr double kLimiterThreshDb = -0.45;          // 0.95 in dBFS
constexpr double kTransientAllowanceDb = 3.0;       // brief limiting is inaudible
constexpr double kLegatoSummationDb = 2.0;          // note-to-note overlap the grid misses
constexpr double kTruePeakCeilingDb =
    kLimiterThreshDb + kTransientAllowanceDb - kLegatoSummationDb;
constexpr double kMaxTargetDrop = 2.5;              // dB the bank may lose to a quiet patch
constexpr float kGainLimitDb = 15.0f;               // .gains parser clamps here too
// A grid cell this far below the patch's own loudest cell is inaudible in
// context; averaging it in the dB domain at full weight would let a dead
// corner of the keyboard dominate the patch's loudness (it cost 22-reese-growl
// 5.9 dB, enough to drag the whole bank's target down with it).
constexpr double kAudibleRangeDb = 40.0;
}

// Accumulates Welch-averaged band energies of the K-weighted signal, and the
// band-summed loudness L = 10*log10(sum_b E_b^a)/a. That scaling makes L move
// exactly 1:1 with dB of applied gain (E scales by g^2), so a gain solve on L
// is a plain subtraction.
class BandLoudness {
public:
    BandLoudness() : fft_(loudness::kFftOrder) {
        window_.resize(loudness::kFftSize);
        for (int i = 0; i < loudness::kFftSize; ++i)   // periodic Hann
            window_[size_t(i)] = 0.5 - 0.5 * std::cos(2.0 * M_PI * i / loudness::kFftSize);
        double sumSq = 0.0;
        for (double w : window_) sumSq += w * w;
        windowPower_ = sumSq / double(loudness::kFftSize);
        reset();
    }
    void reset() {
        pending_.clear();
        bands_.assign(loudness::kBandCount, 0.0);
        frames_ = 0;
    }
    void push(double sample) {
        pending_.push_back(sample);
        if (int(pending_.size()) >= loudness::kFftSize) {
            analyze(pending_.data());
            pending_.erase(pending_.begin(), pending_.begin() + loudness::kHop);
        }
    }
    // dB loudness of what was pushed; NaN-safe floor for silence.
    double loudnessDb() const {
        if (frames_ == 0) return -200.0;
        double sum = 0.0;
        for (int b = 0; b < loudness::kBandCount; ++b) {
            // Spread each band's energy into its neighbours before the
            // compressive sum, so narrowband content is not treated as if it
            // excited a single auditory filter.
            double excitation = 0.0;
            for (int j = 0; j < loudness::kBandCount; ++j)
                excitation += (bands_[size_t(j)] / double(frames_))
                              * std::pow(10.0, -std::abs(b - j)
                                                   * loudness::kSpreadDbPerBand / 10.0);
            sum += std::pow(std::max(excitation, 1e-20), loudness::kExponent);
        }
        return 10.0 * std::log10(std::max(sum, 1e-30)) / loudness::kExponent;
    }

private:
    void analyze(const double* src) {
        std::vector<float> buf(size_t(loudness::kFftSize) * 2, 0.0f);
        for (int i = 0; i < loudness::kFftSize; ++i)
            buf[size_t(i)] = float(src[i] * window_[size_t(i)]);
        fft_.performFrequencyOnlyForwardTransform(buf.data());
        // performFrequencyOnlyForwardTransform yields magnitudes; convert to a
        // one-sided power spectral estimate normalized so a full-scale sine
        // reads its own mean-square regardless of FFT size or window shape.
        const double norm = 1.0 / (windowPower_ * double(loudness::kFftSize)
                                   * double(loudness::kFftSize));
        const double binHz = 48000.0 / double(loudness::kFftSize);
        for (int bin = 0; bin <= loudness::kFftSize / 2; ++bin) {
            const double mag = double(buf[size_t(bin)]);
            double power = mag * mag * norm;
            if (bin > 0 && bin < loudness::kFftSize / 2) power *= 2.0;  // one-sided
            const double hz = bin * binHz;
            for (int b = 0; b < loudness::kBandCount; ++b) {
                if (hz >= loudness::kBandEdges[b] && hz < loudness::kBandEdges[b + 1]) {
                    bands_[size_t(b)] += power;
                    break;
                }
            }
        }
        ++frames_;
    }
    juce::dsp::FFT fft_;
    std::vector<double> window_;
    double windowPower_ = 1.0;
    std::vector<double> pending_;
    std::vector<double> bands_;
    int frames_ = 0;
};

struct PatchMeasurement {
    bool loaded = false;
    double loudnessDb = -200.0;      // zone-weighted band loudness at unity gain
    double sustainPeakDbfs = -200.0; // worst SUSTAINED peak
    double absPeakDbfs = -200.0;     // worst peak incl. attack
    int silentConditions = 0;        // grid cells inaudible against the patch itself
    double worstBreathDipDb = 0.0;   // largest drop caused by MORE breath
};

static int runNormalize(const HostConfig& cfg) {
    using namespace loudness;
    const PatchList patches = scanPatches(cfg.patch_dir);
    if (patches.paths.empty()) {
        std::fprintf(stderr, "normalize: no patches in %s\n", cfg.patch_dir.c_str());
        return 1;
    }
    static HostSynth synth;
    vital::SoundEngine* eng = synth.getEngine();
    const int sr = 48000;                    // K-weighting coefficients are 48k
    synth.pauseProcessing(true);
    eng->setSampleRate(sr);
    eng->updateAllModulationSwitches();
    eng->setBpm(120.0f);
    synth.pauseProcessing(false);
    vital::Value* macro1 = nullptr;
    {
        auto it = synth.getControls().find("macro_control_1");
        if (it != synth.getControls().end()) macro1 = it->second;
    }

    const int chunk = vital::kMaxBufferSize;
    const int stride = vital::poly_float::kSize;
    double t = 0.0;
    Biquad shelfL = makeKShelf(), shelfR = makeKShelf();
    Biquad hpL = makeKHighpass(), hpR = makeKHighpass();
    BandLoudness meterL, meterR;
    double peakAbs = 0.0, peakSustain = 0.0;

    // Renders `sec` of audio. `measure` feeds the band meters and the sustain
    // peak; `trackPeak` accumulates the true peak. They differ because the note
    // attack must count toward the true peak but must NOT be measured as
    // loudness, while the post-allSoundsOff flush is neither — allSoundsOff
    // hard-resets the voices, and that kill transient is an artifact of the
    // measurement rig that would otherwise inflate peaks by up to 4.4 dB.
    auto renderSec = [&](float sec, bool measure, bool trackPeak) {
        const int total = int(sec * float(sr));
        for (int done = 0; done < total; done += chunk) {
            synth.processModulationChanges();
            eng->correctToTime(t);
            eng->process(chunk);
            const float* o = reinterpret_cast<const float*>(eng->output(0)->buffer);
            for (int k = 0; k < chunk; ++k) {
                const double rawL = o[size_t(stride) * size_t(k)];
                const double rawR = o[size_t(stride) * size_t(k) + 1];
                if (trackPeak) {
                    const double mag = std::max(std::fabs(rawL), std::fabs(rawR));
                    peakAbs = std::max(peakAbs, mag);
                    if (measure) peakSustain = std::max(peakSustain, mag);
                }
                if (measure) {
                    meterL.push(hpL.process(shelfL.process(rawL)));
                    meterR.push(hpR.process(shelfR.process(rawR)));
                }
            }
            t += chunk / double(sr);
        }
    };

    // ---- pass 1: measure every patch at unity gain.
    std::vector<PatchMeasurement> meas(patches.paths.size());
    for (size_t i = 0; i < patches.paths.size(); ++i) {
        std::string error;
        juce::File file{juce::String(patches.paths[i])};
        if (!synth.loadFromFile(file, error)) {
            std::fprintf(stderr, "normalize: skip %s (%s) -> 0.00 dB\n",
                         patches.names[i].c_str(), error.c_str());
            continue;
        }
        if (cfg.oversampling >= 0) {
            auto it = synth.getControls().find("oversampling");
            if (it != synth.getControls().end()) it->second->set(vital::mono_float(cfg.oversampling));
            eng->checkOversampling();
        }
        peakAbs = 0.0;
        peakSustain = 0.0;
        double worstDip = 0.0;
        // Measure one discarded note first. Without it the engine state a patch
        // is measured in depends on what ran before it, and the first patch of
        // the bank reads 0.4-0.9 dB quiet against later ones.
        if (macro1) macro1->set(kBreaths[3]);
        eng->noteOn(kNotes[2], 0.79f, 0, 0);
        eng->setChannelAftertouch(0, kBreaths[3], 0);
        renderSec(0.3f, false, false);
        eng->noteOff(kNotes[2], 0.0f, 0, 0);
        eng->allSoundsOff();
        renderSec(0.25f, false, false);

        std::vector<double> condDb, condWeight;
        condDb.reserve(20); condWeight.reserve(20);
        for (int ni = 0; ni < 5; ++ni) {
            double prevBreathDb = -1e9;
            for (int bi = 0; bi < 4; ++bi) {
                const float level = kBreaths[bi];
                if (macro1) macro1->set(level);
                shelfL.reset(); shelfR.reset(); hpL.reset(); hpR.reset();
                meterL.reset(); meterR.reset();
                eng->noteOn(kNotes[ni], 0.79f, 0, 0);
                eng->setChannelAftertouch(0, level, 0);  // breath_to_at mirroring
                renderSec(kSettleSec, false, true);      // attack: peak only
                renderSec(kMeasureSec, true, true);      // sustain: peak + loudness
                eng->noteOff(kNotes[ni], 0.0f, 0, 0);
                eng->allSoundsOff();
                renderSec(0.25f, false, false);          // flush; kill transient ignored
                // Average the two channels' band loudness in the energy domain
                // (a dB average would penalize hard-panned or wide patches).
                const double lDb = meterL.loudnessDb(), rDb = meterR.loudnessDb();
                const double db = 10.0 * std::log10(
                    0.5 * (std::pow(10.0, lDb / 10.0) + std::pow(10.0, rDb / 10.0)));
                if (prevBreathDb > -1e8)                 // quieter with MORE breath
                    worstDip = std::max(worstDip, prevBreathDb - db);
                prevBreathDb = db;
                condDb.push_back(db);
                condWeight.push_back(double(kNoteWeights[ni]) * double(kBreathWeights[bi]));
            }
        }
        // Aggregate only over cells that are audible relative to the patch
        // itself. A dead corner of the keyboard is not part of how loud the
        // patch sounds, but in a dB-domain mean it would dominate.
        const double loudest = *std::max_element(condDb.begin(), condDb.end());
        double weighted = 0.0, weightSum = 0.0;
        int silent = 0;
        for (size_t c = 0; c < condDb.size(); ++c) {
            if (condDb[c] < loudest - kAudibleRangeDb) { ++silent; continue; }
            weighted += condDb[c] * condWeight[c];
            weightSum += condWeight[c];
        }
        meas[i].loaded = true;
        meas[i].loudnessDb = weightSum > 0.0 ? weighted / weightSum : -200.0;
        meas[i].absPeakDbfs = 20.0 * std::log10(peakAbs > 1e-6 ? peakAbs : 1e-6);
        meas[i].sustainPeakDbfs = 20.0 * std::log10(peakSustain > 1e-6 ? peakSustain : 1e-6);
        meas[i].silentConditions = silent;
        meas[i].worstBreathDipDb = worstDip;
    }

    // ---- choose a target every patch can actually reach.
    // Equal loudness is worthless if the quiet patches have to clip to get
    // there, so the target starts at the bank median and drops toward the
    // least-headroom patch — but by at most kMaxTargetDrop, so one pathological
    // patch cannot drag the whole bank down. Anything still short is capped at
    // its headroom and reported.
    std::vector<double> loud;
    for (const auto& m : meas)
        if (m.loaded) loud.push_back(m.loudnessDb);
    if (loud.empty()) {
        std::fprintf(stderr, "normalize: no patch loaded successfully\n");
        return 1;
    }
    std::vector<double> sorted = loud;
    std::sort(sorted.begin(), sorted.end());
    const double median = sorted[sorted.size() / 2];
    // Headroom is the tighter of the sustained and transient constraints.
    auto headroomOf = [](const PatchMeasurement& m) {
        return std::min(kSustainCeilingDb - kSustainGridMarginDb - m.sustainPeakDbfs,
                        kTruePeakCeilingDb - m.absPeakDbfs);
    };
    // Lower the target toward the least-headroom patch so it can still reach
    // equal loudness — but a patch that cannot be reached even at the full drop
    // is EXCLUDED rather than clamped. Chasing it further would attenuate the
    // whole bank while its own gain, pinned at its headroom, never moves: that
    // is how the first version lost 6 dB for everyone and matched no better.
    double feasible = median;
    for (const auto& m : meas) {
        if (!m.loaded) continue;
        const double reach = m.loudnessDb + headroomOf(m);
        if (reach < median - kMaxTargetDrop) continue;   // unreachable: will be capped
        feasible = std::min(feasible, reach);
    }
    std::fprintf(stderr, "normalize: target %.2f dB (median %.2f, %zu patches)\n",
                 feasible, median, loud.size());

    // ---- pass 2: emit gains.
    std::string out;
    for (size_t i = 0; i < patches.paths.size(); ++i) {
        const PatchMeasurement& m = meas[i];
        if (!m.loaded) {
            // Keep the patch in .gains at unity rather than dropping the line —
            // a missing entry is indistinguishable from "measured 0 dB" later on,
            // and a patch caught mid-copy would silently stay unnormalized.
            char line[320];
            std::snprintf(line, sizeof line, "%s 0.00\n", patches.names[i].c_str());
            out += line;
            continue;
        }
        const double headroom = headroomOf(m);
        const double want = feasible - m.loudnessDb;
        const double capped = std::min(want, headroom);
        const float adjust = clampv(float(capped), -kGainLimitDb, kGainLimitDb);
        char line[320];
        std::snprintf(line, sizeof line, "%s %.2f\n", patches.names[i].c_str(), adjust);
        out += line;
        // Report the shortfall against what was actually WRITTEN, so loss to
        // the +/-15 dB clamp cannot slip out silently, and name the cause.
        std::string flags;
        const double shortfall = want - double(adjust);
        if (shortfall > 0.25) {
            char buf[128];
            std::snprintf(buf, sizeof buf, " %s(%.1f dB short)",
                          capped < want - 0.01 ? "HEADROOM-CAPPED" : "CLAMPED", shortfall);
            flags += buf;
        }
        if (m.silentConditions) {
            char buf[64];
            std::snprintf(buf, sizeof buf, " SILENT(%d cells)", m.silentConditions);
            flags += buf;
        }
        if (m.worstBreathDipDb > 3.0) {
            char buf[64];
            std::snprintf(buf, sizeof buf, " BREATH-DIP(%.1f dB)", m.worstBreathDipDb);
            flags += buf;
        }
        std::fprintf(stderr,
                     "normalize: %-28s %7.2f dB loud, sustain %6.1f peak %6.1f dBFS -> %+6.2f dB%s\n",
                     patches.names[i].c_str(), m.loudnessDb, m.sustainPeakDbfs,
                     m.absPeakDbfs, adjust, flags.c_str());
    }
    const std::string gainsPath = cfg.patch_dir + "/.gains";
    if (FILE* g = std::fopen(gainsPath.c_str(), "w")) {
        std::fwrite(out.data(), 1, out.size(), g);
        std::fclose(g);
        std::fprintf(stderr, "normalize: wrote %s\n", gainsPath.c_str());
        return 0;
    }
    std::fprintf(stderr, "normalize: cannot write %s\n", gainsPath.c_str());
    return 1;
}

// ---------------------------------------------------------------------- main
static void enableFlushToZero() {
#if defined(__aarch64__)
    uint64_t fpcr;
    asm volatile("mrs %0, fpcr" : "=r"(fpcr));
    fpcr |= (1ull << 24);
    asm volatile("msr fpcr, %0" ::"r"(fpcr));
#elif defined(__SSE__) || defined(__x86_64__)
    unsigned mxcsr;
    asm volatile("stmxcsr %0" : "=m"(mxcsr));
    mxcsr |= 0x8040;
    asm volatile("ldmxcsr %0" ::"m"(mxcsr));
#endif
}

int main(int argc, char** argv) {
    HostConfig cfg;
    for (int i = 1; i < argc; ++i)
        if (!std::strcmp(argv[i], "--config") && i + 1 < argc) {
            if (!cfg.loadFile(argv[i + 1]))
                std::fprintf(stderr, "config: cannot read %s (defaults)\n", argv[i + 1]);
        }
    bool normalizeMode = false;
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--device") && i + 1 < argc) cfg.audio_device = argv[++i];
        else if (!std::strcmp(argv[i], "--patch-dir") && i + 1 < argc) cfg.patch_dir = argv[++i];
        else if (!std::strcmp(argv[i], "--normalize")) normalizeMode = true;
        else if (!std::strcmp(argv[i], "--config")) ++i;
        else if (!std::strcmp(argv[i], "--help") || !std::strcmp(argv[i], "-h")) {
            std::printf("vitalsynth [--config FILE] [--device DEV] [--patch-dir DIR] [--normalize]\n");
            return 0;
        }
    }
    cfg.sanitize();
    enableFlushToZero();

    if (normalizeMode) return runNormalize(cfg);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
        std::fprintf(stderr, "warning: mlockall failed\n");

    const PatchList patches = scanPatches(cfg.patch_dir);
    if (patches.paths.empty())
        std::fprintf(stderr, "patches: NONE found in %s — running init preset\n",
                     cfg.patch_dir.c_str());
    else {
        std::fprintf(stderr, "patches: %zu found:", patches.paths.size());
        for (const auto& nm : patches.names) std::fprintf(stderr, " [%s]", nm.c_str());
        std::fprintf(stderr, "\n");
    }

    static HostSynth synth; // heavy ctor: builds the whole Vital DSP graph
    static SpscRing<1024> ring;

    if (!patches.paths.empty()) g_requestedPatch.store(0);

    std::thread loader([&] { loaderLoop(synth, cfg, patches); });
    std::thread midi([&] {
        if (midi_in::run(cfg, patches, ring) != 0) g_running.store(false);
    });

    const int rc = audio_out::run(cfg, synth, ring);
    g_running.store(false);
    midi.join();
    loader.join();
    return rc;
}
