#!/bin/bash
# Volume control for whichever card vitalsynth is actually playing through.
#
# Two reasons this is not just "amixer -c <fixed-card>":
#  - the dongle in use changes (only the ATR2x has an inline knob; the HA-3
#    has none), so the target card is resolved at every keypress;
#  - amixer's relative percent steps are broken on bcm2835_headpho, whose
#    range is -10239..400: "4%+" from 50% lands on 0%, and from max it wraps
#    to 8%. So step in raw control units and clamp by hand.
set -u

STEP=${STEP:-4}          # percent of the control's raw range per press

# Card vitalsynth has open. Other apps may hold cards too, so the substream
# owned by the service's own PID wins; otherwise take the first open one.
active_card() {
    local want st card first=""
    want=$(systemctl show -p MainPID --value vitalsynth 2>/dev/null)
    for st in /proc/asound/card*/pcm*p/sub*/status; do
        [ -e "$st" ] || continue
        grep -q '^closed' "$st" && continue
        card=${st#/proc/asound/card}; card=${card%%/*}
        [ -n "$first" ] || first=$card
        if [ -n "$want" ] && [ "$want" != 0 ] && grep -qE "^owner_pid +: +$want\$" "$st"; then
            echo "$card"; return 0
        fi
    done
    [ -n "$first" ] && { echo "$first"; return 0; }
    return 1
}

# Nothing open yet (e.g. ExecStartPre): mirror the synth's own preference,
# read from the same config key rather than a second hardcoded list.
fallback_card() {
    local id prefs
    prefs=$(sed -n 's/^[[:space:]]*audio_prefer[[:space:]]*=[[:space:]]*\([^#]*\).*/\1/p' \
            /etc/vitalsynth.conf 2>/dev/null | tail -1)
    prefs=$(echo "${prefs:-HA3,ATR2xUSB}" | tr ',' ' ')
    for id in $prefs Headphones; do
        [ -n "$id" ] || continue
        [ -e "/proc/asound/$id" ] && { echo "$id"; return 0; }
    done
    return 1
}

# 'PCM' on the HA-3 and the jack, 'Speaker' on the ATR2x. Never a mic control:
# the ATR2x's 'Mic' also reports pvolume.
pick_control() {
    local card=$1 c n avail
    avail=$(amixer -c "$card" scontrols 2>/dev/null | sed -n "s/^Simple mixer control '\([^']*\)'.*/\1/p")
    for c in PCM Speaker Master Headphone; do
        printf '%s\n' "$avail" | grep -qx "$c" || continue
        amixer -c "$card" sget "$c" 2>/dev/null | grep -q pvolume || continue
        echo "$c"; return 0
    done
    while IFS= read -r n; do
        [ -n "$n" ] || continue
        case $n in *[Mm]ic*|*[Cc]apture*) continue;; esac
        amixer -c "$card" sget "$n" 2>/dev/null | grep -q pvolume || continue
        echo "$n"; return 0
    done <<< "$avail"
    return 1
}

card=${VITALVOL_CARD:-}     # override for testing a specific card
if [ "${1:-}" = "startup-max" ]; then
    # Deliberately ignore active_card: the synth is not running yet, and
    # PipeWire may have an unrelated card open.
    [ -n "$card" ] || card=$(fallback_card) || exit 0
else
    [ -n "$card" ] || card=$(active_card) || card=$(fallback_card) || exit 0
fi
ctl=$(pick_control "$card") || exit 0

info=$(amixer -c "$card" sget "$ctl" 2>/dev/null) || exit 0
min=$(printf '%s\n' "$info" | sed -n 's/.*Limits: Playback \(-\?[0-9]*\) - \(-\?[0-9]*\).*/\1/p' | head -1)
max=$(printf '%s\n' "$info" | sed -n 's/.*Limits: Playback \(-\?[0-9]*\) - \(-\?[0-9]*\).*/\2/p' | head -1)
cur=$(printf '%s\n' "$info" | sed -n 's/.*: Playback \(-\?[0-9][0-9]*\) \[.*/\1/p' | head -1)
[ -n "$min" ] && [ -n "$max" ] && [ -n "$cur" ] || exit 0

span=$(( max - min ))
step=$(( span * STEP / 100 ))
[ "$step" -lt 1 ] && step=1

case "${1:-}" in
    up)     new=$(( cur + step )) ;;
    down)   new=$(( cur - step )) ;;
    max|startup-max) new=$max ;;
    toggle) amixer -q -c "$card" sset "$ctl" toggle 2>/dev/null || true; exit 0 ;;
    show)   echo "card=$card control=$ctl range=$min..$max cur=$cur step=$step"
            printf '%s\n' "$info" | grep -E 'Front Left:|Mono:'; exit 0 ;;
    *) echo "usage: ${0##*/} {up|down|max|startup-max|toggle|show}" >&2; exit 2 ;;
esac

[ "$new" -gt "$max" ] && new=$max
[ "$new" -lt "$min" ] && new=$min
amixer -q -c "$card" sset "$ctl" -- "$new" unmute
