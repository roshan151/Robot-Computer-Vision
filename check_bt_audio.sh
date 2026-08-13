#!/usr/bin/env bash
# Determine whether the JBL buds negotiate mSBC (16 kHz wideband) or fall back
# to CVSD (8 kHz narrowband), and switch to mSBC if available.
#
# 16 kHz is the native input rate for Gemini Live / OpenAI Realtime.
# At 8 kHz everything above 4 kHz is gone before the model sees it.

set -uo pipefail

MAC="${BT_MAC:-B4:84:D5:B9:14:23}"
CARD="bluez_card.${MAC//:/_}"
UID_N="$(id -u "${BT_USER:-roshan151}")"
PA=(sudo -u "#${UID_N}" "XDG_RUNTIME_DIR=/run/user/${UID_N}" pactl)

echo "== card: ${CARD}"
"${PA[@]}" list cards 2>/dev/null | grep -q "${CARD}" || {
  echo "!! card not found. Connect first:  bluetoothctl connect ${MAC}"
  exit 1
}

echo
echo "== available head-unit profiles"
"${PA[@]}" list cards | sed -n "/${CARD}/,/^Card #/p" \
  | grep -oE 'headset-head-unit[a-z0-9-]*' | sort -u

echo
if "${PA[@]}" list cards | grep -q 'headset-head-unit-msbc'; then
  echo "== mSBC available -> switching (16 kHz wideband)"
  "${PA[@]}" set-card-profile "${CARD}" headset-head-unit-msbc
else
  echo "!! mSBC NOT offered. Either the buds are HFP<1.6, or - more likely -"
  echo "   msbc is disabled in the audio stack. Enable it, reboot, re-run:"
  echo
  echo "   PulseAudio  /etc/pulse/default.pa:"
  echo "     load-module module-bluetooth-discover enable_msbc=true"
  echo
  echo "   WirePlumber (Pi OS Bookworm default):"
  echo "     ~/.config/wireplumber/wireplumber.conf.d/51-bluez.conf"
  echo '     monitor.bluez.properties = { bluez5.enable-msbc = true }'
  echo
  echo "   Also verify the Pi's onboard controller does transparent SCO -"
  echo "   CYW43455 mSBC support is inconsistent. A ~\$10 USB BT dongle is the"
  echo "   usual fix, and it takes BT off the WiFi antenna as a bonus."
fi

echo
echo "== negotiated source rate (this is the number that matters)"
"${PA[@]}" list sources | grep -A12 "${MAC//:/_}" \
  | grep -E 'Name:|Sample Specification:|Active Port:'
