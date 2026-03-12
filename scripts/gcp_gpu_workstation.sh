#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  gcp_gpu_workstation.sh create
  gcp_gpu_workstation.sh configure
  gcp_gpu_workstation.sh firewall
  gcp_gpu_workstation.sh full
  gcp_gpu_workstation.sh stop
  gcp_gpu_workstation.sh ip

Required environment variables:
  VM_NAME                  Name of the Compute Engine instance.
  ZONE                     Zone for the VM, for example us-west1-b.

Create-related environment variables:
  MACHINE_TYPE             Defaults to g2-standard-8.
  ACCELERATOR              Defaults to nvidia-l4-vws.
  NUM_GPUS                 Defaults to 1.
  BOOT_DISK_SIZE           Defaults to 100.
  BOOT_DISK_TYPE           Defaults to pd-ssd.
  NETWORK                  Defaults to default.

TigerVNC-related environment variables:
  VNC_PASSWORD             Required for configure/full. Max 8 characters due to vncpasswd format.
  VNC_DISPLAY              Defaults to 1, which maps to TCP port 5901.
  VNC_GEOMETRY             Defaults to 1920x1080.
  VNC_DEPTH                Defaults to 24.

Optional environment variables:
  FIREWALL_RULE_NAME       Defaults to allow-vnc.
  VNC_SOURCE_RANGES        Defaults to 0.0.0.0/0.
  USER_PASSWORD            Sets the Linux user's password non-interactively.
  SKIP_REBOOTS             Set to 1 to skip reboot/wait cycles.

Examples:
  export VM_NAME=test-workstation
  export ZONE=us-west1-b
  export VNC_PASSWORD=secret12
  ./scripts/gcp_gpu_workstation.sh full
EOF
}

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Missing required command: $cmd" >&2
    exit 1
  fi
}

require_env() {
  local name="$1"
  if [[ -z "${!name:-}" ]]; then
    echo "Missing required environment variable: $name" >&2
    exit 1
  fi
}

wait_for_ssh() {
  local tries=30
  local delay=20
  local i

  for ((i = 1; i <= tries; i++)); do
    if gcloud compute ssh "${VM_NAME}" --zone "${ZONE}" --command "true" >/dev/null 2>&1; then
      return 0
    fi
    sleep "${delay}"
  done

  echo "Timed out waiting for SSH access to ${VM_NAME} in ${ZONE}." >&2
  exit 1
}

run_remote_script() {
  local script_text="$1"
  gcloud compute ssh "${VM_NAME}" --zone "${ZONE}" --command "bash -s" <<<"${script_text}"
}

remote_base_script() {
  cat <<EOF
set -euo pipefail

if [[ -n "${USER_PASSWORD:-}" ]]; then
  echo "\${USER}:${USER_PASSWORD}" | sudo chpasswd
fi

sudo apt update
sudo apt install -y ubuntu-drivers-common build-essential libvulkan1 gcc-12 "linux-headers-\$(uname -r)"
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 12
sudo ubuntu-drivers install

if [[ "${SKIP_REBOOTS:-0}" != "1" ]]; then
  sudo reboot
fi
EOF
}

remote_desktop_script() {
cat <<EOF
set -euo pipefail

sudo apt update
sudo DEBIAN_FRONTEND=noninteractive apt install -y kubuntu-desktop tigervnc-standalone-server tigervnc-common dbus-x11

mkdir -p "\$HOME/.vnc"
chmod 700 "\$HOME/.vnc"
printf '%s\n%s\n\n' "${VNC_PASSWORD}" "${VNC_PASSWORD}" | vncpasswd "\$HOME/.vnc/passwd"
chmod 600 "\$HOME/.vnc/passwd"

cat > "\$HOME/.vnc/xstartup" <<'XEOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
export XDG_SESSION_TYPE=x11
exec startplasma-x11
XEOF
chmod +x "\$HOME/.vnc/xstartup"

mkdir -p "\$HOME/.config/systemd/user"
cat > "\$HOME/.config/systemd/user/vncserver.service" <<'XEOF'
[Unit]
Description=TigerVNC server
After=network.target

[Service]
Type=forking
PIDFile=%h/.vnc/%H:${VNC_DISPLAY}.pid
ExecStart=/usr/bin/vncserver :${VNC_DISPLAY} -geometry ${VNC_GEOMETRY} -depth ${VNC_DEPTH} -localhost no -rfbauth %h/.vnc/passwd
ExecStop=/usr/bin/vncserver -kill :${VNC_DISPLAY}
Restart=on-failure

[Install]
WantedBy=default.target
XEOF

sudo loginctl enable-linger "\$USER"
systemctl --user daemon-reload
systemctl --user enable vncserver.service
systemctl --user restart vncserver.service

if [[ "${SKIP_REBOOTS:-0}" != "1" ]]; then
  sudo reboot
fi
EOF
}

create_vm() {
  require_env VM_NAME
  require_env ZONE

  local machine_type="${MACHINE_TYPE:-g2-standard-8}"
  local accelerator="${ACCELERATOR:-nvidia-l4-vws}"
  local num_gpus="${NUM_GPUS:-1}"
  local boot_disk_size="${BOOT_DISK_SIZE:-100}"
  local boot_disk_type="${BOOT_DISK_TYPE:-pd-ssd}"
  local network="${NETWORK:-default}"

  gcloud compute instances create "${VM_NAME}" \
    --zone="${ZONE}" \
    --machine-type="${machine_type}" \
    --accelerator="type=${accelerator},count=${num_gpus}" \
    --maintenance-policy=TERMINATE \
    --image-project=ubuntu-os-cloud \
    --image-family=ubuntu-2204-lts \
    --boot-disk-size="${boot_disk_size}" \
    --boot-disk-type="${boot_disk_type}" \
    --network="${network}"
}

configure_vm() {
  require_env VM_NAME
  require_env ZONE
  require_env VNC_PASSWORD

  if (( ${#VNC_PASSWORD} > 8 )); then
    echo "VNC_PASSWORD must be 8 characters or fewer." >&2
    exit 1
  fi

  wait_for_ssh
  run_remote_script "$(remote_base_script)"

  if [[ "${SKIP_REBOOTS:-0}" != "1" ]]; then
    wait_for_ssh
  fi

  run_remote_script "$(remote_desktop_script)"

  if [[ "${SKIP_REBOOTS:-0}" != "1" ]]; then
    wait_for_ssh
  fi
}

create_firewall() {
  local firewall_rule_name="${FIREWALL_RULE_NAME:-allow-vnc}"
  local source_ranges="${VNC_SOURCE_RANGES:-0.0.0.0/0}"
  local display="${VNC_DISPLAY:-1}"
  local port=$((5900 + display))

  if gcloud compute firewall-rules describe "${firewall_rule_name}" >/dev/null 2>&1; then
    echo "Firewall rule ${firewall_rule_name} already exists; skipping."
    return 0
  fi

  gcloud compute firewall-rules create "${firewall_rule_name}" \
    --action=ALLOW \
    --rules="tcp:${port}" \
    --source-ranges="${source_ranges}"
}

stop_vm() {
  require_env VM_NAME
  require_env ZONE
  gcloud compute instances stop "${VM_NAME}" --zone "${ZONE}"
}

show_ip() {
  require_env VM_NAME
  require_env ZONE
  gcloud compute instances describe "${VM_NAME}" --zone "${ZONE}" \
    --format='get(networkInterfaces[0].accessConfigs[0].natIP)'
}

main() {
  require_cmd gcloud

  if [[ $# -ne 1 ]]; then
    usage
    exit 1
  fi

  case "$1" in
    create)
      create_vm
      ;;
    configure)
      configure_vm
      ;;
    firewall)
      create_firewall
      ;;
    full)
      create_vm
      configure_vm
      create_firewall
      ;;
    stop)
      stop_vm
      ;;
    ip)
      show_ip
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      echo "Unknown command: $1" >&2
      usage
      exit 1
      ;;
  esac
}

main "$@"
