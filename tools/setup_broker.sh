#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_BASE="${XDG_CONFIG_HOME:-$HOME/.config}/autonomic_turtlebot3"
ENV_FILE="${AUTONOMIC_ENV_FILE:-$CONFIG_BASE/runtime.env}"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Missing $ENV_FILE. Run tools/generate_runtime_env.sh first." >&2
  exit 1
fi

set -a
source "$ENV_FILE"
set +a
: "${AUTONOMIC_MQTT_PASSWORD:?robot MQTT password is missing}"
: "${AUTONOMIC_MQTT_DASHBOARD_PASSWORD:?dashboard MQTT password is missing}"
: "${AUTONOMIC_MQTT_AUX_PASSWORD:?auxiliary MQTT password is missing}"

sudo install -m 0644 "$PROJECT_ROOT/mosquitto/autonomic.conf" /etc/mosquitto/conf.d/autonomic.conf
sudo install -m 0644 "$PROJECT_ROOT/mosquitto/autonomic.acl" /etc/mosquitto/autonomic.acl
sudo mosquitto_passwd -b -c /etc/mosquitto/autonomic.passwd autonomic_robot "$AUTONOMIC_MQTT_PASSWORD"
sudo mosquitto_passwd -b /etc/mosquitto/autonomic.passwd autonomic_dashboard "$AUTONOMIC_MQTT_DASHBOARD_PASSWORD"
sudo mosquitto_passwd -b /etc/mosquitto/autonomic.passwd autonomic_auxiliary "$AUTONOMIC_MQTT_AUX_PASSWORD"
sudo chown root:mosquitto /etc/mosquitto/autonomic.passwd /etc/mosquitto/autonomic.acl
sudo chmod 0640 /etc/mosquitto/autonomic.passwd /etc/mosquitto/autonomic.acl
sudo systemctl enable --now mosquitto
sudo systemctl restart mosquitto

echo "Mosquitto configured with separate robot and dashboard accounts."
