#!/usr/bin/env bash
set -euo pipefail

CONFIG_BASE="${XDG_CONFIG_HOME:-$HOME/.config}/autonomic_turtlebot3"
ENV_FILE="$CONFIG_BASE/runtime.env"
mkdir -p "$CONFIG_BASE"

if [[ -e "$ENV_FILE" ]]; then
  echo "Refusing to overwrite existing secret file: $ENV_FILE" >&2
  exit 1
fi

umask 077
MASTER_KEY="$(openssl rand -hex 32)"
ROBOT_PASSWORD="$(openssl rand -base64 24 | tr -d '\n')"
DASHBOARD_PASSWORD="$(openssl rand -base64 24 | tr -d '\n')"
AUX_PASSWORD="$(openssl rand -base64 24 | tr -d '\n')"

printf '%s\n' \
  "AUTONOMIC_MASTER_KEY_HEX=$MASTER_KEY" \
  "AUTONOMIC_MQTT_PASSWORD=$ROBOT_PASSWORD" \
  "AUTONOMIC_MQTT_DASHBOARD_PASSWORD=$DASHBOARD_PASSWORD" \
  "AUTONOMIC_MQTT_AUX_PASSWORD=$AUX_PASSWORD" \
  > "$ENV_FILE"
chmod 600 "$ENV_FILE"

echo "Created protected runtime environment: $ENV_FILE"
echo "Use the same AES master key on the host dashboard and ROS MQTT bridge."
