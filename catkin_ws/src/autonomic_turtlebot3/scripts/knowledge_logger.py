#!/usr/bin/env python3
"""Persistent MAPE-K knowledge store backed by SQLite."""

import json
import os
import sqlite3
import threading
from typing import Any

import rospy
from rospy.timer import TimerEvent
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, Int32, String


class KnowledgeLogger:
    def __init__(self) -> None:
        database_path = os.path.abspath(os.path.expanduser(
            rospy.get_param("~database_path", "~/.ros/autonomic_turtlebot3/knowledge.db")
        ))
        os.makedirs(os.path.dirname(database_path), exist_ok=True)
        self._lock = threading.Lock()
        self._connection = sqlite3.connect(database_path, check_same_thread=False)
        self._connection.execute("PRAGMA journal_mode=WAL")
        self._connection.execute(
            """
            CREATE TABLE IF NOT EXISTS events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                ros_time REAL NOT NULL,
                category TEXT NOT NULL,
                payload TEXT NOT NULL
            )
            """
        )
        self._connection.execute(
            "CREATE INDEX IF NOT EXISTS idx_events_time ON events(ros_time)"
        )
        self._connection.commit()
        rospy.loginfo("MAPE-K knowledge database: %s", database_path)

        rospy.Subscriber("/autonomic/mapek_event", String, self._string_event("mapek"), queue_size=200)
        rospy.Subscriber("/autonomic/state", String, self._string_event("state"), queue_size=20)
        rospy.Subscriber("/autonomic/alert", String, self._string_event("alert"), queue_size=20)
        rospy.Subscriber("/factory/queued_orders", Int32, self._value_event("queued_orders"), queue_size=20)
        rospy.Subscriber("/autonomic/target_speed", Float32, self._value_event("target_speed"), queue_size=20)
        rospy.Subscriber("/battery_state", BatteryState, self._battery_event, queue_size=20)
        self._prune_timer = rospy.Timer(rospy.Duration(3600.0), self._prune)
        rospy.on_shutdown(self.shutdown)

    def _insert(self, category: str, payload: Any) -> None:
        encoded = payload if isinstance(payload, str) else json.dumps(payload, separators=(",", ":"))
        with self._lock:
            self._connection.execute(
                "INSERT INTO events(ros_time, category, payload) VALUES (?, ?, ?)",
                (rospy.Time.now().to_sec(), category, encoded),
            )
            self._connection.commit()

    def _string_event(self, category: str):
        def callback(message: String) -> None:
            self._insert(category, message.data)
        return callback

    def _value_event(self, category: str):
        def callback(message: Any) -> None:
            self._insert(category, {"value": message.data})
        return callback

    def _battery_event(self, message: BatteryState) -> None:
        self._insert(
            "battery",
            {
                "voltage": message.voltage,
                "current": message.current,
                "percentage": message.percentage,
            },
        )

    def _prune(self, _event: TimerEvent) -> None:
        retention_days = max(1, int(rospy.get_param("~retention_days", 90)))
        cutoff = rospy.Time.now().to_sec() - retention_days * 86400
        with self._lock:
            self._connection.execute("DELETE FROM events WHERE ros_time < ?", (cutoff,))
            self._connection.commit()

    def shutdown(self) -> None:
        if hasattr(self, "_connection"):
            with self._lock:
                self._connection.commit()
                self._connection.close()


def main() -> None:
    rospy.init_node("knowledge_logger")
    KnowledgeLogger()
    rospy.spin()


if __name__ == "__main__":
    main()
