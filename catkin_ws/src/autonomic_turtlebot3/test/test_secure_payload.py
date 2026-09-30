#!/usr/bin/env python3

import json
import unittest

try:
    from autonomic_turtlebot3.secure_payload import SecurePayload, SecurePayloadError
except ImportError:
    SecurePayload = None
    SecurePayloadError = ValueError


@unittest.skipIf(SecurePayload is None, "python3-cryptography is not installed")
class SecurePayloadTest(unittest.TestCase):
    def setUp(self):
        self.codec = SecurePayload(bytes(range(32)), max_age_seconds=120)

    def test_round_trip(self):
        envelope = self.codec.encrypt({"queued_orders": 4}, timestamp=1000)
        self.assertEqual(self.codec.decrypt(envelope, now=1000), {"queued_orders": 4})

    def test_tampering_is_rejected(self):
        envelope = json.loads(self.codec.encrypt({"state": "RUNNING"}, timestamp=1000))
        envelope["ct"] = ("A" if envelope["ct"][0] != "A" else "B") + envelope["ct"][1:]
        with self.assertRaises(SecurePayloadError):
            self.codec.decrypt(json.dumps(envelope), now=1000)

    def test_replay_is_rejected(self):
        envelope = self.codec.encrypt({"state": "RUNNING"}, timestamp=1000)
        self.codec.decrypt(envelope, now=1000)
        with self.assertRaises(SecurePayloadError):
            self.codec.decrypt(envelope, now=1000)


if __name__ == "__main__":
    unittest.main()
