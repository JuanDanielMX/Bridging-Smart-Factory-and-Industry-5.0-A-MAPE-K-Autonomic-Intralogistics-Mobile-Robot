"""Authenticated AES-256-CBC envelopes for selected MQTT payloads.

AES-CBC alone does not authenticate data. The paper's confidentiality mechanism is
retained and strengthened with encrypt-then-HMAC, replay age checks, and random IVs.
"""

import base64
import hashlib
import hmac
import json
import os
import time
from typing import Any, Dict, Optional, Set

from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import hashes, padding
from cryptography.hazmat.primitives.ciphers import Cipher, algorithms, modes
from cryptography.hazmat.primitives.kdf.hkdf import HKDF


class SecurePayloadError(ValueError):
    pass


def key_from_hex(value: str) -> bytes:
    try:
        key = bytes.fromhex(value.strip())
    except ValueError as exc:
        raise SecurePayloadError("AUTONOMIC_MASTER_KEY_HEX is not valid hexadecimal") from exc
    if len(key) != 32:
        raise SecurePayloadError("AUTONOMIC_MASTER_KEY_HEX must contain exactly 32 bytes (64 hex chars)")
    return key


class SecurePayload:
    VERSION = 1
    ALGORITHM = "AES-256-CBC+HMAC-SHA256"

    def __init__(self, master_key: bytes, max_age_seconds: int = 120) -> None:
        if len(master_key) != 32:
            raise SecurePayloadError("master key must be 32 bytes")
        self.max_age_seconds = int(max_age_seconds)
        self.encryption_key = self._derive(master_key, b"autonomic-tb3/aes")
        self.authentication_key = self._derive(master_key, b"autonomic-tb3/hmac")
        self._seen_nonces: Set[str] = set()

    @staticmethod
    def _derive(master_key: bytes, info: bytes) -> bytes:
        return HKDF(
            algorithm=hashes.SHA256(),
            length=32,
            salt=None,
            info=info,
            backend=default_backend(),
        ).derive(master_key)

    @staticmethod
    def _b64(value: bytes) -> str:
        return base64.b64encode(value).decode("ascii")

    @staticmethod
    def _unb64(value: str) -> bytes:
        try:
            return base64.b64decode(value.encode("ascii"), validate=True)
        except Exception as exc:
            raise SecurePayloadError("invalid base64 in secure envelope") from exc

    @staticmethod
    def _authenticated_bytes(version: int, timestamp: int, nonce: str, iv: str, ciphertext: str) -> bytes:
        return f"{version}|{timestamp}|{nonce}|{iv}|{ciphertext}".encode("utf-8")

    def encrypt(self, value: Dict[str, Any], timestamp: Optional[int] = None) -> str:
        plaintext = json.dumps(value, separators=(",", ":"), sort_keys=True).encode("utf-8")
        padder = padding.PKCS7(128).padder()
        padded = padder.update(plaintext) + padder.finalize()
        iv = os.urandom(16)
        encryptor = Cipher(
            algorithms.AES(self.encryption_key),
            modes.CBC(iv),
            backend=default_backend(),
        ).encryptor()
        ciphertext = encryptor.update(padded) + encryptor.finalize()
        ts = int(time.time() if timestamp is None else timestamp)
        nonce = self._b64(os.urandom(12))
        iv_b64 = self._b64(iv)
        ciphertext_b64 = self._b64(ciphertext)
        authenticated = self._authenticated_bytes(self.VERSION, ts, nonce, iv_b64, ciphertext_b64)
        tag = hmac.new(self.authentication_key, authenticated, hashlib.sha256).digest()
        return json.dumps(
            {
                "alg": self.ALGORITHM,
                "ct": ciphertext_b64,
                "iv": iv_b64,
                "nonce": nonce,
                "tag": self._b64(tag),
                "ts": ts,
                "v": self.VERSION,
            },
            separators=(",", ":"),
            sort_keys=True,
        )

    def decrypt(self, envelope: str, now: Optional[int] = None) -> Dict[str, Any]:
        try:
            packet = json.loads(envelope)
            version = int(packet["v"])
            timestamp = int(packet["ts"])
            nonce = str(packet["nonce"])
            iv_b64 = str(packet["iv"])
            ciphertext_b64 = str(packet["ct"])
            tag = self._unb64(str(packet["tag"]))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            raise SecurePayloadError("malformed secure envelope") from exc

        if version != self.VERSION or packet.get("alg") != self.ALGORITHM:
            raise SecurePayloadError("unsupported secure envelope version or algorithm")
        current_time = int(time.time() if now is None else now)
        if self.max_age_seconds > 0 and abs(current_time - timestamp) > self.max_age_seconds:
            raise SecurePayloadError("secure envelope is outside the accepted time window")
        if nonce in self._seen_nonces:
            raise SecurePayloadError("replayed secure envelope")

        authenticated = self._authenticated_bytes(version, timestamp, nonce, iv_b64, ciphertext_b64)
        expected_tag = hmac.new(self.authentication_key, authenticated, hashlib.sha256).digest()
        if not hmac.compare_digest(tag, expected_tag):
            raise SecurePayloadError("secure envelope authentication failed")

        iv = self._unb64(iv_b64)
        ciphertext = self._unb64(ciphertext_b64)
        if len(iv) != 16 or not ciphertext or len(ciphertext) % 16:
            raise SecurePayloadError("invalid AES-CBC IV or ciphertext length")

        try:
            decryptor = Cipher(
                algorithms.AES(self.encryption_key),
                modes.CBC(iv),
                backend=default_backend(),
            ).decryptor()
            padded = decryptor.update(ciphertext) + decryptor.finalize()
            unpadder = padding.PKCS7(128).unpadder()
            plaintext = unpadder.update(padded) + unpadder.finalize()
            value = json.loads(plaintext.decode("utf-8"))
        except Exception as exc:
            raise SecurePayloadError("secure payload decryption failed") from exc
        if not isinstance(value, dict):
            raise SecurePayloadError("secure payload must decode to a JSON object")

        self._seen_nonces.add(nonce)
        if len(self._seen_nonces) > 2048:
            self._seen_nonces.clear()
            self._seen_nonces.add(nonce)
        return value
