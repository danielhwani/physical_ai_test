"""DIS PDU 메시지 인증 (HMAC-SHA256, 공유 키). 표준 라이브러리만 사용.

    python -m dis_console.auth keygen ~/.config/physics_ai_test/dis.key     # 키 만들기 (권한 600)

PDU 끝에 Variable Datum 하나(DATUM_AUTH)를 붙인다: 카운터 8 B + HMAC 32 B (= 320 bit, 64 bit 경계라 패딩 없음).
  MAC = HMAC-SHA256(키, 인증 datum을 붙이기 전 PDU 바이트 + 카운터)
헤더(Exercise ID, PDU 종류, 타임스탬프)와 페이로드가 모두 MAC에 들어가므로 한 바이트라도 바꾸면 거부된다
(예: DIS_test 재생기가 Exercise ID를 99로 바꿔 보낸 명령). 받는 쪽 코드는 인증 datum을 모르는 체계(DIS_test,
Wireshark)에서도 그대로 읽힌다: Variable Datum이 하나 더 있을 뿐이다.

재전송 공격 방지 (Verifier): 카운터 = 보낸 시각(마이크로초, 보낼 때마다 1 이상 증가).
  - 받은 시각과 MAX_SKEW_S 넘게 차이 나면 거부 (오래된 캡처를 다시 보냄)
  - 보낸 쪽(Entity ID)별로 지난 카운터보다 커야 받는다. 다만 최근에 받은 패킷과 바이트까지 같으면 받는다
    (응답이 없어 같은 요청을 그대로 다시 보내는 핸드셰이크 재전송. 그 사이 heartbeat가 더 큰 카운터로 지나갔어도)
다른 장비끼리 쓰려면 시계를 맞춰야 한다 (NTP/PTP, 문서 §9).
"""
import hashlib
import hmac
import os
import struct
import sys
import time
from pathlib import Path

from . import envelope as E

DATUM_AUTH = 500005           # 설명용 예시값 (인터페이스 규약에서 확정)
MAX_SKEW_S = 30.0
RECENT = 512
_DATUM_OFFSET = {E.PDU_DATA: 32, E.PDU_ACTION_REQUEST_R: 36, E.PDU_ACTION_RESPONSE_R: 32}   # Datum 레코드 시작
_AUTH_LEN = 8 + 8 + 32        # Datum ID + 길이(bit) + 카운터 + MAC


class AuthError(ValueError):
    pass


def load_key(path):
    key = bytes.fromhex(Path(path).expanduser().read_text().strip())
    if len(key) < 16:
        raise AuthError("키가 너무 짧다 (16 B 이상)")
    return key


def keygen(path):
    p = Path(path).expanduser()
    p.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(p, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(os.urandom(32).hex() + "\n")
    return p


def _counts_offset(raw):
    if len(raw) < 12:
        raise AuthError("헤더보다 짧다")
    off = _DATUM_OFFSET.get(raw[2])                   # 헤더 3번째 바이트 = PDU 종류 (길이 필드는 아직 맞지 않을 수 있다)
    if off is None or len(raw) < off + 8:
        raise AuthError(f"인증하지 않는 PDU 종류 {raw[2]}")
    return off


def _with_counts(raw, d_var, d_len):
    """Variable Datum 수와 헤더 길이를 바꾼 PDU 바이트."""
    off = _counts_offset(raw)
    n_fixed, n_var = struct.unpack_from(">II", raw, off)
    out = bytearray(raw)
    struct.pack_into(">I", out, off + 4, n_var + d_var)
    struct.pack_into(">H", out, 8, len(raw) + d_len)
    return bytes(out)


def _mac(key, raw_unsigned, counter):
    return hmac.new(key, raw_unsigned + struct.pack(">Q", counter), hashlib.sha256).digest()


class Signer:
    def __init__(self, key):
        self.key, self.counter = key, 0

    def sign(self, raw):
        self.counter = max(self.counter + 1, int(time.time() * 1e6))
        datum = struct.pack(">IIQ", DATUM_AUTH, (8 + 32) * 8, self.counter) + _mac(self.key, raw, self.counter)
        return _with_counts(raw + datum, +1, 0)


def split(raw):
    """서명된 PDU -> (서명 전 바이트, 카운터, MAC). 인증 datum이 없으면 AuthError."""
    if len(raw) < _AUTH_LEN or struct.unpack_from(">I", raw, len(raw) - _AUTH_LEN)[0] != DATUM_AUTH:
        raise AuthError("인증 datum 없음")
    did, nbits, counter = struct.unpack_from(">IIQ", raw, len(raw) - _AUTH_LEN)
    if nbits != (8 + 32) * 8:
        raise AuthError("인증 datum 길이가 다르다")
    body = raw[:-_AUTH_LEN]
    return _with_counts(body, -1, 0), counter, raw[-32:]


class Verifier:
    def __init__(self, key, max_skew_s=MAX_SKEW_S):
        self.key, self.max_skew = key, max_skew_s
        self.last = {}            # 보낸 쪽 -> 가장 큰 카운터
        self.recent = {}          # 보낸 쪽 -> {카운터: PDU 해시} (최근 RECENT개, 같은 바이트 재전송 허용)

    def verify(self, raw):
        """통과하면 서명 전 PDU 바이트를 돌려주고, 아니면 AuthError (이유)."""
        unsigned, counter, mac = split(raw)
        if not hmac.compare_digest(mac, _mac(self.key, unsigned, counter)):
            raise AuthError("MAC 불일치 (키가 다르거나 바뀐 패킷)")
        if abs(counter / 1e6 - time.time()) > self.max_skew:
            raise AuthError(f"카운터 시각이 {self.max_skew:g} s 넘게 어긋남 (오래된 패킷 재전송)")
        sender = bytes(raw[12:18])                    # 보낸 쪽 Entity ID (모든 SIMAN PDU에서 같은 자리)
        digest = hashlib.sha256(raw).digest()
        recent = self.recent.setdefault(sender, {})
        if counter <= self.last.get(sender, -1):
            if recent.get(counter) != digest:
                raise AuthError("카운터가 줄었다 (재전송 공격)")
            return unsigned                           # 같은 패킷을 그대로 다시 보냄 (핸드셰이크 재전송)
        self.last[sender] = counter
        recent[counter] = digest
        if len(recent) > RECENT:
            for c in sorted(recent)[:len(recent) - RECENT]:
                del recent[c]
        return unsigned


def main():
    if len(sys.argv) != 3 or sys.argv[1] != "keygen":
        sys.exit("사용: python -m dis_console.auth keygen <키 파일>")
    print(f"키를 만들었다: {keygen(sys.argv[2])} (권한 600). 시뮬레이터와 콘솔에 같은 파일을 준다")


if __name__ == "__main__":
    main()
