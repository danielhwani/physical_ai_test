"""원본 PDU 기록 (문서 §11 원본 층). 형식은 DIS_test(siman_r/pdulog.py)와 같아 그쪽 재생기·모니터로 바로 읽힌다.

    JSONL 한 줄 = 한 패킷: t_rel, t_utc, dir("console->sim" / "sim->console"), src, dst, len, 해석 층(헤더, Entity ID,
    Request ID, 페이로드), raw(PDU 바이트 hex). 인증에 실패해 버린 패킷은 "dropped": "auth"와 이유를 남긴다.
    pcap: 이더넷 없이 IPv4/UDP 헤더만 붙인다 (LINKTYPE_IPV4). Wireshark가 UDP 3000번을 DIS로 해석한다.

    재생: python3 -m siman_r.player <run>/dis_pdus.jsonl --print            (~/DIS_test에서)
"""
import json
import socket
import struct
import time
from datetime import datetime, timezone

from . import envelope as E

PDU_TYPE_NAMES = {1: "Entity State", E.PDU_DATA: "Data", E.PDU_ACTION_REQUEST_R: "Action Request-R",
                  E.PDU_ACTION_RESPONSE_R: "Action Response-R"}
LINKTYPE_IPV4 = 228


def summarize(data):
    """해석 층 필드. 디코딩에 실패해도 원본은 남기므로 예외를 던지지 않는다."""
    try:
        h = E.peek_header(data)
    except E.DecodeError as e:
        return {"decode_error": str(e)}
    out = {"exercise_id": h.exercise_id, "pdu_type": h.pdu_type,
           "pdu_name": PDU_TYPE_NAMES.get(h.pdu_type, f"Type {h.pdu_type}"), "family": h.family, "timestamp": h.timestamp}
    try:
        pdu = E.decode(data)
    except E.DecodeError as e:
        return {**out, "decode_error": str(e)}
    if pdu is None:
        return out
    out.update(originating=str(pdu.originating), receiving=str(pdu.receiving), request_id=pdu.request_id,
               payload_type=pdu.payload.type, payload_body=pdu.payload.body)
    if isinstance(pdu, E.ActionResponseR):
        out["request_status"] = pdu.request_status
    return out


def _ip_checksum(hdr):
    s = sum(struct.unpack(f">{len(hdr) // 2}H", hdr))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF


class PcapWriter:
    def __init__(self, f):
        self.f, self.ip_id = f, 0
        f.write(struct.pack("<IHHiIII", 0xA1B2C3D4, 2, 4, 0, 0, 65535, LINKTYPE_IPV4))

    def write(self, t_epoch, src, dst, payload):
        udp = struct.pack(">HHHH", src[1], dst[1], 8 + len(payload), 0) + payload     # IPv4 UDP 체크섬 0 = 미사용
        self.ip_id = (self.ip_id + 1) & 0xFFFF
        ip = struct.pack(">BBHHHBBH4s4s", 0x45, 0, 20 + len(udp), self.ip_id, 0, 64, socket.IPPROTO_UDP, 0,
                         socket.inet_aton(src[0]), socket.inet_aton(dst[0]))
        ip = ip[:10] + struct.pack(">H", _ip_checksum(ip)) + ip[12:]
        sec = int(t_epoch)
        self.f.write(struct.pack("<IIII", sec, int((t_epoch - sec) * 1e6), len(ip + udp), len(ip + udp)) + ip + udp)
        self.f.flush()


class PduRecorder:
    """시뮬레이터가 받고 보낸 PDU를 그대로 기록한다 (jsonl + pcap)."""

    def __init__(self, out_dir, local_addr):
        self.local, self.t0 = local_addr, time.monotonic()
        self.jsonl = open(out_dir / "dis_pdus.jsonl", "w", encoding="utf-8")
        self.pcap_f = open(out_dir / "dis_pdus.pcap", "wb")
        self.pcap = PcapWriter(self.pcap_f)

    def record(self, rx, peer, data, **extra):
        src, dst = (peer, self.local) if rx else (self.local, peer)
        now = time.time()
        rec = {"t_rel": round(time.monotonic() - self.t0, 6),
               "t_utc": datetime.fromtimestamp(now, timezone.utc).isoformat(timespec="microseconds"),
               "dir": "console->sim" if rx else "sim->console", "src": f"{src[0]}:{src[1]}", "dst": f"{dst[0]}:{dst[1]}",
               "len": len(data), **summarize(data), **extra, "raw": data.hex()}
        self.jsonl.write(json.dumps(rec, ensure_ascii=False) + "\n")
        self.jsonl.flush()
        if not extra.get("dropped"):
            self.pcap.write(now, src, dst, data)

    def close(self):
        self.jsonl.close()
        self.pcap_f.close()
