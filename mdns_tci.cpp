/*
 * mdns_tci.cpp — see mdns_tci.h for usage.
 *
 * Packet layout reference (RFC 6762 / 6763):
 *   Response header: 12 bytes (ID, flags=0x8400, QDCOUNT=0, ANCOUNT, NSCOUNT=0, ARCOUNT)
 *   Each RR: NAME, TYPE(2), CLASS(2), TTL(4), RDLENGTH(2), RDATA(...)
 *
 * The PTR reply we send has 1 answer (PTR) and 3 additionals (SRV, TXT, A).
 * Name compression: the first occurrence of "_tci._tcp.local" lands at
 * offset 12; we keep the offsets of "<instance>._tci._tcp.local" and
 * "<host>.local" so later records can point at them with 0xC0XX.
 */
#include "mdns_tci.h"

namespace {
    const IPAddress kMdnsGroup(224, 0, 0, 251);
    const uint16_t  kMdnsPort = 5353;

    const uint32_t  kTtlLong  = 4500;   // PTR / TXT
    const uint32_t  kTtlShort = 120;    // SRV / A (RFC 6762 §10)

    const uint16_t  kFlagsResponse = 0x8400;  // QR=1, AA=1
    const uint16_t  kClassIN       = 0x0001;
    const uint16_t  kClassINFlush  = 0x8001;  // IN + cache-flush bit

    const uint16_t  kTypePTR = 0x000C;
    const uint16_t  kTypeSRV = 0x0021;
    const uint16_t  kTypeTXT = 0x0010;
    const uint16_t  kTypeA   = 0x0001;
    const uint16_t  kTypeANY = 0x00FF;

    // Bytes for "_tci._tcp.local" — used as a fast-path match in incoming
    // queries. Case-sensitive; we rely on standard lowercase DNS-SD usage.
    const uint8_t kServiceNameBytes[] = {
        0x04, '_','t','c','i',
        0x04, '_','t','c','p',
        0x05, 'l','o','c','a','l',
        0x00
    };
    const size_t kServiceNameLen = sizeof(kServiceNameBytes); // 17

    // Tx buffer — sized for a single PTR-with-additionals reply.
    // Bound: header(12) + ptr_rr(~30) + srv_rr(~30 + host) + txt_rr(~80) + a_rr(~20)
    // Easily fits in 256.  Bumped to 320 to allow longer instance names.
    constexpr size_t kTxBufSize = 320;
}

bool TciMdnsAdvertiser::begin(const char* model,
                              const char* devClass,
                              const char* tciVersion,
                              const char* instanceName,
                              const char* hostnameNoLocal,
                              uint16_t    tciPort)
{
    if (_started) return true;

    _model      = model      ? model      : "unknown";
    _class      = devClass   ? devClass   : "other";
    _tciVersion = tciVersion ? tciVersion : "1.0";
    _instance   = instanceName  ? instanceName  : "TCI peripheral";
    _host       = hostnameNoLocal ? hostnameNoLocal : "tci-peripheral";
    _port       = tciPort;
    _ip         = WiFi.localIP();

    // Bind to the mDNS multicast group on 5353.  WiFi.h on Giga R1 exposes
    // beginMulticast(); on platforms where it doesn't, fall back to begin().
#if defined(ARDUINO_GIGA) || defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_OPTA)
    int ok = _udp.beginMulticast(kMdnsGroup, kMdnsPort);
#else
    int ok = _udp.beginMulticast(kMdnsGroup, kMdnsPort);
#endif
    if (!ok) {
        Serial.println("mDNS: beginMulticast failed");
        return false;
    }

    _started              = true;
    _startupAnnouncesLeft = 3;          // RFC 6762 §8.3 — burst of 2-3 announces
    _nextAnnounceMs       = millis() + 100;
    Serial.print("mDNS: advertising "); Serial.print(_instance);
    Serial.print(" ("); Serial.print(_model); Serial.print(", class=");
    Serial.print(_class); Serial.print(", host="); Serial.print(_host);
    Serial.print(".local, port="); Serial.print(_port);
    Serial.println(")");
    return true;
}

void TciMdnsAdvertiser::stop() {
    if (!_started) return;
    _udp.stop();
    _started = false;
}

void TciMdnsAdvertiser::loop() {
    if (!_started) return;

    onIncomingPacket();

    unsigned long now = millis();
    if ((long)(now - _nextAnnounceMs) >= 0) {
        sendUnsolicitedAnnounce();
        if (_startupAnnouncesLeft > 0) {
            _startupAnnouncesLeft--;
            _nextAnnounceMs = now + 1000;       // 1 s between startup announces
        } else {
            _nextAnnounceMs = now + 60UL * 1000UL;  // 60 s steady-state
        }
    }
}

// ---------------------------------------------------------------------------
// Incoming
// ---------------------------------------------------------------------------
void TciMdnsAdvertiser::onIncomingPacket() {
    int pktLen = _udp.parsePacket();
    if (pktLen < 13) {
        if (pktLen > 0) {
            // Drain so we don't loop on the same packet.
            uint8_t scratch[64];
            _udp.read(scratch, sizeof(scratch));
        }
        return;
    }

    uint8_t buf[256];
    int n = _udp.read(buf, sizeof(buf));
    if (n < 12) return;

    // Skip if this is itself a response.  QR bit = bit 15 of flags (buf[2]).
    uint16_t flags = ((uint16_t)buf[2] << 8) | buf[3];
    if (flags & 0x8000) return;

    uint16_t qid     = ((uint16_t)buf[0] << 8) | buf[1];
    uint16_t qdcount = ((uint16_t)buf[4] << 8) | buf[5];
    if (qdcount == 0) return;

    // Walk questions.  We only need to find ONE that asks for
    // _tci._tcp.local with type PTR or ANY.
    size_t off = 12;
    for (uint16_t q = 0; q < qdcount && off + kServiceNameLen + 4 <= (size_t)n; ++q) {
        // Match the service name byte-for-byte.  We don't follow compression
        // pointers in the question section because mDNS queries from real
        // browsers always write the full name uncompressed.
        bool match = true;
        for (size_t i = 0; i < kServiceNameLen; ++i) {
            if (buf[off + i] != kServiceNameBytes[i]) { match = false; break; }
        }
        if (!match) {
            // Skip this question: advance over its name, then QTYPE+QCLASS.
            while (off < (size_t)n && buf[off] != 0x00) {
                uint8_t b = buf[off];
                if ((b & 0xC0) == 0xC0) { off += 2; goto fields; }   // pointer
                if (b > 63) return;                                  // malformed
                off += 1 + b;
            }
            off += 1;     // null terminator
        fields:
            off += 4;     // QTYPE(2) + QCLASS(2)
            continue;
        }

        size_t fieldOff = off + kServiceNameLen;
        if (fieldOff + 4 > (size_t)n) return;
        uint16_t qtype  = ((uint16_t)buf[fieldOff] << 8)     | buf[fieldOff + 1];
        uint16_t qclass = ((uint16_t)buf[fieldOff + 2] << 8) | buf[fieldOff + 3];
        bool wantsUnicast = (qclass & 0x8000) != 0;
        (void)qclass;

        if (qtype == kTypePTR || qtype == kTypeANY) {
            sendPtrReply(_udp.remoteIP(), _udp.remotePort(), qid, wantsUnicast);
            return;
        }
        off = fieldOff + 4;
    }
}

// ---------------------------------------------------------------------------
// Outgoing
// ---------------------------------------------------------------------------
void TciMdnsAdvertiser::sendUnsolicitedAnnounce() {
    uint8_t buf[kTxBufSize];
    size_t len = buildPtrReply(buf, sizeof(buf), 0);
    if (len == 0) return;
    if (_udp.beginPacket(kMdnsGroup, kMdnsPort)) {
        _udp.write(buf, len);
        _udp.endPacket();
        _announcesSent++;
    }
}

void TciMdnsAdvertiser::sendPtrReply(IPAddress dst, uint16_t dstPort,
                                     uint16_t qid, bool unicast)
{
    uint8_t buf[kTxBufSize];
    size_t len = buildPtrReply(buf, sizeof(buf), qid);
    if (len == 0) return;

    // mDNS-over-multicast is the safe default.  If the querier set the QU
    // bit (preference for unicast), we reply unicast to its source addr.
    IPAddress target = unicast ? dst         : kMdnsGroup;
    uint16_t  port   = unicast ? dstPort     : kMdnsPort;
    if (_udp.beginPacket(target, port)) {
        _udp.write(buf, len);
        _udp.endPacket();
        _responsesSent++;
    }
}

// ---------------------------------------------------------------------------
// Packet builders
// ---------------------------------------------------------------------------
size_t TciMdnsAdvertiser::buildPtrReply(uint8_t* buf, size_t cap, uint16_t qid) {
    if (cap < 64) return 0;
    size_t o = 0;

    // Header — 1 answer (PTR), 3 additional (SRV, TXT, A)
    o = writeHeader(buf, qid, /*ancount=*/1, /*arcount=*/3);

    // ---- Answer: PTR _tci._tcp.local -> <instance>._tci._tcp.local --------
    // Name = _tci._tcp.local
    size_t serviceNameOff = o;
    o = writeServiceName(buf, o);
    o = writeU16(buf, o, kTypePTR);
    o = writeU16(buf, o, kClassIN);       // PTR is shared, no cache-flush
    o = writeU32(buf, o, kTtlLong);

    // RDLENGTH placeholder
    size_t rdlenPos = o; o += 2;
    size_t rdStart  = o;

    // RDATA: <instance> label + pointer to "_tci._tcp.local"
    size_t instanceNameOff = o;           // remembered for SRV/TXT compression
    o = writeLabel(buf, o, _instance.c_str());
    o = writePointer(buf, o, serviceNameOff);

    uint16_t rdlen = (uint16_t)(o - rdStart);
    buf[rdlenPos]     = (uint8_t)(rdlen >> 8);
    buf[rdlenPos + 1] = (uint8_t)(rdlen & 0xFF);

    // ---- Additional 1: SRV <instance>._tci._tcp.local --------------------
    o = writePointer(buf, o, instanceNameOff);
    o = writeU16(buf, o, kTypeSRV);
    o = writeU16(buf, o, kClassINFlush);
    o = writeU32(buf, o, kTtlShort);
    size_t srvLenPos = o; o += 2;
    size_t srvStart  = o;
    o = writeU16(buf, o, 0);              // priority
    o = writeU16(buf, o, 0);              // weight
    o = writeU16(buf, o, _port);          // port
    // Target = <host>.local
    size_t hostNameOff = o;
    o = writeLabel(buf, o, _host.c_str());
    // Point to the "local" label inside the service name at serviceNameOff
    // (which is at serviceNameOff + 10 — 5 bytes "_tci" labelled + 5 bytes "_tcp" labelled).
    // For safety we recompute: _tci label = 5 bytes, _tcp label = 5 bytes,
    // so "local" starts at serviceNameOff + 10.
    o = writePointer(buf, o, serviceNameOff + 10);
    uint16_t srvLen = (uint16_t)(o - srvStart);
    buf[srvLenPos]     = (uint8_t)(srvLen >> 8);
    buf[srvLenPos + 1] = (uint8_t)(srvLen & 0xFF);

    // ---- Additional 2: TXT <instance>._tci._tcp.local --------------------
    o = writePointer(buf, o, instanceNameOff);
    o = writeU16(buf, o, kTypeTXT);
    o = writeU16(buf, o, kClassINFlush);
    o = writeU32(buf, o, kTtlLong);
    size_t txtLenPos = o; o += 2;
    size_t txtStart  = o;

    auto writeTxtEntry = [&](const String& kv) {
        size_t len = kv.length();
        if (len > 255) len = 255;
        if (o + 1 + len >= cap) return;
        buf[o++] = (uint8_t)len;
        memcpy(buf + o, kv.c_str(), len);
        o += len;
    };
    writeTxtEntry(String("txtvers=1"));
    writeTxtEntry(String("model=")       + _model);
    writeTxtEntry(String("class=")       + _class);
    writeTxtEntry(String("tci-version=") + _tciVersion);

    uint16_t txtLen = (uint16_t)(o - txtStart);
    buf[txtLenPos]     = (uint8_t)(txtLen >> 8);
    buf[txtLenPos + 1] = (uint8_t)(txtLen & 0xFF);

    // ---- Additional 3: A <host>.local -> our IPv4 -------------------------
    o = writePointer(buf, o, hostNameOff);
    o = writeU16(buf, o, kTypeA);
    o = writeU16(buf, o, kClassINFlush);
    o = writeU32(buf, o, kTtlShort);
    o = writeU16(buf, o, 4);              // RDLENGTH
    buf[o++] = _ip[0];
    buf[o++] = _ip[1];
    buf[o++] = _ip[2];
    buf[o++] = _ip[3];

    return o;
}

// ---------------------------------------------------------------------------
// Primitives
// ---------------------------------------------------------------------------
size_t TciMdnsAdvertiser::writeHeader(uint8_t* buf, uint16_t qid,
                                      uint16_t ancount, uint16_t arcount)
{
    buf[0] = (uint8_t)(qid >> 8);
    buf[1] = (uint8_t)(qid & 0xFF);
    buf[2] = (uint8_t)(kFlagsResponse >> 8);
    buf[3] = (uint8_t)(kFlagsResponse & 0xFF);
    buf[4] = 0; buf[5] = 0;               // QDCOUNT
    buf[6] = (uint8_t)(ancount >> 8); buf[7] = (uint8_t)(ancount & 0xFF);
    buf[8] = 0; buf[9] = 0;               // NSCOUNT
    buf[10] = (uint8_t)(arcount >> 8); buf[11] = (uint8_t)(arcount & 0xFF);
    return 12;
}

size_t TciMdnsAdvertiser::writeServiceName(uint8_t* buf, size_t off) {
    memcpy(buf + off, kServiceNameBytes, kServiceNameLen);
    return off + kServiceNameLen;
}

size_t TciMdnsAdvertiser::writeLabel(uint8_t* buf, size_t off, const char* s) {
    size_t len = strlen(s);
    if (len > 63) len = 63;
    buf[off++] = (uint8_t)len;
    memcpy(buf + off, s, len);
    return off + len;
}

size_t TciMdnsAdvertiser::writePointer(uint8_t* buf, size_t off, uint16_t target) {
    buf[off++] = 0xC0 | (uint8_t)((target >> 8) & 0x3F);
    buf[off++] = (uint8_t)(target & 0xFF);
    return off;
}

size_t TciMdnsAdvertiser::writeU16(uint8_t* buf, size_t off, uint16_t v) {
    buf[off++] = (uint8_t)(v >> 8);
    buf[off++] = (uint8_t)(v & 0xFF);
    return off;
}

size_t TciMdnsAdvertiser::writeU32(uint8_t* buf, size_t off, uint32_t v) {
    buf[off++] = (uint8_t)((v >> 24) & 0xFF);
    buf[off++] = (uint8_t)((v >> 16) & 0xFF);
    buf[off++] = (uint8_t)((v >>  8) & 0xFF);
    buf[off++] = (uint8_t)( v        & 0xFF);
    return off;
}
