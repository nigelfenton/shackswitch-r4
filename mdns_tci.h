/*
 * mdns_tci.h
 * Minimal mDNS / DNS-SD advertiser for the _tci._tcp.local schema defined
 * in https://github.com/ten9876/AetherSDR/blob/master/docs/tci-discovery.md
 *
 * Scope
 *   - Advertises ONE service instance under _tci._tcp.local.
 *   - Periodic announce + reply-to-query for PTR lookups of the service type.
 *   - Builds SRV / TXT / A records as Additional records in PTR replies so
 *     a single round-trip is enough for the AetherSDR Peripherals browse.
 *
 * Out of scope
 *   - Probing / conflict detection (assumes hostname unique on the LAN).
 *   - Goodbye packets on stop().
 *   - Direct queries for the instance's SRV/TXT/A by name (the AetherSDR
 *     browse asks the PTR question first, so it gets back the SRV/TXT/A as
 *     additional records in the same reply, which is what it needs).
 *
 * Platform
 *   - Tested against Arduino Giga R1 WiFi (WiFi.h, mbed core).
 *   - Should port cleanly to any Arduino WiFi stack that gives you a
 *     WiFiUDP that supports beginMulticast() + multicast TX (R4 WiFi /
 *     WiFiS3, Uno Q WiFi, etc).
 */
#pragma once

#include <Arduino.h>
#if defined(ARDUINO_UNOR4_WIFI)
  #include <WiFiS3.h>
#else
  #include <WiFi.h>
#endif
#include <WiFiUdp.h>

class TciMdnsAdvertiser {
public:
    // begin() must be called AFTER WiFi is fully associated (localIP() valid).
    //
    //   model        - lowercase, hyphen-separated stable identifier
    //                  (e.g. "aether-pad", "shackswitch-v2")
    //   devClass     - one of: controller, antenna-switch, rotator,
    //                  amplifier, tuner, other
    //   tciVersion   - "MAJOR.MINOR" of the highest TCI version spoken
    //   instanceName - user-visible name shown in AetherSDR's list
    //                  (free-form UTF-8, <= 63 bytes after encoding)
    //   hostnameNoLocal - DNS A-record hostname WITHOUT the .local suffix;
    //                     must be unique on the LAN
    //   tciPort      - peripheral's TCI listening port; pass 0 if this
    //                  peripheral is a TCI client only (no inbound server).
    //                  AetherSDR's browse code may surface port=0 entries
    //                  as informational-only.
    bool begin(const char* model,
               const char* devClass,
               const char* tciVersion,
               const char* instanceName,
               const char* hostnameNoLocal,
               uint16_t    tciPort);

    // Call from loop(). Cheap when idle (one socket parsePacket() check).
    void loop();

    // Stop announcing and close the socket. Does NOT send a goodbye packet.
    void stop();

    // Diagnostics
    bool   running() const { return _started; }
    size_t announceCount() const { return _announcesSent; }
    size_t responseCount() const { return _responsesSent; }

private:
    void onIncomingPacket();
    void sendUnsolicitedAnnounce();
    void sendPtrReply(IPAddress dst, uint16_t dstPort, uint16_t qid, bool unicast);

    size_t buildPtrReply(uint8_t* buf, size_t cap, uint16_t qid);
    static size_t writeHeader(uint8_t* buf, uint16_t qid,
                              uint16_t ancount, uint16_t arcount);
    static size_t writeServiceName(uint8_t* buf, size_t off);   // _tci._tcp.local
    static size_t writeLabel(uint8_t* buf, size_t off, const char* s);
    static size_t writePointer(uint8_t* buf, size_t off, uint16_t target);
    static size_t writeU16(uint8_t* buf, size_t off, uint16_t v);
    static size_t writeU32(uint8_t* buf, size_t off, uint32_t v);

    WiFiUDP _udp;
    String  _model, _class, _tciVersion, _instance, _host;
    uint16_t _port = 0;
    IPAddress _ip;

    bool          _started = false;
    unsigned long _nextAnnounceMs = 0;
    uint8_t       _startupAnnouncesLeft = 0;
    size_t        _announcesSent = 0;
    size_t        _responsesSent = 0;
};
