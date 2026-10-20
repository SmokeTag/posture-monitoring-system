/*
 * SerialOut — non-blocking line output for firmware/chair and firmware/body
 * --------------------------------------------------------------------------
 * Why: on this core (Adafruit TinyUSB, Seeeduino:nrf52 1.1.13) Serial.write()
 * loops `while (remain && connected) yield();` until the 256-byte CDC FIFO has
 * room. With a host that holds DTR but stops reading, any print() can stall
 * loop() forever — and the chair only ACKs the body from inside loop().
 *
 * How: a line is built in a fixed buffer (Line, no heap) and written whole, in
 * one write(), only if Serial.availableForWrite() (= free bytes in that FIFO)
 * can take all of it — so write() never waits and a line is never cut. Lines
 * that don't fit are dropped and counted; nothing waits for the host.
 *   - rows   (CSV): now or never; a dropped row is counted in rowDrops.
 *   - events ('#' lines): small FIFO queue (QN lines), sent before any later
 *     row; queue full -> the new event is dropped and counted in eventDrops.
 *   - header (banner, init failures, column names): re-sent from the start
 *     whenever a host opens the port (DTR rises); rows wait
 *     (and count as dropped) until it is out.
 *   - stats: sent like a row but not counted when it can't go; the caller keeps
 *     accumulating and retries (see stats()).
 * No host (DTR low / USB unplugged / battery): everything is discarded without
 * counting — no reader means nothing is lost to a log. Counts restart when a
 * host connects.
 *
 * Port: anything with dtr(), availableForWrite(), write(const uint8_t*, size_t)
 * (Adafruit_USBD_CDC here; a fake in firmware/test).
 */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "FloatBits.h"

namespace posture {

// One output line, built in place. Too long -> marked bad, never truncated.
class Line {
 public:
  static const uint8_t CAP = 176;              // content bytes; includes maximum v3 stats counters

  Line() { clear(); }
  void clear() { len_ = 0; bad_ = false; }
  bool ok() const { return !bad_; }
  uint8_t length() const { return len_; }
  const char *text() const { buf_[len_] = '\0'; return buf_; }

  Line &chr(char c) { if (len_ < CAP) buf_[len_++] = c; else bad_ = true; return *this; }
  Line &str(const char *s) { while (*s) chr(*s++); return *this; }
  Line &u32(uint32_t v) { return u64(v); }
  Line &i32(int32_t v) {
    if (v < 0) { chr('-'); return u64((uint64_t)(-(int64_t)v)); }
    return u64((uint64_t)v);
  }
  Line &hex8(uint8_t v) {
    const char *h = "0123456789abcdef";
    return chr(h[v >> 4]).chr(h[v & 15]);
  }

  // Same text as Print::print(float, digits) on this core (snprintf "%.*f",
  // round half to even on the exact value), without printf: digits <= 4.
  Line &fix(float v, uint8_t digits) {
    if (floatIsNan(v)) return str("nan");
    bool neg = (floatBits(v) >> 31) != 0;
    if (!floatIsFinite(v)) return str(neg ? "-inf" : "inf");
    double x = neg ? -(double)v : (double)v;
    if (x > 1e15) return str(neg ? "-inf" : "inf");   // inf, or far beyond any value here
    static const uint32_t P10[] = {1, 10, 100, 1000, 10000};
    if (digits > 4) digits = 4;
    double s = x * P10[digits];                // exact: 24-bit mantissa * <= 14 bits
    uint64_t n = (uint64_t)s;
    double fr = s - (double)n;
    if (fr > 0.5 || (fr == 0.5 && (n & 1))) n++;
    if (neg) chr('-');
    u64(n / P10[digits]);
    if (digits) {
      chr('.');
      uint32_t f = (uint32_t)(n % P10[digits]);
      for (uint32_t p = P10[digits] / 10; p; p /= 10) chr('0' + (char)(f / p % 10));
    }
    return *this;
  }

  // Bytes to write: the content + "\r\n" (println's line end).
  const uint8_t *bytes() const { buf_[len_] = '\r'; buf_[len_ + 1] = '\n'; return (const uint8_t *)buf_; }
  uint8_t size() const { return len_ + 2; }

 private:
  Line &u64(uint64_t v) {
    char d[20];
    int n = 0;
    do { d[n++] = '0' + (char)(v % 10); v /= 10; } while (v);
    while (n) chr(d[--n]);
    return *this;
  }

  mutable char buf_[CAP + 2];
  uint8_t len_;
  bool bad_;
};

// TinyUSB's CDC TX FIFO on this core (CFG_TUD_CDC_TX_BUFSIZE): put() needs
// the whole line free in it at once, so a longer line could never go out.
const int USB_TX_FIFO = 256;
static_assert(Line::CAP + 2 <= USB_TX_FIFO, "a full Line must fit the USB TX FIFO");

// Header line i (0, 1, ...) into l; false = no line i (end of header).
// Leaving l empty skips line i (e.g. an init failure that didn't happen).
typedef bool (*HeaderFn)(uint8_t i, Line &l);

template <class Port, uint8_t QN = 8>
class SerialOut {
 public:
  SerialOut(Port &port, HeaderFn header) : port_(port), header_(header) {}

  // Call every loop(): notices a host connecting, pushes header + events.
  void poll() {
    bool c = port_.dtr();
    if (c != conn_) {
      conn_ = c;
      qHead_ = qCount_ = 0;
      if (c) { hdr_ = 0; clearDrops(); }
    }
    if (!conn_) return;
    while (hdr_ != NO_HEADER) {
      Line l;
      if (!header_(hdr_, l)) { hdr_ = NO_HEADER; break; }
      if (l.length() && !put(l)) return;       // a bad (too long) line is skipped
      hdr_++;
    }
    while (qCount_) {
      if (!put(q_[qHead_])) return;
      qHead_ = (qHead_ + 1) % QN;
      qCount_--;
    }
  }

  // '#' event: queued behind earlier ones, ahead of later rows.
  void event(const Line &l) {
    poll();
    if (!conn_) return;
    if (!l.ok() || qCount_ == QN) { eventDrops_++; return; }
    q_[(qHead_ + qCount_) % QN] = l;
    qCount_++;
    poll();
  }

  // CSV row: sent now or dropped (counted). True if sent, or no host.
  bool row(const Line &l) {
    if (direct(l)) return true;
    rowDrops_++;
    return false;
  }

  // Stats line: like row() but a failure is not counted — the caller keeps its
  // counters and retries later. True = sent (or no host): reset the counters.
  bool stats(const Line &l) { return direct(l); }

  bool connected() const { return conn_; }
  uint32_t rowDrops() const { return rowDrops_; }
  uint32_t eventDrops() const { return eventDrops_; }
  void clearDrops() { rowDrops_ = eventDrops_ = 0; }

 private:
  static const uint8_t NO_HEADER = 0xFF;

  bool direct(const Line &l) {
    poll();
    if (!conn_) return true;
    return hdr_ == NO_HEADER && qCount_ == 0 && l.ok() && put(l);
  }

  // Whole line or nothing: with this much room write() returns at once.
  bool put(const Line &l) {
    int room = port_.availableForWrite();
    if (room < (int)l.size()) return false;
    return port_.write(l.bytes(), l.size()) == l.size();
  }

  Port &port_;
  HeaderFn header_;
  bool conn_ = false;
  uint8_t hdr_ = NO_HEADER;
  Line q_[QN];
  uint8_t qHead_ = 0, qCount_ = 0;
  uint32_t rowDrops_ = 0, eventDrops_ = 0;
};

}  // namespace posture
