#define __STDC_FORMAT_MACROS 1
#include <inttypes.h>
#include "midas.h"
#include "mfe.h"
#include "odbxx.h"

#include <libusb-1.0/libusb.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include <dirent.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>

/*-- SDG2122X SCPI protocol -------------------------------------------*/

#define SDG2122X_MAX_DEV         4      // realistically 1
#define SDG2122X_NUM_CH          2      // CH1, CH2
#define SDG2122X_SCPI_PORT       5025   // LAN raw SCPI port
#define SDG2122X_CMD_DELAY_MS    60     // settle between commands
#define SDG2122X_TCP_READ_TIMEOUT_MS 2000
#define SDG2122X_MAX_QUERY_READS 20     // bulk-in reads before giving up on a query
#define SDG2122X_USB_READ_TIMEOUT_MS 300
#define SDG2122X_MAX_RESP        64 * 1024
#define SDG2122X_MODE_POLL_EVERY 10     // read MODE?/ROSC? every Nth poll

// Siglent USBTMC vendor id. Both tested SDG2122X variants ("ee38" per the
// sdg2122-cli and the "1102" unit on this bench) show up under vendor f4ec.
#define SDG2122X_USB_VENDOR_ID  "f4ec"

static const char* SDG2122X_CH_NAME[SDG2122X_NUM_CH] = {"CH1", "CH2"};
static const char* SDG2122X_WAVE_MAIN[SDG2122X_NUM_CH] = {"C1", "C2"};

// USB-TMC framing constants (class USBTMC bulk layout).
#define USBTMC_MSG_DEV_DEP_MSG_OUT 1
#define USBTMC_MSG_DEV_DEP_MSG_IN  2
#define USBTMC_EP_BULK_OUT         0x01
#define USBTMC_EP_BULK_IN          0x81
#define USBTMC_IFACE               0

/*-- Small helpers -----------------------------------------------------*/

static double parse_number(const std::string& s) {
  // Siglent readbacks carry the unit glued to the number ("1000HZ", "0.001S",
  // "6.6V", "-53.0103dBm") and may use an SI multiplier ("1kHZ", "10MHz").
  // Parse the leading numeric quantity and apply a k/M/G/m/u/n/p multiplier
  // when the trailing unit actually uses one; ignore the rest of the string.
  const char* p = s.c_str();
  while (*p == ' ') p++;
  char* end = nullptr;
  double v = strtod(p, &end);
  if (end == p) return 0.0;
  // One SI-multiplier letter followed by at least one more alpha (a real
  // unit, e.g. "1kHZ" -> 1000). A bare number with no suffix is fine too.
  const char* q = end;
  if (*q) {
    char m = *q;
    double scale = 0;
    if      (m == 'k' || m == 'K') scale = 1e3;
    else if (m == 'M')             scale = 1e6;
    else if (m == 'G')             scale = 1e9;
    else if (m == 'm')             scale = 1e-3;
    else if (m == 'u')             scale = 1e-6;
    else if (m == 'n')             scale = 1e-9;
    else if (m == 'p')             scale = 1e-12;
    if (scale != 0 && q[1] != '\0' && isalpha((unsigned char)q[1]))
      v *= scale;
  }
  return v;
}

struct SCPIResponse {
  std::string raw;
  std::string cmd;      // "BSWV", "OUTP", ...
  std::string channel;  // "" | "C1" | "C2"
  std::map<std::string, std::string> kv;

  bool has(const char* k) const { return kv.find(k) != kv.end(); }
  std::string get(const char* k) const {
    auto it = kv.find(k);
    return it == kv.end() ? std::string() : it->second;
  }
};

// Parse a SIGLENT "legacy" comma-separated response ("C1:OUTP ON,LOAD,HZ,...")
// into a per-key map. Mirrors sdg2122-cli's parse_legacy_response semantics
// for the fields the frontend reads back.
static SCPIResponse parse_scpi(const std::string& text) {
  SCPIResponse r;
  r.raw = text;
  std::string s = text;
  // trim
  size_t b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return r;
  s = s.substr(b);
  // channel prefix
  if (s.size() >= 3 && s[1] == '1' && s[0] == 'C' && s[2] == ':') {
    r.channel = s.substr(0, 2); s = s.substr(3);
  } else if (s.size() >= 3 && s[1] == '2' && s[0] == 'C' && s[2] == ':') {
    r.channel = s.substr(0, 2); s = s.substr(3);
  }
  // trim again, then command token = up to first ' ' or ','
  b = s.find_first_not_of(" \t");
  if (b == std::string::npos) return r;
  s = s.substr(b);
  size_t sp = s.find_first_of(" ,");
  r.cmd = (sp == std::string::npos) ? s : s.substr(0, sp);
  std::string rest = (sp == std::string::npos) ? "" : s.substr(sp);
  rest.erase(0, rest.find_first_not_of(" ,"));
  if (rest.empty()) return r;

  // Single-token leading values the device emits without a KEY, prefix
  // (parsed the same way the CLI does).
  if (r.cmd == "OUTP" || r.cmd == "OUTPUT") {
    if (rest.compare(0, 2, "ON") == 0 || rest.compare(0, 3, "OFF") == 0) {
      size_t e = rest.find(',');
      r.kv["STATE"] = (e == std::string::npos) ? rest : rest.substr(0, e);
      rest = (e == std::string::npos) ? "" : rest.substr(e);
      rest.erase(0, rest.find_first_not_of(" ,"));
    }
  } else if (r.cmd == "ROSC") {
    if (rest.compare(0, 3, "INT") == 0 || rest.compare(0, 3, "EXT") == 0) {
      size_t e = rest.find(',');
      r.kv["SOURCE"] = (e == std::string::npos) ? rest : rest.substr(0, e);
      rest = (e == std::string::npos) ? "" : rest.substr(e);
      rest.erase(0, rest.find_first_not_of(" ,"));
    }
  } else if (r.cmd == "MODE") {
    if (rest.compare(0, 6, "PHASE-") == 0 || rest.compare(0, 12, "PHASELOCKED") == 0 ||
        rest.compare(0, 11, "INDEPENDENT") == 0) {
      size_t e = rest.find(',');
      r.kv["STATE"] = (e == std::string::npos) ? rest : rest.substr(0, e);
      rest = (e == std::string::npos) ? "" : rest.substr(e);
      rest.erase(0, rest.find_first_not_of(" ,"));
    }
  }

  if (rest.empty()) return r;
  // Remaining KEY,VALUE,KEY,VALUE,... pairs.
  std::vector<std::string> toks;
  size_t start = 0;
  while (true) {
    size_t c = rest.find(',', start);
    if (c == std::string::npos) { toks.push_back(rest.substr(start)); break; }
    toks.push_back(rest.substr(start, c - start));
    start = c + 1;
  }
  for (size_t i = 0; i + 1 < toks.size(); i += 2) {
    std::string key = toks[i];
    for (auto& ch : key) ch = (char)toupper((unsigned char)ch);
    std::string val = toks[i + 1];
    size_t vb = val.find_first_not_of(" ");
    if (vb != std::string::npos) val = val.substr(vb);
    r.kv[key] = val;
  }
  return r;
}

// Does the response belong to this query? The SDG2122X firmware echoes
// deferred/duplicated answers, so each response must be matched against the
// outstanding query before it is accepted. This mirrors _response_matches in
// sdg2122-cli's transport.py.
static bool response_matches(const std::string& text, const std::string& cmd) {
  if (cmd == "*IDN?" || cmd == "*IDN" )
    return text.find("SDG2122") != std::string::npos ||
           text.compare(0, 7, "Siglent") == 0;
  std::string head = cmd;
  size_t q = head.find('?');
  if (q != std::string::npos) head.resize(q);
  // Normalize the aliases the unit is known to use in readbacks.
  if (head == "BASIC_WAVE") head = "BSWV";
  if (head == "OUTPUT") head = "OUTP";
  return text.compare(0, head.size(), head) == 0;
}

static bool is_identity_response(const std::string& text) {
  return text.find("Siglent") != std::string::npos ||
         text.find("SDG2122") != std::string::npos;
}

// MDWV? snapshots omit a STATE field when modulation is enabled — they lead
// with the active type's namespace ("C1:MDWV AM,DEPTH,100,FRQ,1kHZ,...").
// Sniff the first comma-separated token after the command header.
static bool mdwv_enabled(const std::string& raw) {
  size_t sp = raw.find_first_of(" ,");
  std::string rest = (sp == std::string::npos) ? "" : raw.substr(sp + 1);
  rest.erase(0, rest.find_first_not_of(" ,"));
  return rest.compare(0, 3, "AM,") == 0 || rest.compare(0, 3, "FM,") == 0 ||
         rest.compare(0, 3, "PM,") == 0 || rest.compare(0, 4, "PWM,") == 0 ||
         rest.compare(0, 6, "DSBAM,") == 0;
}

/*-- LAN SCPI transport (raw TCP :5025) --------------------------------*/

class ScpiTcp {
  int    fd = -1;
public:
  bool open_(const std::string& host) {
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    struct addrinfo* res = nullptr;
    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", SDG2122X_SCPI_PORT);
    int end = getaddrinfo(host.c_str(), port_str, &hints, &res);
    if (end != 0 || res == nullptr) return false;
    for (struct addrinfo* p = res; p; p = p->ai_next) {
      int s = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (s < 0) continue;
      if (connect(s, p->ai_addr, p->ai_addrlen) == 0) { fd = s; break; }
      close(s);
    }
    freeaddrinfo(res);
    if (fd < 0) return false;
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &yes, sizeof(yes));
    struct timeval tv;
    tv.tv_sec = SDG2122X_TCP_READ_TIMEOUT_MS / 1000;
    tv.tv_usec = (SDG2122X_TCP_READ_TIMEOUT_MS % 1000) * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    return true;
  }
  void close_() { if (fd >= 0) { ::close(fd); fd = -1; } }
  bool is_open() const { return fd >= 0; }

  bool write_cmd(const std::string& cmd) {
    if (fd < 0) return false;
    std::string s = cmd + "\n";
    size_t n = 0;
    while (n < s.size()) {
      ssize_t w = ::write(fd, s.data() + n, s.size() - n);
      if (w < 0) { if (errno == EINTR) continue; return false; }
      n += w;
    }
    usleep(SDG2122X_CMD_DELAY_MS * 1000);
    return true;
  }

  bool query(const std::string& cmd, std::string& out) {
    if (fd < 0) return false;
    std::string s = cmd + "\n";
    size_t n = 0;
    while (n < s.size()) {
      ssize_t w = ::write(fd, s.data() + n, s.size() - n);
      if (w < 0) { if (errno == EINTR) continue; return false; }
      n += w;
    }
    out.clear();
    char buf[1024];
    bool eom = false;
    struct pollfd pfd; pfd.fd = fd; pfd.events = POLLIN;
    while (!eom && out.size() < SDG2122X_MAX_RESP) {
      int pr = poll(&pfd, 1, SDG2122X_TCP_READ_TIMEOUT_MS);
      if (pr <= 0) break;
      ssize_t r = ::read(fd, buf, sizeof(buf));
      if (r <= 0) break;
      for (ssize_t i = 0; i < r; i++) {
        if (buf[i] == '\n') { eom = true; break; }
        if (buf[i] != '\r') out.push_back(buf[i]);
      }
    }
    usleep(SDG2122X_CMD_DELAY_MS * 1000);
    return !out.empty();
  }
};

/*-- USB-TMC transport (libusb, SDG2122X firmware aware) ----------------*/

// The SDG2122X firmware (2.01.01.37R8 on this bench unit, and per the
// sdg2122-cli project the 39R7/ee38 variant too) returns legacy-command
// responses tagged with a stale USBTMC bTag, so the Linux kernel usbtmc
// driver discards them and every query times out. We therefore talk straight
// to the device with libusb, frame the USBTMC messages ourselves, and enforce
// a "write once, then drain until the matching response arrives" discipline:
// the firmware defers exactly one response per query-write, so stale prefix
// responses are discarded and the read loop keeps draining until a response
// whose header matches the outstanding query shows up. (The sdg2122-cli used
// the same framing but wrote each query twice; on this firmware double-writing
// only accumulates deferred junk.)
class ScpiUsb {
public:
  bool open_(const std::string& path) {
    struct libusb_device* dev = find_device(path);
    if (!dev) {
      cm_msg(MERROR, "SDG2122X", "no SIGLENT libusb device for %s "
             "(bus/addr not resolved or vendor f4ec absent)", path.c_str());
      return false;
    }
    int ret = libusb_open(dev, &h);
    if (ret != 0) {
      cm_msg(MERROR, "SDG2122X", "libusb_open(%s) failed: %s", path.c_str(),
             libusb_error_name(ret));
      return false;
    }
    libusb_set_auto_detach_kernel_driver(h, 1);
    if (libusb_claim_interface(h, USBTMC_IFACE) != 0) {
      cm_msg(MERROR, "SDG2122X", "claim_interface failed on %s", path.c_str());
      close_();
      return false;
    }
    connected = true;
    // Drain anything left over from a previous session so the first query
    // does not observe an old response.
    drain_bounded(4);
    return true;
  }

  void close_() {
    if (h) {
      if (connected) {
        libusb_release_interface(h, USBTMC_IFACE);
        connected = false;
      }
      libusb_close(h);
      h = nullptr;
    }
    tag = 1;
  }

  bool is_open() const { return h != nullptr; }

  bool write_cmd(const std::string& cmd) {
    if (!h) return false;
    std::string payload = cmd;
    if (payload.empty() || payload.back() != '\n') payload += "\n";
    if (!out_frame(payload, h, tag)) return false;
    tag = next_tag(tag);
    usleep(SDG2122X_CMD_DELAY_MS * 1000);
    return true;
  }

  bool query(const std::string& cmd, std::string& out) {
    if (!h) return false;
    std::string payload = cmd;
    if (payload.empty() || payload.back() != '\n') payload += "\n";
    if (!out_frame(payload, h, tag)) return false;
    tag = next_tag(tag);
    usleep(SDG2122X_CMD_DELAY_MS * 1000);

    for (int i = 0; i < SDG2122X_MAX_QUERY_READS; i++) {
      std::string text;
      int status = read_payload(text);
      if (status == 0 && response_matches(text, cmd)) {
        out = text;
        usleep(SDG2122X_CMD_DELAY_MS * 1000);
        return true;
      }
      // status==1: timeout, nothing pending. status==2: I/O error.
      if (status == 2) {
        cm_msg(MERROR, "SDG2122X", "query \"%s\" I/O error", cmd.c_str());
        return false;
      }
      usleep(5000);
    }
    cm_msg(MERROR, "SDG2122X", "query \"%s\" got no matching response in %d reads",
           cmd.c_str(), SDG2122X_MAX_QUERY_READS);
    return false;
  }

  // Drain pending deferred responses so they don't contaminate the next
  // query. Called after a plain write (the firmware answers every command
  // with a deferred response, even ones that are not queries).
  void drain_bounded(int max) {
    for (int i = 0; i < max; i++) {
      std::string text;
      int status = read_payload(text);
      if (status != 0) break;  // quiet
      if (text.empty() || is_identity_response(text)) continue;  // discard junk
    }
  }

private:
  libusb_device_handle* h = nullptr;
  bool connected = false;
  uint8_t tag = 1;

  static uint8_t next_tag(uint8_t t) { return (t % 255) + 1; }

  // Resolve a /dev/usbtmc* node to the matching libusb device, either by
  // bus/address read from sysfs (exact) or by vendor-id fallback.
  static libusb_device* find_device(const std::string& path) {
    int bus = -1, addr = -1;
    if (path.compare(0, 11, "/dev/usbtmc") == 0) {
      std::string leaf = path.substr(strlen("/dev/"));
      std::string sys = "/sys/class/usbmisc/" + leaf;
      FILE* f = fopen((sys + "/device/busnum").c_str(), "r");
      if (f) { if (fscanf(f, "%d", &bus) != 1) bus = -1; fclose(f); }
      f = fopen((sys + "/device/devnum").c_str(), "r");
      if (f) { if (fscanf(f, "%d", &addr) != 1) addr = -1; fclose(f); }
    }

    libusb_device** list = nullptr;
    ssize_t n = libusb_get_device_list(nullptr, &list);
    if (n < 0) return nullptr;
    libusb_device* found = nullptr;
    for (ssize_t i = 0; i < n && !found; i++) {
      struct libusb_device_descriptor d;
      if (libusb_get_device_descriptor(list[i], &d) != 0) continue;
      bool vendor_ok = false;
      char vbuf[8];
      snprintf(vbuf, sizeof(vbuf), "%04x", d.idVendor);
      if (strcmp(vbuf, SDG2122X_USB_VENDOR_ID) == 0) vendor_ok = true;
      if (!vendor_ok) continue;
      if (bus >= 0 && addr >= 0) {
        if (libusb_get_bus_number(list[i]) == bus && libusb_get_device_address(list[i]) == addr) {
          libusb_ref_device(list[i]);
          found = list[i];
        }
      } else {
        libusb_ref_device(list[i]);
        found = list[i];  // vendor match fallback
      }
    }
    libusb_free_device_list(list, 1);
    return found;
  }

  // Build and send a USBTMC initiate-write bulk-out frame (tag flows in the
  // endpoint header; the payload is padded to a 4-byte boundary).
  bool out_frame(const std::string& payload, libusb_device_handle* dev, uint8_t btag) {
    uint32_t len = (uint32_t)payload.size();
    uint8_t header[12];
    memset(header, 0, sizeof(header));
    header[0] = USBTMC_MSG_DEV_DEP_MSG_OUT;
    header[1] = btag;
    header[2] = (uint8_t)(255 - btag);
    header[3] = 0;
    header[4] = (uint8_t)(len & 0xFF);
    header[5] = (uint8_t)((len >> 8) & 0xFF);
    header[6] = (uint8_t)((len >> 16) & 0xFF);
    header[7] = (uint8_t)((len >> 24) & 0xFF);
    header[8] = 1;  // EOM
    header[9] = 0; header[10] = 0; header[11] = 0;

    size_t pad = ((12 + payload.size() + 3) & ~(size_t)3) - 12 - payload.size();
    std::string frame;
    frame.reserve(12 + payload.size() + pad);
    frame.append((const char*)header, 12);
    frame.append(payload);
    frame.append(pad, '\0');

    int transferred = 0;
    int ret = libusb_bulk_transfer(dev, USBTMC_EP_BULK_OUT,
                                   (unsigned char*)frame.data(), (int)frame.size(),
                                   &transferred, SDG2122X_USB_READ_TIMEOUT_MS + 1000);
    if (ret != 0 || transferred != (int)frame.size()) {
      cm_msg(MERROR, "SDG2122X", "bulk-out failed: %s (%d/%d)",
             libusb_error_name(ret), transferred, (int)frame.size());
      return false;
    }
    return true;
  }

  int read_payload(std::string& out) {
    out.clear();
    // USBTMC initiate-read request frame, then a bulk-in transfer.
    uint8_t req[12];
    memset(req, 0, sizeof(req));
    req[0] = USBTMC_MSG_DEV_DEP_MSG_IN;
    req[1] = tag;
    req[2] = (uint8_t)(255 - tag);
    uint32_t maxb = SDG2122X_MAX_RESP;
    req[4] = (uint8_t)(maxb & 0xFF);
    req[5] = (uint8_t)((maxb >> 8) & 0xFF);
    req[6] = (uint8_t)((maxb >> 16) & 0xFF);
    req[7] = (uint8_t)((maxb >> 24) & 0xFF);
    int transferred = 0;
    int ret = libusb_bulk_transfer(h, USBTMC_EP_BULK_OUT, req, sizeof(req),
                                   &transferred, SDG2122X_USB_READ_TIMEOUT_MS);
    if (ret != 0) return 1;  // timeout/NAK: nothing pending

    uint8_t buf[SDG2122X_MAX_RESP + 16];
    transferred = 0;
    ret = libusb_bulk_transfer(h, USBTMC_EP_BULK_IN, buf, sizeof(buf),
                               &transferred, SDG2122X_USB_READ_TIMEOUT_MS);
    if (ret != 0) {
      if (ret == LIBUSB_ERROR_TIMEOUT) return 1;
      return 2;
    }
    // The tested firmware occasionally emits a 4-byte all-zero packet while
    // applying coupling parameters; not a USBTMC response, drop it (treat as
    // "nothing pending yet" so the query loop keeps reading).
    if (transferred <= 4)
      return 1;
    if (transferred < 12) return 1;
    uint8_t msg_id = buf[0];
    uint8_t btag  = buf[1];
    (void)btag;
    uint32_t size = (uint32_t)buf[4] | ((uint32_t)buf[5] << 8) |
                    ((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 24);
    uint8_t attrs  = buf[8];
    if (msg_id != USBTMC_MSG_DEV_DEP_MSG_IN) return 1;
    if (size > SDG2122X_MAX_RESP) return 2;
    size_t got = transferred;
    if (got > 12 + size) got = 12 + size;
    size_t remaining = (12 + size) - got;
    while (remaining > 0) {
      int got2 = 0;
      int r2 = libusb_bulk_transfer(h, USBTMC_EP_BULK_IN, buf + got, sizeof(buf) - got,
                                    &got2, SDG2122X_USB_READ_TIMEOUT_MS);
      if (r2 != 0) return 1;
      got += got2;
      remaining = (12 + size) > got ? (12 + size) - got : 0;
    }
    if (!(attrs & 1)) return 1;  // EOM not set - not a complete message
    out.assign((const char*)buf + 12, size);
    while (!out.empty() && (out.back() == '\n' || out.back() == '\r'))
      out.pop_back();
    return 0;
  }
};

/*-- SDG2122X device wrapper -------------------------------------------*/

class SDG2122XDevice {
  enum Transport { TR_NONE = 0, TR_USB, TR_TCP };
  ScpiUsb usb;
  ScpiTcp tcp;
  Transport transport = TR_NONE;
  bool connected = false;
  std::string host;

public:
  SDG2122XDevice() {}
  ~SDG2122XDevice() { disconnect(); }

  int connect_device(const char* addr) {
    if (connected && host == addr) return 0;
    disconnect();
    if (strncmp(addr, "/dev/usbtmc", 11) == 0) {
      if (usb.open_(addr)) {
        // Mark the transport live BEFORE probing: probe()'s query needs
        // transport/connected set, or it is a no-op that always fails.
        transport = TR_USB;
        connected = true;
        if (!probe()) { disconnect(); return -1; }
        host = addr;
        return 0;
      }
      return -1;
    }
    if (tcp.open_(addr)) {
      transport = TR_TCP;
      connected = true;
      if (!probe()) { disconnect(); return -1; }
      host = addr;
      return 0;
    }
    return -1;
  }

  void disconnect() {
    usb.close_();
    tcp.close_();
    transport = TR_NONE;
    connected = false;
  }

  bool is_connected() const { return connected; }
  const char* get_host() const { return host.c_str(); }

  bool write_cmd(const std::string& cmd) {
    if (!connected) return false;
    if (transport == TR_USB) {
      if (usb.write_cmd(cmd)) return true;
      usb.close_(); connected = false;
      return false;
    }
    if (tcp.write_cmd(cmd)) return true;
    tcp.close_(); connected = false;
    return false;
  }

  bool query(const std::string& cmd, std::string& out) {
    if (!connected) return false;
    if (transport == TR_USB) {
      if (usb.query(cmd, out)) return true;
      usb.close_(); connected = false;
      return false;
    }
    if (tcp.query(cmd, out)) return true;
    tcp.close_(); connected = false;
    return false;
  }

  void drain_deferred() {
    if (transport == TR_USB) usb.drain_bounded(4);
  }

private:
  bool probe() {
    std::string idn;
    if (!query("*IDN?", idn)) return false;
    if (idn.find("SDG2122") == std::string::npos) {
      cm_msg(MERROR, "SDG2122X", "*IDN? responded but not an SDG2122X: \"%s\"", idn.c_str());
      return false;
    }
    cm_msg(MINFO, "SDG2122X", "Device says: %s", idn.c_str());
    return true;
  }
};

/*-- Auto-discovery -----------------------------------------------------*/

// Walk /dev/usbtmcN and return the first SIGLENT (vendor f4ec) node. The
// authoritative SDG2122X check happens in connect_device's *IDN? probe — a
// throwaway probe session here would leave this firmware's deferred-response
// pipeline in a state where the immediately following connect probe flakes.
static std::string discover_sdg2122(void) {
  DIR* d = opendir("/dev");
  if (!d) return "";
  std::string result;
  struct dirent* e;
  while ((e = readdir(d)) && result.empty()) {
    if (strncmp(e->d_name, "usbtmc", 6) != 0) continue;
    // /sys/class/usbmisc/usbtmcN/device is a symlink to the USB *interface*
    // directory; the idVendor we need sits in the parent usb_device dir.
    std::string linkpath = std::string("/sys/class/usbmisc/") + e->d_name + "/device";
    char resolved[PATH_MAX] = {0};
    if (realpath(linkpath.c_str(), resolved) == nullptr) continue;
    std::string vdir = resolved;
    size_t sl = vdir.find_last_of('/');
    if (sl == std::string::npos) continue;
    vdir.resize(sl);
    char vbuf[16] = {0};
    FILE* f = fopen((vdir + "/idVendor").c_str(), "r");
    if (!f) continue;
    size_t nn = fread(vbuf, 1, sizeof(vbuf) - 1, f);
    fclose(f);
    vbuf[nn] = 0;
    for (auto& c : std::string(vbuf)) c = (char)tolower((unsigned char)c);
    if (strstr(vbuf, "f4ec") == nullptr) continue;
    result = std::string("/dev/") + e->d_name;
  }
  closedir(d);
  return result;
}

// This firmware is erratic right after USB re-enumeration: the first session
// after discovery often fails *IDN?, while the next succeeds. Retry the whole
// open+probe a few times instead of trusting a single attempt.
static int connect_with_retry(SDG2122XDevice& d, const char* host) {
  for (int i = 0; i < 4; i++) {
    if (d.connect_device(host) == 0) return 0;
    d.disconnect();
    usleep(250 * 1000);
  }
  return -1;
}

/*-- Globals ------------------------------------------------------------*/

const char* frontend_name = "SDG2122X";
const char* frontend_file_name = __FILE__;

BOOL frontend_call_loop = FALSE;
INT  display_period = 0;

INT  max_event_size      = 1024 * 1024;
INT  max_event_size_frag = 1024 * 1024;
INT  event_buffer_size   = 10 * 1024 * 1024;

SDG2122XDevice dev[SDG2122X_MAX_DEV];
midas::odb    settings;
midas::odb    hist_vars;
int           num_devices = 1;
extern HNDLE  hDB;

/*-- Profile/stub declarations (defined below) --------------------------*/

INT frontend_init(void);
INT frontend_exit(void);
INT begin_of_run(INT, char*);
INT end_of_run(INT, char*);
INT pause_run(INT, char*);
INT resume_run(INT, char*);
INT frontend_loop(void);
INT read_periodic_event(char* pevent, INT off);
INT poll_event(INT, INT, BOOL);
INT interrupt_configure(INT, INT, POINTER_T);

BOOL equipment_common_overwrite = TRUE;

EQUIPMENT equipment[] = {
  {"SDG2122X%02d",           // equipment name
   {1004, 0,                 // event ID, trigger mask
    "SYSTEM",                // event buffer
    EQ_PERIODIC,             // equipment type
    0,                       // event source
    "MIDAS",                 // format
    TRUE,                    // enabled
    RO_ALWAYS,               // always read
    2000,                    // poll every 2000 ms
    0,                       // event limit
    0,                       // num sub events
    1,                       // log history
    "", "", "", "", "", 0},
   read_periodic_event       // readout routine
  },
  {""}
};

// Cached state for change detection across poll cycles
static char  last_host[SDG2122X_MAX_DEV][256] = {{0}};
static bool  device_initialized[SDG2122X_MAX_DEV] = {false};
static int   mode_poll_count[SDG2122X_MAX_DEV] = {0};

// Per-channel values as last applied to (or read back from) the hardware.
// -1 / "" sentinel = "never applied", forcing a write when the device is
// first (re)connected.
struct ChCache {
  double freq=-1, amp=-1, offset=-1, phase=-1, duty=-1, sym=-1, width=-1;
  int    arb_index=-1, out=-1;
  std::string wave, load, polarity;
  int    mod_enable=-1;
  std::string mod_src, mod_type;
  double mod_freq=-1, am_depth=-1, fm_dev=-1, pm_dev=-1, pwm_dev=-1;
  int    sweep_enable=-1;
  double sweep_start=-1, sweep_stop=-1, sweep_time=-1;
  std::string sweep_spacing, sweep_dir, sweep_trigger;
  int    burst_enable=-1;
  std::string burst_mode, burst_trigger;
  double burst_cycles=-1, burst_period=-1;
  int    harm_enable=-1;
  std::string harm_type;
  int    harm_cfg_hash = -1;
};
static ChCache last_ch[SDG2122X_MAX_DEV][SDG2122X_NUM_CH];
static int     last_phase_mode[SDG2122X_MAX_DEV] = {-1};
static int     last_clock_source[SDG2122X_MAX_DEV] = {-1};
static int     last_counter_enable[SDG2122X_MAX_DEV] = {-1};
static std::string last_counter_coupling[SDG2122X_MAX_DEV];

void reset_ch_cache(ChCache& c) {
  c = ChCache();
}

/*-- ODB subtree builders ----------------------------------------------*/

static void create_channel_subtree(const char* bpath, int ch) {
  midas::odb defaults({
    {"Output",                false},
    {"Wave Type",             std::string("SINE")},
    {"Options Wave Type",     std::array<std::string,7>{"SINE","SQUARE","RAMP","PULSE","NOISE","ARB","DC"}},
    {"Frequency (Hz)",        1000.0},
    {"Amplitude (Vpp)",       2.0},
    {"Offset (V)",            0.0},
    {"Phase (deg)",           0.0},
    {"Duty (%)",              50.0},
    {"Symmetry (%)",          50.0},
    {"Pulse Width (s)",       0.0001},
    {"Load",                  std::string("HZ")},
    {"Options Load",          std::array<std::string,2>{"HZ","50"}},
    {"Polarity",              std::string("NOR")},
    {"Options Polarity",      std::array<std::string,2>{"NOR","INVT"}},
    {"Arb Index",             1},
    {"Mod Enable",            false},
    {"Mod Source",            std::string("INT")},
    {"Options Mod Source",    std::array<std::string,2>{"INT","EXT"}},
    {"Mod Type",              std::string("AM")},
    {"Options Mod Type",      std::array<std::string,5>{"AM","FM","PM","PWM","DSBAM"}},
    {"Mod Frequency (Hz)",    1000.0},
    {"Mod Depth (%)",         50.0},
    {"FM Deviation (Hz)",     1000.0},
    {"PM Deviation (deg)",    10.0},
    {"PWM Deviation (%)",     50.0},
    {"Sweep Enable",          false},
    {"Sweep Start (Hz)",      100.0},
    {"Sweep Stop (Hz)",       10000.0},
    {"Sweep Time (s)",        1.0},
    {"Sweep Spacing",         std::string("LOG")},
    {"Options Sweep Spacing", std::array<std::string,2>{"LINE","LOG"}},
    {"Sweep Direction",       std::string("UP")},
    {"Options Sweep Direction", std::array<std::string,2>{"UP","DOWN"}},
    {"Sweep Trigger",         std::string("INT")},
    {"Options Sweep Trigger", std::array<std::string,3>{"INT","EXT","MAN"}},
    {"Burst Enable",          false},
    {"Burst Mode",            std::string("NCYC")},
    {"Options Burst Mode",    std::array<std::string,2>{"NCYC","GATE"}},
    {"Burst Cycles",          1.0},
    {"Burst Period (s)",      0.01},
    {"Burst Trigger",         std::string("INT")},
    {"Options Burst Trigger", std::array<std::string,3>{"INT","EXT","MAN"}},
    {"Harmonic Enable",       false},
    {"Harmonic Type",         std::string("ALL")},
    {"Options Harmonic Type", std::array<std::string,3>{"EVEN","ODD","ALL"}},
    {"Harmonic Config",       std::string("")},
    {"Wave Rdbk",             std::string("")},
    {"Freq Rdbk (Hz)",        0.0},
    {"Amp Rdbk (Vpp)",        0.0},
    {"Offs Rdbk (V)",         0.0},
    {"Phase Rdbk (deg)",      0.0},
    {"Load Rdbk",             std::string("")},
    {"Arb Name Rdbk",         std::string("")},
  });
  defaults.connect_and_fix_structure(bpath);
}

static void create_device_subtree(const char* bpath) {
  // Ensure string keys are strings (a buggy earlier version could have left
  // DIR/KEY entries behind).
  for (const char* k : {"Host", "Status", "Phase Mode", "Clock Source"}) {
    char kpath[512];
    snprintf(kpath, sizeof(kpath), "%s/%s", bpath, k);
    HNDLE hKey;
    if (db_find_key(hDB, 0, kpath, &hKey) == DB_SUCCESS) {
      KEY key;
      if (db_get_key(hDB, hKey, &key) == DB_SUCCESS && key.type != TID_STRING) {
        db_delete_key(hDB, hKey, FALSE);
      }
    }
  }

  midas::odb defaults({
    {"Phase Mode",               std::string("PHASE-LOCKED")},
    {"Options Phase Mode",       std::array<std::string,2>{"PHASE-LOCKED","INDEPENDENT"}},
    {"Clock Source",             std::string("INT")},
    {"Options Clock Source",     std::array<std::string,2>{"INT","EXT"}},
    {"CH1", {}},
    {"CH2", {}},
    {"Counter", {}},
  });
  defaults.connect_and_fix_structure(bpath);

  char sub[512];
  snprintf(sub, sizeof(sub), "%s/CH1", bpath); create_channel_subtree(sub, 0);
  snprintf(sub, sizeof(sub), "%s/CH2", bpath); create_channel_subtree(sub, 1);

  // Counter subtree (non-channel global frequency counter).
  {
    midas::odb cnt({
      {"Enable",           false},
      {"Coupling",         std::string("AC")},
      {"Options Coupling", std::array<std::string,2>{"AC","DC"}},
      {"Frequency (Hz)",   0.0},   // readback, updated every poll
    });
    char cpath[512];
    snprintf(cpath, sizeof(cpath), "%s/Counter", bpath);
    cnt.connect_and_fix_structure(cpath);
  }

  midas::odb dev_odb;
  dev_odb.connect(bpath);
  dev_odb["Host"]   = std::string("");
  dev_odb["Status"] = std::string("Not connected");
}

/*-- Per-channel wiring -------------------------------------------------*/

// Write length-limited amplitude guard before sending AMP/OFST so a mistake
// in the load's voltage rail doesn't damage equipment (mirrors the CLI's
// rail check: HZ -> 20Vpp rail, 50ohm -> 10Vpp rail).
static bool rail_ok(const std::string& load, double vpp, double offset) {
  double rail = (load == "50") ? 5.0 : 10.0;
  return fabs(offset) + vpp / 2.0 <= rail + 1e-9;
}

static void set_if_changed(SDG2122XDevice& d, const std::string& cmd,
                           const std::string& neww, std::string& last,
                           const char* label) {
  if (neww != last) {
    if (d.write_cmd(cmd)) {
      last = neww;
      cm_msg(MINFO, "SDG2122X", "%s -> %s", label, neww.c_str());
    }
  }
}

static void set_f_if_changed(SDG2122XDevice& d, const std::string& cmd,
                             double newv, double& last, const char* label) {
  if (newv != last) {
    char buf[80];
    snprintf(buf, sizeof(buf), "%.10g", newv);
    // Siglent SCPI pairs the field name and value with a comma ("BSWV AMP,2"),
    // exactly like the CLI renders it. A space separator is silently ignored.
    std::string s = cmd + "," + buf;
    if (d.write_cmd(s)) {
      last = newv;
      cm_msg(MINFO, "SDG2122X", "%s -> %s", label, buf);
    }
  }
}

static void set_bool_changed(SDG2122XDevice& d, const std::string& cmd,
                             int newv, int& last, const char* label) {
  if (newv != last) {
    std::string s = cmd + (newv ? " ON" : " OFF");
    if (d.write_cmd(s)) {
      last = newv;
      cm_msg(MINFO, "SDG2122X", "%s -> %s", label, newv ? "ON" : "OFF");
    }
  }
}

// Channel-2 firmware quirk similar to the CLI's: CH2's write-side built-in
// table omits M29..M31 and is shifted by 3 above that.
static int arb_wire_index(int ch, int index, bool& ok) {
  if (index < 1 || index > 196) { ok = false; return index; }
  if (ch == 1) {
    if (index >= 29 && index <= 31) { ok = false; return index; }
    if (index >= 32) index -= 3;
  }
  ok = true;
  return index;
}

// Apply one channel's selected modulation configuration. The device exposes
// per-mod-type namespaces (MDWV AM,SRC,... / MDWV FM,DEVI,...), so the chosen
// type plus its parameters are written as one transaction: mute, configure,
// then restore state. Only runs when something in the mod block changed.
static void apply_modulation(SDG2122XDevice& d, int ch,
                             const std::string& type, const std::string& src,
                             double freq, double depth, double fm_dev,
                             double pm_dev, double pwm_dev, bool enable,
                             ChCache& last) {
  bool dirty = (int)enable != last.mod_enable ||
               src != last.mod_src || type != last.mod_type ||
               freq != last.mod_freq;
  const std::string c = std::string(SDG2122X_WAVE_MAIN[ch]) + ":MDWV";
  std::string active = "";
  if (type == "AM")   { active = "DEPTH";   dirty = dirty || depth    != last.am_depth; }
  if (type == "FM")   { active = "DEVI";    dirty = dirty || fm_dev   != last.fm_dev; }
  if (type == "PM")   { active = "DEVI";    dirty = dirty || pm_dev   != last.pm_dev; }
  if (type == "PWM")  { active = "DEVI";    dirty = dirty || pwm_dev  != last.pwm_dev; }
  if (!dirty && (int)enable == last.mod_enable) return;

  if (!enable) {
    if (last.mod_enable == 1) {
      if (!d.write_cmd(c + " STATE,OFF")) return;
    }
  } else {
    // Mute, configure the active type's namespace, re-enable.
    d.write_cmd(c + " STATE,OFF");
    if (!d.write_cmd(c + " " + type + ",SRC," + src)) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "%.10g", freq);
    if (!d.write_cmd(c + " " + type + ",FRQ," + buf)) return;
    if (active == "DEPTH") {
      snprintf(buf, sizeof(buf), "%.10g", depth);
      d.write_cmd(c + " " + type + ",DEPTH," + buf);
    } else if (active == "DEVI") {
      double dev = 0;
      if      (type == "FM")  dev = fm_dev;
      else if (type == "PM")  dev = pm_dev;
      else if (type == "PWM") dev = pwm_dev;
      snprintf(buf, sizeof(buf), "%.10g", dev);
      d.write_cmd(c + " " + type + ",DEVI," + buf);
    }
    d.write_cmd(c + " STATE,ON");
  }
  // Cache only on success path.
  last.mod_enable = enable ? 1 : 0;
  last.mod_src = src; last.mod_type = type;
  last.mod_freq = freq; last.am_depth = depth; last.fm_dev = fm_dev;
  last.pm_dev = pm_dev; last.pwm_dev = pwm_dev;
  cm_msg(MINFO, "SDG2122X", "%s modulation %s (%s)",
         SDG2122X_CH_NAME[ch], enable ? "ON" : "OFF", type.c_str());
}

static void apply_sweep(SDG2122XDevice& d, int ch,
                        double start, double stop, double time,
                        const std::string& spacing, const std::string& dir,
                        const std::string& trigger, bool enable, ChCache& last) {
  bool dirty = (int)enable != last.sweep_enable ||
               start != last.sweep_start || stop != last.sweep_stop ||
               time  != last.sweep_time || spacing != last.sweep_spacing ||
               dir != last.sweep_dir || trigger != last.sweep_trigger;
  if (!dirty) return;
  const std::string c = std::string(SDG2122X_WAVE_MAIN[ch]) + ":SWWV";
  if (!enable) {
    if (last.sweep_enable == 1) d.write_cmd(c + " STATE,OFF");
  } else {
    d.write_cmd(c + " STATE,OFF");
    char buf[64];
    snprintf(buf, sizeof(buf), "%.10g", start);
    d.write_cmd(c + " START," + std::string(buf));
    snprintf(buf, sizeof(buf), "%.10g", stop);
    d.write_cmd(c + " STOP," + std::string(buf));
    snprintf(buf, sizeof(buf), "%.10g", time);
    d.write_cmd(c + " TIME," + std::string(buf));
    d.write_cmd(c + " SWMD," + spacing);
    d.write_cmd(c + " DIR," + dir);
    d.write_cmd(c + " TRSR," + trigger);
    d.write_cmd(c + " STATE,ON");
  }
  last.sweep_enable = enable ? 1 : 0;
  last.sweep_start = start; last.sweep_stop = stop; last.sweep_time = time;
  last.sweep_spacing = spacing; last.sweep_dir = dir; last.sweep_trigger = trigger;
  cm_msg(MINFO, "SDG2122X", "%s sweep %s", SDG2122X_CH_NAME[ch], enable ? "ON" : "OFF");
}

static void apply_burst(SDG2122XDevice& d, int ch,
                        const std::string& mode, double cycles, double period,
                        const std::string& trigger, bool enable, ChCache& last) {
  bool dirty = (int)enable != last.burst_enable || mode != last.burst_mode ||
               cycles != last.burst_cycles || period != last.burst_period ||
               trigger != last.burst_trigger;
  if (!dirty) return;
  const std::string c = std::string(SDG2122X_WAVE_MAIN[ch]) + ":BTWV";
  if (!enable) {
    if (last.burst_enable == 1) d.write_cmd(c + " STATE,OFF");
  } else {
    d.write_cmd(c + " STATE,OFF");
    char buf[64];
    d.write_cmd(c + " GATE_NCYC," + mode);
    snprintf(buf, sizeof(buf), "%.10g", cycles);
    d.write_cmd(c + " TIME," + std::string(buf));
    snprintf(buf, sizeof(buf), "%.10g", period);
    d.write_cmd(c + " PRD," + std::string(buf));
    d.write_cmd(c + " TRSR," + trigger);
    d.write_cmd(c + " STATE,ON");
  }
  last.burst_enable = enable ? 1 : 0;
  last.burst_mode = mode; last.burst_cycles = cycles;
  last.burst_period = period; last.burst_trigger = trigger;
  cm_msg(MINFO, "SDG2122X", "%s burst %s", SDG2122X_CH_NAME[ch], enable ? "ON" : "OFF");
}

static void apply_harmonic(SDG2122XDevice& d, int ch,
                           bool enable, const std::string& type,
                           const std::string& config, ChCache& last) {
  int hash = 0;
  for (char cc : config) hash = hash * 31 + cc;
  bool dirty = (int)enable != last.harm_enable || type != last.harm_type ||
               hash != last.harm_cfg_hash;
  if (!dirty) return;
  const std::string c = std::string(SDG2122X_WAVE_MAIN[ch]) + ":HARM";
  if (enable) {
    d.write_cmd(c + " HARMTYPE," + type);
    if (!config.empty())
      d.write_cmd(c + " HARMORDER," + config);
    d.write_cmd(c + " HARMSTATE,ON");
    usleep(120 * 1000);   // harmonic register writes need breathing room
  } else {
    d.write_cmd(c + " HARMSTATE,OFF");
  }
  last.harm_enable = enable ? 1 : 0;
  last.harm_type = type;
  last.harm_cfg_hash = hash;
  cm_msg(MINFO, "SDG2122X", "%s harmonic %s", SDG2122X_CH_NAME[ch], enable ? "ON" : "OFF");
}

/*-- Frontend Init ------------------------------------------------------*/

INT frontend_init() {
  cm_get_experiment_database(&hDB, NULL);

  int status = libusb_init(NULL);
  if (status != 0) {
    cm_msg(MERROR, "SDG2122X", "libusb_init failed: %d", status);
  }

  char settings_path[512];
  snprintf(settings_path, sizeof(settings_path), "/Equipment/%s/Settings", equipment[0].name);
  settings.connect(settings_path);

  // Create Device0 with defaults ONLY if it does not already exist. On a
  // restart the subtree must be left untouched: connect_and_fix_structure's
  // {"CH1",{}} outer wallet deletes existing CH1 children (any key not
  // represented in the wallet is deleted), which would reset every user-set
  // value back to its default on each frontend start.
  {
    char bpath[512];
    snprintf(bpath, sizeof(bpath), "%s/Device0", settings_path);
    HNDLE hKey;
    if (db_find_key(hDB, 0, bpath, &hKey) != DB_SUCCESS)
      create_device_subtree(bpath);
  }

  // Discover additional devices (1..SDG2122X_MAX_DEV-1)
  num_devices = 1;
  for (int i = 1; i < SDG2122X_MAX_DEV; i++) {
    char bpath[512];
    snprintf(bpath, sizeof(bpath), "%s/Device%d", settings_path, i);
    HNDLE hKey;
    if (db_find_key(hDB, 0, bpath, &hKey) == DB_SUCCESS) {
      num_devices = i + 1;
    } else {
      break;
    }
  }
  cm_msg(MINFO, "SDG2122X", "Found %d device(s) in ODB", num_devices);

  // Auto-discover Device0 via /dev/usbtmc scan if Host is empty.
  {
    char bpath[512];
    snprintf(bpath, sizeof(bpath), "%s/Device0", settings_path);
    midas::odb dev_odb;
    bool host_empty = true;
    try {
      dev_odb.connect(bpath);
      host_empty = ((std::string)dev_odb["Host"]).empty();
    } catch (...) { host_empty = true; }

    if (host_empty) {
      std::string found = discover_sdg2122();
      if (!found.empty()) {
        if (connect_with_retry(dev[0], found.c_str()) == 0) {
          try { dev_odb["Host"] = found; } catch (...) {}
          device_initialized[0] = true;
          strncpy(last_host[0], found.c_str(), sizeof(last_host[0]) - 1);
          last_host[0][sizeof(last_host[0]) - 1] = '\0';
          cm_msg(MINFO, "SDG2122X", "Auto-discovered SDG2122X at %s; Host field updated", found.c_str());
        } else {
          cm_msg(MERROR, "SDG2122X", "Found %s via /dev scan but *IDN? probe failed", found.c_str());
        }
      } else {
        cm_msg(MINFO, "SDG2122X", "No SDG2122X on USB; set Device0/Host manually in ODB");
      }
    }
  }

  // Create history Variables subtree
  {
    char var_path[512];
    snprintf(var_path, sizeof(var_path), "/Equipment/%s/Variables", equipment[0].name);
    hist_vars.connect(var_path, true);

    for (int i = 0; i < num_devices; i++) {
      for (int ch = 0; ch < SDG2122X_NUM_CH; ch++) {
        const char* cn = SDG2122X_CH_NAME[ch];
        char vname[64];
        snprintf(vname, sizeof(vname), "Freq_B%d_%s",  i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Amp_B%d_%s",   i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Offs_B%d_%s",  i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Phase_B%d_%s", i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Output_B%d_%s", i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Mod_B%d_%s",   i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Sweep_B%d_%s", i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Burst_B%d_%s", i, cn); hist_vars[vname] = 0.0f;
        snprintf(vname, sizeof(vname), "Harm_B%d_%s",  i, cn); hist_vars[vname] = 0.0f;
      }
      char vname[64];
      snprintf(vname, sizeof(vname), "Counter_B%d", i); hist_vars[vname] = 0.0f;
    }
    cm_msg(MINFO, "SDG2122X", "Created history variables under %s", var_path);
  }

  return SUCCESS;
}

INT frontend_exit() {
  for (int i = 0; i < SDG2122X_MAX_DEV; i++)
    dev[i].disconnect();
  libusb_exit(nullptr);
  return SUCCESS;
}

INT begin_of_run(INT run_number, char* error) { return SUCCESS; }
INT end_of_run(INT run_number, char* error)   { return SUCCESS; }
INT pause_run(INT run_number, char* error)    { return SUCCESS; }
INT resume_run(INT run_number, char* error)   { return SUCCESS; }
INT frontend_loop()                          { return SUCCESS; }

INT poll_event(INT source, INT count, BOOL test) {
  if (test) ss_sleep(count);
  return TRUE;
}

INT interrupt_configure(INT cmd, INT source, POINTER_T adr) {
  return SUCCESS;
}

/*-- Periodic readout (auto-connect, apply settings, monitor, history) ---*/

INT read_periodic_event(char* pevent, INT off) {
  bk_init32(pevent);
  char status_buf[512];

  for (int d = 0; d < num_devices && d < SDG2122X_MAX_DEV; d++) {
    char bpath[512];
    snprintf(bpath, sizeof(bpath), "/Equipment/%s/Settings/Device%d", equipment[0].name, d);

    midas::odb dev_odb;
    try { dev_odb.connect(bpath); }
    catch (...) { continue; }
    status_buf[0] = '\0';   // rebuilt per device; strncat below appends to it

    /* ---- 1. Read current settings from ODB ---- */
    std::string host;
    std::string phase_mode, clock_source;
    bool        counter_enable = false;
    std::string counter_coupling;
    try {
      host = (std::string)dev_odb["Host"];
      phase_mode   = (std::string)dev_odb["Phase Mode"];
      clock_source = (std::string)dev_odb["Clock Source"];
      char q[512];
      snprintf(q, sizeof(q), "%s/Counter", bpath);
      midas::odb cnt;
      cnt.connect(q);
      counter_enable = (BOOL)cnt["Enable"];
      counter_coupling = (std::string)cnt["Coupling"];
    } catch (...) { continue; }

    /* ---- 2. Connect / reconnect if Host changed or device lost ---- */
    if (!device_initialized[d] || strcmp(last_host[d], host.c_str()) != 0) {
      dev[d].disconnect();
      strncpy(last_host[d], host.c_str(), sizeof(last_host[d]) - 1);
      last_host[d][sizeof(last_host[d]) - 1] = '\0';

      if (!host.empty()) {
        int ret = connect_with_retry(dev[d], host.c_str());
        if (ret == 0) {
          cm_msg(MINFO, "SDG2122X", "Device %d: connected to %s", d, host.c_str());
        } else {
          cm_msg(MERROR, "SDG2122X", "Device %d: failed to connect to %s", d, host.c_str());
        }
      }
      for (int ch = 0; ch < SDG2122X_NUM_CH; ch++) reset_ch_cache(last_ch[d][ch]);
      last_phase_mode[d] = -1; last_clock_source[d] = -1;
      last_counter_enable[d] = -1;
      device_initialized[d] = true;
    }

    if (!dev[d].is_connected()) {
      try { dev_odb["Status"].set_string_size("Not connected", 64); } catch (...) {}
      continue;
    }

    /* ---- 3. Apply global settings if changed ---- */
    {
      int pm = (phase_mode == "INDEPENDENT") ? 2 : 1;
      if (pm != last_phase_mode[d]) {
        if (dev[d].write_cmd("MODE " + phase_mode)) {
          last_phase_mode[d] = pm;
          cm_msg(MINFO, "SDG2122X", "Device %d: phase mode -> %s", d, phase_mode.c_str());
        }
      }
      int cs = (clock_source == "EXT") ? 2 : 1;
      if (cs != last_clock_source[d]) {
        if (dev[d].write_cmd("ROSC " + clock_source)) {
          last_clock_source[d] = cs;
          cm_msg(MINFO, "SDG2122X", "Device %d: clock source -> %s", d, clock_source.c_str());
        }
      }
      int ce = counter_enable ? 1 : 0;
      if (ce != last_counter_enable[d] || counter_coupling != last_counter_coupling[d]) {
        dev[d].write_cmd("FCNT MODE," + counter_coupling);
        if (dev[d].write_cmd(counter_enable ? "FCNT STATE,ON" : "FCNT STATE,OFF")) {
          last_counter_enable[d] = ce;
          last_counter_coupling[d] = counter_coupling;
          cm_msg(MINFO, "SDG2122X", "Device %d: counter %s", d, counter_enable ? "ON" : "OFF");
        }
      }
    }

    /* ---- 4. Per-channel: read ODB, apply, read back ---- */
    double out_float[SDG2122X_NUM_CH]  = {0};
    double freq_float[SDG2122X_NUM_CH] = {0};
    double amp_float[SDG2122X_NUM_CH]  = {0};
    double offs_float[SDG2122X_NUM_CH] = {0};
    double phase_float[SDG2122X_NUM_CH] = {0};
    double mod_float[SDG2122X_NUM_CH] = {0};
    double sweep_float[SDG2122X_NUM_CH] = {0};
    double burst_float[SDG2122X_NUM_CH] = {0};
    double harm_float[SDG2122X_NUM_CH] = {0};

    for (int ch = 0; ch < SDG2122X_NUM_CH; ch++) {
      char cpath[512];
      snprintf(cpath, sizeof(cpath), "%s/%s", bpath, SDG2122X_CH_NAME[ch]);
      midas::odb ch_odb;
      try { ch_odb.connect(cpath); } catch (...) { continue; }

      /* ---- 4a. read settings ---- */
      bool        output   = false;
      std::string wave, load, polarity, mod_src, mod_type, sweep_spacing,
                  sweep_dir, sweep_trg, burst_mode, burst_trg, harm_type, harm_cfg;
      bool        mod_en, sweep_en, burst_en, harm_en = false;
      double      freq = 1000, amp = 2, offset = 0, phase = 0, duty = 50, sym = 50, width = 0.0001;
      double      mod_freq = 1000, am_depth = 50, fm_dev = 1000, pm_dev = 10, pwm_dev = 50;
      double      sweep_start = 100, sweep_stop = 10000, sweep_time = 1;
      double      burst_cycles = 1, burst_period = 0.01;
      int         arb_idx = 1;
      try {
        output = (BOOL)ch_odb["Output"];
        wave   = (std::string)ch_odb["Wave Type"];
        load   = (std::string)ch_odb["Load"];
        polarity = (std::string)ch_odb["Polarity"];
        arb_idx = ch_odb["Arb Index"];
        freq    = ch_odb["Frequency (Hz)"];
        amp     = ch_odb["Amplitude (Vpp)"];
        offset  = ch_odb["Offset (V)"];
        phase   = ch_odb["Phase (deg)"];
        duty    = ch_odb["Duty (%)"];
        sym     = ch_odb["Symmetry (%)"];
        width   = ch_odb["Pulse Width (s)"];
        mod_en  = (BOOL)ch_odb["Mod Enable"];
        mod_src = (std::string)ch_odb["Mod Source"];
        mod_type = (std::string)ch_odb["Mod Type"];
        mod_freq = ch_odb["Mod Frequency (Hz)"];
        am_depth = ch_odb["Mod Depth (%)"];
        fm_dev  = ch_odb["FM Deviation (Hz)"];
        pm_dev  = ch_odb["PM Deviation (deg)"];
        pwm_dev = ch_odb["PWM Deviation (%)"];
        sweep_en = (BOOL)ch_odb["Sweep Enable"];
        sweep_start = ch_odb["Sweep Start (Hz)"];
        sweep_stop  = ch_odb["Sweep Stop (Hz)"];
        sweep_time  = ch_odb["Sweep Time (s)"];
        sweep_spacing = (std::string)ch_odb["Sweep Spacing"];
        sweep_dir  = (std::string)ch_odb["Sweep Direction"];
        sweep_trg  = (std::string)ch_odb["Sweep Trigger"];
        burst_en   = (BOOL)ch_odb["Burst Enable"];
        burst_mode = (std::string)ch_odb["Burst Mode"];
        burst_cycles = ch_odb["Burst Cycles"];
        burst_period = ch_odb["Burst Period (s)"];
        burst_trg  = (std::string)ch_odb["Burst Trigger"];
        harm_en    = (BOOL)ch_odb["Harmonic Enable"];
        harm_type  = (std::string)ch_odb["Harmonic Type"];
        harm_cfg   = (std::string)ch_odb["Harmonic Config"];
      } catch (...) { continue; }

      /* ---- 4b. apply changed settings (order matches sdg2122-cli) ---- */
      ChCache& last = last_ch[d][ch];
      const std::string C = std::string(SDG2122X_WAVE_MAIN[ch]);

      // Load first: switching HZ<->50 rescales the displayed amplitude.
      set_if_changed(dev[d], C + ":OUTP LOAD," + load, load, last.load,
                     (std::string(SDG2122X_CH_NAME[ch]) + " load").c_str());

      // Arb selection only when the user chose ARB.
      if (wave == "ARB") {
        bool ok = true;
        int wire = arb_wire_index(ch, arb_idx, ok);
        if (ok && wire != last.arb_index) {
          char buf[64];
          snprintf(buf, sizeof(buf), "%d", wire);
          if (dev[d].write_cmd(C + ":ARWV INDEX," + std::string(buf))) {
            last.arb_index = wire;
            cm_msg(MINFO, "SDG2122X", "%s arb index -> %d", SDG2122X_CH_NAME[ch], wire);
          }
        }
      } else if (last.arb_index != -1) {
        last.arb_index = -1;  // leaving ARB mode clears the pending arb write
      }

      set_if_changed(dev[d], C + ":BSWV WVTP," + wave, wave, last.wave,
                     (std::string(SDG2122X_CH_NAME[ch]) + " wave").c_str());

      // Amplitude guard before sending the pair that could exceed the rail.
      if ((amp != last.amp || offset != last.offset) && !rail_ok(load, amp, offset)) {
        cm_msg(MERROR, "SDG2122X", "%s: |offset|+amp/2=%.3fV exceeds %s rail; AMP/OFFSET not applied",
               SDG2122X_CH_NAME[ch], fabs(offset) + amp / 2.0,
               load == "50" ? "50 ohm 5V" : "HZ 10V");
        amp = last.amp; offset = last.offset;   // keep old values
      }
      set_f_if_changed(dev[d], C + ":BSWV FRQ", freq, last.freq,
                       (std::string(SDG2122X_CH_NAME[ch]) + " freq").c_str());
      set_f_if_changed(dev[d], C + ":BSWV AMP", amp, last.amp,
                       (std::string(SDG2122X_CH_NAME[ch]) + " amp").c_str());
      set_f_if_changed(dev[d], C + ":BSWV OFST", offset, last.offset,
                       (std::string(SDG2122X_CH_NAME[ch]) + " offset").c_str());
      set_f_if_changed(dev[d], C + ":BSWV PHSE", phase, last.phase,
                       (std::string(SDG2122X_CH_NAME[ch]) + " phase").c_str());
      set_f_if_changed(dev[d], C + ":BSWV DUTY", duty, last.duty,
                       (std::string(SDG2122X_CH_NAME[ch]) + " duty").c_str());
      set_f_if_changed(dev[d], C + ":BSWV SYM", sym, last.sym,
                       (std::string(SDG2122X_CH_NAME[ch]) + " symmetry").c_str());
      set_f_if_changed(dev[d], C + ":BSWV WIDTH", width, last.width,
                       (std::string(SDG2122X_CH_NAME[ch]) + " width").c_str());
      set_if_changed(dev[d], C + ":OUTP PLRT," + polarity, polarity, last.polarity,
                     (std::string(SDG2122X_CH_NAME[ch]) + " polarity").c_str());

      // Modes (mod/sweep/burst are mutually exclusive on a channel; the
      // device keeps the last one enabled, and the CLI README's rule is to
      // disable the previous before enabling another).
      apply_modulation(dev[d], ch, mod_type, mod_src, mod_freq, am_depth,
                       fm_dev, pm_dev, pwm_dev, mod_en, last);
      apply_sweep(dev[d], ch, sweep_start, sweep_stop, sweep_time,
                  sweep_spacing, sweep_dir, sweep_trg, sweep_en, last);
      apply_burst(dev[d], ch, burst_mode, burst_cycles, burst_period,
                  burst_trg, burst_en, last);
      apply_harmonic(dev[d], ch, harm_en, harm_type, harm_cfg, last);

      // Output last, so changing wave parameters never fights a live output.
      set_bool_changed(dev[d], C + ":OUTP", output ? 1 : 0, last.out,
                       (std::string(SDG2122X_CH_NAME[ch]) + " output").c_str());

      /* ---- 4c. read back the configured waveform ---- */
      std::string out_resp, wv_resp, mod_resp, swp_resp, bst_resp, arb_resp;
      if (dev[d].query(C + ":OUTP?", out_resp) ) {
        SCPIResponse o = parse_scpi(out_resp);
        std::string st = o.get("STATE");
        bool dev_on = (st == "ON");
        double dev_out = dev_on ? 1 : 0;
        out_float[ch] = dev_out;
        try { ch_odb["Load Rdbk"] = std::string(o.get("LOAD")); } catch (...) {}
        // Reconcile output enable if the device disagrees (front-panel edits).
        if (o.has("STATE") && (int)dev_out != last.out) {
          last.out = (int)dev_out;
          try { ch_odb["Output"] = (BOOL)dev_on; } catch (...) {}
        }
      }
      if (dev[d].query(C + ":BSWV?", wv_resp)) {
        SCPIResponse w = parse_scpi(wv_resp);
        std::string wt = w.get("WVTP");
        double wf = parse_number(w.get("FRQ"));
        double wa = parse_number(w.get("AMP"));
        double wo = parse_number(w.get("OFST"));
        double wp = parse_number(w.get("PHSE"));
        if (!w.has("WVTP")) wt = wave;   // trust ODB if device omitted
        try {
          ch_odb["Wave Rdbk"]  = std::string(wt);
          ch_odb["Freq Rdbk (Hz)"]  = wf;
          ch_odb["Amp Rdbk (Vpp)"]  = wa;
          ch_odb["Offs Rdbk (V)"]   = wo;
          ch_odb["Phase Rdbk (deg)"] = wp;
        } catch (...) {}
        freq_float[ch]  = w.has("FRQ") ? wf : freq;
        amp_float[ch]   = w.has("AMP") ? wa : amp;
        offs_float[ch]  = w.has("OFST") ? wo : offset;
        phase_float[ch] = w.has("PHSE") ? wp : phase;
      }
      // Modes readback: reconcile the enable flags and ODB from the device.
      if (dev[d].query(C + ":MDWV?", mod_resp)) {
        SCPIResponse m = parse_scpi(mod_resp);
        bool on = m.has("STATE") ? (m.get("STATE") == "ON")
                                 : mdwv_enabled(mod_resp);
        mod_float[ch] = on ? 1 : 0;
        if (m.has("STATE") || mdwv_enabled(mod_resp)) {
          int dev_en = on ? 1 : 0;
          if (dev_en != last.mod_enable) {
            last.mod_enable = dev_en;
            try { ch_odb["Mod Enable"] = (BOOL)on; } catch (...) {}
          }
        }
      }
      if (dev[d].query(C + ":SWWV?", swp_resp)) {
        SCPIResponse m = parse_scpi(swp_resp);
        bool on = (m.get("STATE") == "ON");
        sweep_float[ch] = on ? 1 : 0;
        if (m.has("STATE")) {
          int dev_en = on ? 1 : 0;
          if (dev_en != last.sweep_enable) {
            last.sweep_enable = dev_en;
            try { ch_odb["Sweep Enable"] = (BOOL)on; } catch (...) {}
          }
        }
      }
      if (dev[d].query(C + ":BTWV?", bst_resp)) {
        SCPIResponse m = parse_scpi(bst_resp);
        bool on = (m.get("STATE") == "ON");
        burst_float[ch] = on ? 1 : 0;
        if (m.has("STATE")) {
          int dev_en = on ? 1 : 0;
          if (dev_en != last.burst_enable) {
            last.burst_enable = dev_en;
            try { ch_odb["Burst Enable"] = (BOOL)on; } catch (...) {}
          }
        }
      }
      if (dev[d].query(C + ":ARWV?", arb_resp)) {
        SCPIResponse a = parse_scpi(arb_resp);
        if (a.has("NAME")) {
          try { ch_odb["Arb Name Rdbk"] = std::string(a.get("NAME")); } catch (...) {}
        }
      }

      /* ---- 4d. status line fragment ---- */
      {
        char full[160];
        const char* olabel = out_float[ch] > 0.5 ? "ON" : "OFF";
        double hz = freq_float[ch];
        char hzb[32];
        if (hz >= 1e6) snprintf(hzb, sizeof(hzb), "%.4gMHz", hz / 1e6);
        else if (hz >= 1e3) snprintf(hzb, sizeof(hzb), "%.4gkHz", hz / 1e3);
        else snprintf(hzb, sizeof(hzb), "%.5gHz", hz);
        char ab[24];
        snprintf(ab, sizeof(ab), "%.4gVpp", amp_float[ch]);
        snprintf(full, sizeof(full), "%s | %s %s %s %s",
                 SDG2122X_CH_NAME[ch], olabel, wave.c_str(), hzb, ab);
        if (mod_float[ch])   strncat(full, "+MOD", sizeof(full) - strlen(full) - 1);
        if (sweep_float[ch]) strncat(full, "+SWP", sizeof(full) - strlen(full) - 1);
        if (burst_float[ch]) strncat(full, "+BST", sizeof(full) - strlen(full) - 1);
        if (ch > 0) strncat(status_buf, "; ", sizeof(status_buf) - strlen(status_buf) - 1);
        strncat(status_buf, full, sizeof(status_buf) - strlen(status_buf) - 1);
      }
    }

    /* ---- 4e. counter readback ---- */
    double counter_freq = 0;
    if (counter_enable) {
      std::string resp;
      if (dev[d].query("FCNT?", resp)) {
        SCPIResponse c = parse_scpi(resp);
        if (c.has("MAFREQ"))
          counter_freq = parse_number(c.get("MAFREQ"));
        try {
          char q[512];
          snprintf(q, sizeof(q), "%s/Counter", bpath);
          midas::odb cnt;
          cnt.connect(q);
          cnt["Frequency (Hz)"] = counter_freq;
        } catch (...) {}
      }
    }
    char ctr_buf[48];
    if (counter_enable)
      snprintf(ctr_buf, sizeof(ctr_buf), "Counter %.4gHz", counter_freq);
    else
      snprintf(ctr_buf, sizeof(ctr_buf), "Counter off");
    if (strlen(status_buf))
      strncat(status_buf, " | ", sizeof(status_buf) - strlen(status_buf) - 1);
    strncat(status_buf, ctr_buf, sizeof(status_buf) - strlen(status_buf) - 1);

    /* ---- 4f. global readbacks (infrequently) ---- */
    if (dev[d].is_connected() &&
        (++mode_poll_count[d] % SDG2122X_MODE_POLL_EVERY) == 0) {
      std::string resp;
      if (dev[d].query("MODE?", resp)) {
        SCPIResponse m = parse_scpi(resp);
        std::string st = m.get("STATE");
        if (!st.empty() && st != phase_mode) {
          try { dev_odb["Phase Mode"] = std::string(st); } catch (...) {}
        }
      }
      if (dev[d].query("ROSC?", resp)) {
        SCPIResponse m = parse_scpi(resp);
        std::string st = m.get("SOURCE");
        if (!st.empty() && st != clock_source) {
          try { dev_odb["Clock Source"] = std::string(st); } catch (...) {}
        }
      }
    }

    /* ---- 5. Status string + history ---- */
    try { dev_odb["Status"].set_string_size(status_buf, 256); } catch (...) {}

    try {
      for (int ch = 0; ch < SDG2122X_NUM_CH; ch++) {
        const char* cn = SDG2122X_CH_NAME[ch];
        char vname[64];
        snprintf(vname, sizeof(vname), "Freq_B%d_%s",  d, cn); hist_vars[vname] = freq_float[ch];
        snprintf(vname, sizeof(vname), "Amp_B%d_%s",   d, cn); hist_vars[vname] = amp_float[ch];
        snprintf(vname, sizeof(vname), "Offs_B%d_%s",  d, cn); hist_vars[vname] = offs_float[ch];
        snprintf(vname, sizeof(vname), "Phase_B%d_%s", d, cn); hist_vars[vname] = phase_float[ch];
        snprintf(vname, sizeof(vname), "Output_B%d_%s", d, cn); hist_vars[vname] = out_float[ch];
        snprintf(vname, sizeof(vname), "Mod_B%d_%s",   d, cn); hist_vars[vname] = mod_float[ch];
        snprintf(vname, sizeof(vname), "Sweep_B%d_%s", d, cn); hist_vars[vname] = sweep_float[ch];
        snprintf(vname, sizeof(vname), "Burst_B%d_%s", d, cn); hist_vars[vname] = burst_float[ch];
        snprintf(vname, sizeof(vname), "Harm_B%d_%s",  d, cn); hist_vars[vname] = harm_float[ch];
      }
      char vname[64];
      snprintf(vname, sizeof(vname), "Counter_B%d", d);
      hist_vars[vname] = counter_freq;
    } catch (...) {}

    /* ---- 6. Write event bank for MIDAS history logging ---- */
    // Layout per device (19 floats):
    //   [CH1.freq, CH1.amp, CH1.offset, CH1.phase, CH1.out, CH1.mod, CH1.sweep, CH1.burst, CH1.harm,
    //    CH2...same 9...,
    //    counter]
    float data[19];
    int idx = 0;
    for (int ch = 0; ch < SDG2122X_NUM_CH; ch++) {
      data[idx++] = freq_float[ch];
      data[idx++] = amp_float[ch];
      data[idx++] = offs_float[ch];
      data[idx++] = phase_float[ch];
      data[idx++] = out_float[ch];
      data[idx++] = mod_float[ch];
      data[idx++] = sweep_float[ch];
      data[idx++] = burst_float[ch];
      data[idx++] = harm_float[ch];
    }
    data[idx++] = counter_freq;

    char bank_name[8];
    snprintf(bank_name, sizeof(bank_name), "VAR%d", d);
    float* pdata;
    bk_create(pevent, bank_name, TID_FLOAT, (void**)&pdata);
    memcpy(pdata, data, sizeof(float) * 19);
    bk_close(pevent, pdata + 19);
  }

  return bk_size(pevent);
}