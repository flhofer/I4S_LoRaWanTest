/*
  This file is part of the MKRWAN library.
  Copyright (C) 2017  Arduino AG (http://www.arduino.cc/)

  Based on the TinyGSM library https://github.com/vshymanskyy/TinyGSM
  Copyright (c) 2016 Volodymyr Shymanskyy

  MKRWAN library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  MKRWAN library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with MKRWAN library.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"

#ifdef PORTENTA_CARRIER
#undef LORA_RESET
#define LORA_RESET (PD_5)
#undef LORA_BOOT0
#define LORA_BOOT0 (PJ_11)
#endif

#define DEFAULT_JOIN_TIMEOUT 60000L

template <class T, unsigned N>
class SerialFifo
{
public:
    SerialFifo()
    {
        clear();
    }

    void clear()
    {
        _r = 0;
        _w = 0;
    }

    // writing thread/context API
    //-------------------------------------------------------------

    bool writeable(void)
    {
        return free() > 0;
    }

    int free(void)
    {
        int s = _r - _w;
        if (s <= 0)
            s += N;
        return s - 1;
    }

    bool put(const T& c)
    {
        int i = _w;
        int j = i;
        i = _inc(i);
        if (i == _r) // !writeable()
            return false;
        _b[j] = c;
        _w = i;
        return true;
    }

    int put(const T* p, int n, bool t = false)
    {
        int c = n;
        while (c)
        {
            int f;
            while ((f = free()) == 0) // wait for space
            {
                if (!t) return n - c; // no more space and not blocking
                /* nothing / just wait */;
            }
            // check free space
            if (c < f) f = c;
            int w = _w;
            int m = N - w;
            // check wrap
            if (f > m) f = m;
            memcpy(&_b[w], p, f);
            _w = _inc(w, f);
            c -= f;
            p += f;
        }
        return n - c;
    }

    // reading thread/context API
    // --------------------------------------------------------

    bool readable(void)
    {
        return (_r != _w);
    }

    size_t size(void)
    {
        int s = _w - _r;
        if (s < 0)
            s += N;
        return s;
    }

    bool get(T* p)
    {
        int r = _r;
        if (r == _w) // !readable()
            return false;
        *p = _b[r];
        _r = _inc(r);
        return true;
    }

    bool peek(T* p)
    {
        int r = _r;
        if (r == _w) // !readable()
            return false;
        *p = _b[r];
        return true;
    }

    int get(T* p, int n, bool t = false)
    {
        int c = n;
        while (c)
        {
            int f;
            for (;;) // wait for data
            {
                f = size();
                if (f)  break;        // free space
                if (!t) return n - c; // no space and not blocking
                /* nothing / just wait */;
            }
            // check available data
            if (c < f) f = c;
            int r = _r;
            int m = N - r;
            // check wrap
            if (f > m) f = m;
            memcpy(p, &_b[r], f);
            _r = _inc(r, f);
            c -= f;
            p += f;
        }
        return n - c;
    }

private:
    int _inc(int i, int n = 1)
    {
        return (i + n) % N;
    }

    T    _b[N];
    int  _w;
    int  _r;
};

#ifndef YIELD
  #define YIELD() { delay(2); }
#endif

typedef const char* ConstStr;
#define GFP(x) x
#define GF(x)  x

#ifdef LORA_DEBUG
namespace {
  template<typename T>
  static void DBG(T last) {
    LORA_DEBUG.println(last);
  }

  template<typename T, typename... Args>
  static void DBG(T head, Args... tail) {
    LORA_DEBUG.print(head);
    LORA_DEBUG.print(' ');
    DBG(tail...);
  }
}
#else
  #define DBG(...)
#endif

template<class T>
const T& Min(const T& a, const T& b)
{
    return (b < a) ? b : a;
}

template<class T>
const T& Max(const T& a, const T& b)
{
    return (b < a) ? a : b;
}

#if !defined(LORA_RX_BUFFER)
  #define LORA_RX_BUFFER 256
#endif

#define LORA_NL "\r"
static const char LORA_OK[] = "+OK\r";
static const char LORA_ERROR[] = "+ERR\r";
static const char LORA_ERROR_PARAM[] = "+ERR_PARAM\r";
static const char LORA_ERROR_BUSY[] = "+ERR_BUSY\r";
static const char LORA_ERROR_OVERFLOW[] = "+ERR_PARAM_OVERFLOW\r";
static const char LORA_ERROR_NO_NETWORK[] = "+ERR_NO_NETWORK\r";
static const char LORA_ERROR_RX[] = "+ERR_RX\r";
static const char LORA_ERROR_UNKNOWN[] = "+ERR_UNKNOWN\r";

static const char ARDUINO_FW_VERSION[] = "ARD-078 1.2.4";
static const char ARDUINO_FW_IDENTIFIER[] = "ARD-078";

typedef enum {
    AS923 = 0,
    AU915,
    CN470,
    CN779,
    EU433,
    EU868 = 5,
    KR920,
    IN865,
    US915,
    US915_HYBRID,
} _lora_band;

typedef enum {
    RFO = 0,
    PABOOST,
} _rf_mode;

typedef enum {
    ABP = 0,
    OTAA,
} _lora_mode;

typedef enum {
    APP_EUI = 0,
    APP_KEY,
    DEV_EUI,
    DEV_ADDR,
    NWKS_KEY,
    APPS_KEY,
    NWK_ID,
} _lora_property;

typedef enum {
    CLASS_A = 'A',
    CLASS_B,
    CLASS_C,
} _lora_class;

class LoRaModem : public Stream // @suppress("Class has a virtual method and non-virtual destructor")
{

public:
  LoRaModem(__attribute__((unused)) Stream& stream = (Stream&)Serial)
#ifdef SerialLoRa
    : stream(SerialLoRa), lastPollTime(millis()), pollInterval(300000)
#else
    : stream(stream), lastPollTime(millis()), pollInterval(300000)
#endif
    {
	  network_joined = false;
	  mask_size = 1;
	  region = EU868;
    }

public:
  typedef SerialFifo<uint8_t, LORA_RX_BUFFER> RxFifo;

private:
  Stream&       stream;
  bool          network_joined;
  RxFifo        rx;
  RxFifo        tx;
  String        fw_version;
  unsigned long lastPollTime;
  unsigned long pollInterval;
  int           mask_size;
  uint16_t      channelsMask[6];
  String        channel_mask_str;
  _lora_band    region;

public:
  virtual int joinOTAA(const char *appEui, const char *appKey, const char *devEui, uint32_t timeout) {
    YIELD();
    rx.clear();
    changeMode(OTAA);
    set(APP_EUI, appEui);
    set(APP_KEY, appKey);
    if (devEui != NULL) {
        set(DEV_EUI, devEui);
    }
    network_joined = join(timeout);
    delay(1000);
    return network_joined;
  }

  virtual int joinOTAA(String appEui, String appKey, uint32_t timeout = DEFAULT_JOIN_TIMEOUT) {
    return joinOTAA(appEui.c_str(), appKey.c_str(), NULL, timeout);
  }

  virtual int joinOTAA(String appEui, String appKey, String devEui, uint32_t timeout = DEFAULT_JOIN_TIMEOUT) {
    return joinOTAA(appEui.c_str(), appKey.c_str(), devEui.c_str(), timeout);
  }

  virtual int joinABP(/*const char* nwkId, */const char * devAddr, const char * nwkSKey, const char * appSKey, uint32_t timeout = DEFAULT_JOIN_TIMEOUT) {
    YIELD();
    rx.clear();
    changeMode(ABP);
    //set(NWK_ID, nwkId);
    set(DEV_ADDR, devAddr);
    set(NWKS_KEY, nwkSKey);
    set(APPS_KEY, appSKey);
    network_joined = join(timeout);
    return (getJoinStatus() == 1);
  }

  virtual int joinABP(/*String nwkId, */String devAddr, String nwkSKey, String appSKey) {
    return joinABP(/*nwkId.c_str(), */devAddr.c_str(), nwkSKey.c_str(), appSKey.c_str());
  }

  // Stream compatibility (like UDP)
  void beginPacket() {
    tx.clear();
  }

  int endPacket(bool confirmed = false) {
    uint8_t buffer[LORA_RX_BUFFER];
    memset(buffer, 0, LORA_RX_BUFFER);
    int size = tx.get(buffer, tx.size());
    return modemSend(buffer, size, confirmed);
  }

  size_t write(uint8_t c) {
    return tx.put(c);
  };

  size_t write(const uint8_t *buffer, size_t size) {
    return tx.put(buffer, size);
  };

  template <typename T> inline size_t write(T val) {return write((uint8_t*)&val, sizeof(T));};
  using Print::write;

  int parsePacket() {
    return available();
  }

  virtual int available() {
    YIELD();
    if (!rx.size()) {
      maintain();
    }
    return rx.size(); // + buf_available;
  }

  virtual int read(uint8_t *buf, size_t size) {
    YIELD();
    maintain();
    size_t cnt = 0;
    while (cnt < size) {
      size_t chunk = Min(size-cnt, rx.size());
      if (chunk > 0) {
        rx.get(buf, chunk);
        buf += chunk;
        cnt += chunk;
        continue;
      }
      // TODO: Read directly into user buffer?
      maintain();
      /*
      if (buf_available > 0) {
        modemRead(rx.free());
      } else {
        break;
      }
      */
    }
    return cnt;
  }

  virtual int read() {
    uint8_t c;
    if (read(&c, 1) == 1) {
      return c;
    }
    return -1;
  }

  virtual int peek() {
    uint8_t c;
    if (rx.peek(&c) == true) {
      return c;
    }
    return -1;
  }

  virtual void flush() { stream.flush(); }

  virtual uint8_t connected() {
    if (available()) {
      return true;
    }
    return network_joined;
  }

  virtual operator bool() { return connected(); }

public:

  /*
   * Basic functions
   */
  bool begin(_lora_band band, uint32_t baud = 19200, uint16_t config = SERIAL_8N2) {
#ifdef SerialLoRa
    SerialLoRa.begin(baud, config);
    pinMode(LORA_BOOT0, OUTPUT);
    digitalWrite(LORA_BOOT0, LOW);
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
    digitalWrite(LORA_RESET, LOW);
    delay(200);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
#endif
    region = band;
    if (init()) {
        return configureBand(band);
    } else {
      return begin(band, baud, SERIAL_8N1);
    }
    return false;
  }

  bool init() {
    if (!autoBaud()) {
      return false;
    }
    // populate version field on startup
    version();
    if (!isLatestFW()) {
      DBG("Please update fw using MKRWANFWUpdate_standalone.ino sketch");
    }
    return true;
  }

  bool configureClass(_lora_class _class) {
    return setValue(GF("+CLASS="), (char)_class);
  }

  bool configureBand(_lora_band band) {
    sendAT(GF("+BAND="), band);
    if (waitResponse() != 1) {
        return false;
    }
    if (band == EU868 && isArduinoFW()) {
        return dutyCycle(true);
    }
    return true;
  }

  int getChannelMaskSize(_lora_band band) {
    switch (band)
    {
      case AS923:
      case CN779:
      case EU433:
      case EU868:
      case KR920:
      case IN865:
        mask_size = 1;
        break;
      case AU915:
      case CN470:
      case US915:
      case US915_HYBRID:
        mask_size = 6;
        break;
      default:
        break;
    }
    return mask_size;
  }

  String getChannelMask() {
    int size = 4*getChannelMaskSize(region);
    sendAT(GF("+CHANMASK?"));
    if (waitResponse("+OK=") == 1) {
        channel_mask_str = stream.readStringUntil('\r');
        DBG("Full channel mask string: ", channel_mask_str);
        sscanf(channel_mask_str.c_str(), "%04hx%04hx%04hx%04hx%04hx%04hx", &channelsMask[0], &channelsMask[1], &channelsMask[2],
                                                    &channelsMask[3], &channelsMask[4], &channelsMask[5]);

        return channel_mask_str.substring(0, size);
    }
    String str = "0";
    return str;
  }

  int isChannelEnabled(int pos) {
    //Populate channelsMask array
    int max_retry = 3;
    int retry = 0;
    while (retry < max_retry) {
      String mask = getChannelMask();
      if (mask != "0") {
        break;
      }
      retry++;
    }

    int row = pos / 16;
    int col = pos % 16;
    uint16_t channel = (uint16_t)(1 << col);

    channel = ((channelsMask[row] & channel) >> col);

    return channel;
  }

  bool disableChannel(int pos) {
    //Populate channelsMask array
    int max_retry = 3;
    int retry = 0;
    while (retry < max_retry) {
      String mask = getChannelMask();
      if (mask != "0") {
        break;
      }
      retry++;
    }

    int row = pos / 16;
    int col = pos % 16;
    uint16_t mask = ~(uint16_t)(1 << col);

    channelsMask[row] = channelsMask[row] & mask;

    return sendMask();
  }

  bool enableChannel(int pos) {
    //Populate channelsMask array
    int max_retry = 3;
    int retry = 0;
    while (retry < max_retry) {
      String mask = getChannelMask();
      if (mask != "0") {
        break;
      }
      retry++;
    }

    int row = pos / 16;
    int col = pos % 16;
    uint16_t mask = (uint16_t)(1 << col);

    channelsMask[row] = channelsMask[row] | mask;

    return sendMask();
  }

  bool sendMask() {
    String newMask;

    /* Convert channel mask into string */
    for (int i = 0; i < 6; i++) {
      char hex[5];
      sprintf(hex, "%04x", channelsMask[i]);
      newMask.concat(hex);
    }

    DBG("Newmask: ", newMask);

    return sendMask(newMask);
  }

  bool sendMask(String newMask) {
    return setValue(GF("+CHANMASK="), newMask);
  }

  void setBaud(unsigned long baud) {
    sendAT(GF("+UART="), baud);
  }

  bool autoBaud(unsigned long timeout = 10000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      sendAT(GF(""));
      if (waitResponse(200) == 1) {
          delay(100);
          return true;
      }
      delay(100);
    }
    return false;
  }

  String version() {
    sendAT(GF("+DEV?"));
    if (waitResponse("+OK=") == 1) {
        fw_version = stream.readStringUntil('\r');
    }
    sendAT(GF("+VER?"));
    if (waitResponse("+OK=") == 1) {
        fw_version += " " + stream.readStringUntil('\r');
    }

    return fw_version;
  }

  String deviceEUI() {
    return getStringValue(GF("+DEVEUI?"));
  }

  void maintain() {
    while (stream.available()) {
      waitResponse(100);
    }
  }

  void minPollInterval(unsigned long secs) {
    pollInterval = secs * 1000;
  }

  void poll() {
    if (millis() - lastPollTime < pollInterval) return;
    lastPollTime = millis();
    // simply trigger a fake write
    uint8_t dummy = 0;
    modemSend(&dummy, 1, true);
  }

  bool factoryDefault() {
    sendAT(GF("+FACNEW"));  // Factory
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */

  bool restart() {
    if (!autoBaud()) {
      return false;
    }
    sendAT(GF("+REBOOT"));
    if (waitResponse(10000L, "+EVENT=0,0\r") != 1) {
      return false;
    }
    // Second answer, always returns +OK on new firmware versions
    (void)streamSkipUntil('\r', 0);
    delay(1000);
    return init();
  }

  bool power(_rf_mode mode, uint8_t transmitPower) { // transmitPower can be between 0 and 5
    sendAT(GF("+RFPOWER="), mode,",",transmitPower);
    if (waitResponse() != 1)
      return false;
    String resp = stream.readStringUntil('\r');
    return true;
  }

  int32_t getPower() {
    return getIntValue(GF("+RFPOWER?"));
  }

#ifdef SerialLoRa
  // Sends the modem into dumb mode, so the Semtech chip can be controlled directly
  // The only way to exit this mode is through a begin()
  void dumb() {
	SerialLoRa.end();
	pinMode(LORA_IRQ_DUMB, OUTPUT);
	digitalWrite(LORA_IRQ_DUMB, LOW);

	// Hardware reset
	pinMode(LORA_BOOT0, OUTPUT);
	digitalWrite(LORA_BOOT0, LOW);
	pinMode(LORA_RESET, OUTPUT);
	digitalWrite(LORA_RESET, HIGH);
	delay(200);
	digitalWrite(LORA_RESET, LOW);
	delay(200);
	digitalWrite(LORA_RESET, HIGH);
	delay(50);

	// You can now use SPI1 and LORA_IRQ_DUMB as CS to interface with the chip
  }
#endif

  bool dutyCycle(bool on) {
    return setValue(GF("+DUTYCYCLE="), on);
  }

  bool setPort(uint8_t port) {
    return setValue(GF("+PORT="), port);
  }

  bool publicNetwork(bool publicNetwork) {
    return setValue(GF("+NWK="), publicNetwork);
  }

  bool sleep(bool on = true) {
    return setValue(GF("+SLEEP="), on);
  }

  bool format(bool hexMode) {
    return setValue(GF("+DFORMAT="), hexMode);
  }

/*
	DataRate 	Modulation 	SF 	BW 	bit/s
	0 	LoRa 	12 	125 	250
	1 	LoRa 	11 	125 	440
	2 	LoRa 	10 	125 	980
	3 	LoRa 	9 	125 	1'760
	4 	LoRa 	8 	125 	3'125
	5 	LoRa 	7 	125 	5'470
	6 	LoRa 	7 	250 	11'000
*/

  bool dataRate(uint8_t dr) {
    return setValue(GF("+DR="), dr);
  }

  int getDataRate() {
    return (int)getIntValue(GF("+DR?"));
  }

  bool setADR(bool adr) {
    return setValue(GF("+ADR="), adr);
  }

  int getADR() {
    return (int)getIntValue(GF("+ADR?"));
  }

  String getDevAddr() {
    return getStringValue(GF("+DEVADDR?"));
  }

  String getNwkSKey() {
    return getStringValue(GF("+NWKSKEY?"));
  }

  String getAppSKey() {
    return getStringValue(GF("+APPSKEY?"));
  }

  int getRX2DR() {
    return (int)getIntValue(GF("+RX2DR?"));
  }

  bool setRX2DR(uint8_t dr) {
    return setValue(GF("+RX2DR="),dr);
  }

  int32_t getRX2Freq() {
    return getIntValue(GF("+RX2FQ?"));
  }

  bool setRX2Freq(uint32_t freq) {
    return setValue(GF("+RX2FQ="),freq);
  }

  bool setFCU(uint16_t fcu) {
    return setValue(GF("+FCU="), fcu);
  }

  int32_t getFCU() {
    return getIntValue(GF("+FCU?"));
  }

  bool setFCD(uint16_t fcd) {
    return setValue(GF("+FCD="), fcd);
  }

  int32_t getFCD() {
    return getIntValue(GF("+FCD?"));
  }

  int32_t getRSSI() {
    return getIntValue(GF("+RSSI?"));
  }

  int32_t getSNR() {
    return getIntValue(GF("+SNR?"));
  }

private:

  bool isArduinoFW() {
    return (fw_version.indexOf(ARDUINO_FW_IDENTIFIER) >= 0);
  }

  bool isLatestFW() {
    return (fw_version == ARDUINO_FW_VERSION);
  }

  bool changeMode(_lora_mode mode) {
    return setValue(GF("+MODE="), mode);
  }

  bool join(uint32_t timeout) {
    sendAT(GF("+JOIN"));
    sendAT();
    if (waitResponse(timeout, "+EVENT=1,1\r") != 1) {
      return false;
    }
    // Second answer, always returns +OK on new firmware versions
    (void)streamSkipUntil('\r', 0L);
    return true;
  }

  bool set(_lora_property prop, const char* value) {
    switch (prop) {
        case APP_EUI:
            sendAT(GF("+APPEUI="), value);
            break;
        case APP_KEY:
            sendAT(GF("+APPKEY="), value);
            break;
        case DEV_EUI:
            sendAT(GF("+DEVEUI="), value);
            break;
        case DEV_ADDR:
            sendAT(GF("+DEVADDR="), value);
            break;
        case NWKS_KEY:
            sendAT(GF("+NWKSKEY="), value);
            break;
        case NWK_ID:
            sendAT(GF("+IDNWK="), value);
            break;
        case APPS_KEY:
            sendAT(GF("+APPSKEY="), value);
            break;
        default:
            return false;
    }
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  /**
   * @brief transmit uplink
   * 
   * @param buff data to transmit
   * @param len length of the buffer
   * @param confirmed true = transmit confirmed uplink
   * @return int a positive number indicate success and is the number of bytes transmitted
   *             -1 indicates a timeout error
   *             -2 indicates LORA_ERROR
   *             -3 indicates LORA_ERROR_PARAM
   *             -4 indicates LORA_ERROR_BUSY
   *             -5 indicates LORA_ERROR_OVERFLOW
   *             -6 indicates LORA_ERROR_NO_NETWORK
   *             -7 indicates LORA_ERROR_RX
   *             -8 indicates LORA_ERROR_UNKNOWN
   *             -20 packet exceeds max length
   *             
   */
  int modemSend(const void* buff, size_t len, bool confirmed) {

    size_t max_len = modemGetMaxSize();
    if (len > max_len) {
        return -20;
    }

    if (confirmed) {
        sendAT(GF("+CTX "), len);
    } else {
        sendAT(GF("+UTX "), len);
    }

    stream.write((uint8_t*)buff, len);

    int8_t rc = waitResponse();
    if (rc == 1) {            ///< OK
      return len;
    } else if ( rc > 1 ) {    ///< LORA ERROR
      return -rc;
    } else {                  ///< timeout
      return -1;
    }
  }

  size_t modemGetMaxSize() {
    if (isArduinoFW()) {
      return 64;
    }
    sendAT(GF("+MSIZE?"));
    if (waitResponse(2000L, "+OK=") != 1) {
      return 0;
    }
    return stream.readStringUntil('\r').toInt();
  }

  bool getJoinStatus() {
    return (getIntValue(GF("+NJS?")));
  }

  /* Utilities */
  template<typename T>
  void streamWrite(T last) {
    stream.print(last);
  }

  template<typename T, typename... Args>
  void streamWrite(T head, Args... tail) {
    stream.print(head);
    streamWrite(tail...);
  }

  int streamRead() { return stream.read(); }

  bool streamSkipUntil(char c, unsigned long timeout = 1000L) {
    unsigned long startMillis = millis();
	do {
	  if (stream.available()) {
		if (stream.read() == c)
	      return true;
	  }
    } while (millis() - startMillis < timeout);
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., LORA_NL);
    stream.flush();
    YIELD();
    DBG("### AT:", cmd...);
  }

  /**
   * @brief wait for a response from the modem.
   * 
   * @param timeout the time in milliseconds to wait for a response
   * @param data a string containing the response to wait for
   * @param r1 response string defaults to LORA_OK
   * @param r2 response string defaults to LORA_ERROR
   * @param r3 response string defaults to LORA_ERROR_PARAM
   * @param r4 response string defaults to LORA_ERROR_BUSY
   * @param r5 response string defaults to LORA_ERROR_OVERFLOW
   * @param r6 response string defaults to LORA_ERROR_NO_NETWORK
   * @param r7 response string defaults to LORA_ERROR_RX
   * @param r8 response string defaults to LORA_ERROR_UNKNOWN
   * @return int8_t   n if the response = r<n>
   *                  99 if the response ends with "+RECV="
   *                  -1 if timeout
   */
  int8_t waitResponse(uint32_t timeout, String& data,
                       ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=GFP(LORA_ERROR_PARAM), ConstStr r4=GFP(LORA_ERROR_BUSY), ConstStr r5=GFP(LORA_ERROR_OVERFLOW),
                       ConstStr r6=GFP(LORA_ERROR_NO_NETWORK), ConstStr r7=GFP(LORA_ERROR_RX), ConstStr r8=GFP(LORA_ERROR_UNKNOWN))
  {
    data.reserve(64);
    int8_t index = -1;
    int length = 0;
    unsigned long startMillis = millis();
    do {
      YIELD();
      while (stream.available() > 0) {
        int a = streamRead();
        if (a < 0) continue;
        data += (char)a;
        length++;
        if (length >= 64){
        	DBG("### Data string too long:", data);
        	return index;
        }
        if ((a > '0' && a != '=')  ||
        	(a != '\r' && a != ':' && a != ' '))
        	continue;				// continue receive until terminator
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (r6 && data.endsWith(r6)) {
          index = 6;
          goto finish;
        } else if (r7 && data.endsWith(r7)) {
          index = 7;
          goto finish;
        } else if (r8 && data.endsWith(r8)) {
          index = 8;
          goto finish;
        } else if (data.endsWith("+RECV=")) {
          data = "";
          (void)stream.readStringUntil(',').toInt();
          length = stream.readStringUntil('\r').toInt();
          (void)streamSkipUntil('\n');
          (void)streamSkipUntil('\n');
          for (int i = 0; i < length;) {
            if (stream.available()) {
                rx.put(stream.read());
                i++;
            }
          }
        }
      }
    } while (millis() - startMillis < timeout);
finish:
    if (!index) {
      data.trim();
      if (data.length()) {
        DBG("### Unhandled:", data);
      }
      data = "";
    }
    return index;
  }

  int8_t waitResponse(uint32_t timeout,
                       ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=GFP(LORA_ERROR_PARAM), ConstStr r4=GFP(LORA_ERROR_BUSY), ConstStr r5=GFP(LORA_ERROR_OVERFLOW),
                       ConstStr r6=GFP(LORA_ERROR_NO_NETWORK), ConstStr r7=GFP(LORA_ERROR_RX), ConstStr r8=GFP(LORA_ERROR_UNKNOWN))
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4, r5, r6, r7, r8);
  }

  int8_t waitResponse(ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
					  ConstStr r3=GFP(LORA_ERROR_PARAM), ConstStr r4=GFP(LORA_ERROR_BUSY), ConstStr r5=GFP(LORA_ERROR_OVERFLOW),
					  ConstStr r6=GFP(LORA_ERROR_NO_NETWORK), ConstStr r7=GFP(LORA_ERROR_RX), ConstStr r8=GFP(LORA_ERROR_UNKNOWN))
  {
    return waitResponse(1000, r1, r2, r3, r4, r5, r6, r7, r8);
  }

  String getStringValue(ConstStr cmd){
	String value = "";
	sendAT(cmd);
	if (waitResponse("+OK=") == 1) {
		value = stream.readStringUntil('\r');
	}
	return value;
  }

  int32_t getIntValue(ConstStr cmd){
	int32_t value = -1;
	sendAT(cmd);
	if (waitResponse("+OK=") == 1) {
		value = stream.readStringUntil('\r').toInt();
	}
	return value;
  }

  template<typename T, typename U>
  bool setValue(T cmd, U value) {
	sendAT(cmd, value);
	return (waitResponse() == 1);
  }

};
