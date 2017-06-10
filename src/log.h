#ifdef SERIAL_OUTPUT
#define LOG(LEVEL, MSG) { \
  syslog.log((LEVEL),(MSG)); \
  Serial.println((MSG)); \
}
#else
#define LOG(LEVEL, MSG) { \
  syslog.log((LEVEL),(MSG)); \
}
#endif

#ifdef SERIAL_OUTPUT
#define LOGF(LEVEL, MSG, ...) { \
  syslog.logf((uint16_t)(LEVEL),(MSG),  ## __VA_ARGS__);  \
  Serial.printf((MSG),  ## __VA_ARGS__); \
}
#else
#define LOGF(LEVEL, MSG, ...) { \
  syslog.logf((LEVEL), (MSG),  ## __VA_ARGS__); \
}
#endif
