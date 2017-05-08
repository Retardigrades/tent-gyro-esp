#include <ESP8266httpUpdate.h>

#define UPDATE(host, port, endpoint)                                    \
  do {                                                                  \
    t_httpUpdate_return ret = ESPhttpUpdate.update(                     \
        host, port, endpoint, "TENT_VERSION::" __DATE__ "::" __TIME__); \
    switch (ret) {                                                      \
      case HTTP_UPDATE_FAILED:                                          \
        Serial.printf("update_failed (%s): %s\n",                       \
                      ESPhttpUpdate.getLastError(),                     \
                      ESPhttpUpdate.getLastErrorString().c_str());      \
        \
break;                                                                  \
        \
case HTTP_UPDATE_NO_UPDATES : Serial.println("no update");              \
        break;                                                          \
        \
case HTTP_UPDATE_OK : Serial.println("update ok");                      \
        break;                                                          \
    \
};                                                                      \
  \
\
}                                                               \
  \
while(0)
