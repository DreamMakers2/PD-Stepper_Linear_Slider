# Wi-Fi configuration

Edit [`include/wifi_config.h`](include/wifi_config.h) with the Wi-Fi network used by the external controller:

```cpp
#define SLIDER_WIFI_SSID "your-wifi-name"
#define SLIDER_WIFI_PASSWORD "your-wifi-password"
#define SLIDER_FALLBACK_AP_PASSWORD "a-password-of-at-least-8-characters"
```

`include/wifi_config.h` is present locally but intentionally listed in `.gitignore`, so credentials are not committed. The tracked [`include/wifi_config.example.h`](include/wifi_config.example.h) is the template to copy if the local file is missing:

```sh
cp include/wifi_config.example.h include/wifi_config.h
```

At boot the controller tries station mode for 10 seconds. If the SSID is left as `your-wifi-name` or the connection fails, it creates a password-protected `PD-Stepper-xxxx` access point. Its API address is `http://192.168.4.1`; both station and fallback addresses are printed to the PlatformIO serial monitor.
