*
 * ShackSwitch_R4_1x4.ino  —  G0JKN ShackSwitch  —  1×4 single-radio antenna switch
 * Arduino Uno R4 WiFi
 *
 * Nigel Fenton  G0JKN  —  https://github.com/nigelfenton/shackswitch
 * MIT Licence
 *
 * Hardware
 *   Relays : D2–D5  (one active at a time, all others deactivated)
 *   Display: built-in 12×8 LED matrix  (shows active port digit 1–4)
 *
 * Network
 *   HTTP  port 8080   — web UI + REST API
 *   TCP   port 9007 — 4O3A Antenna Genius protocol (AetherSDR)
 *   UDP   port 9007 — discovery beacon every 5 s
 *
 * REST endpoints
 *   GET /                                  — status page
 *   GET /settings                          — settings page
 *   GET /status                            — JSON status
 *   GET /select?port=N                     — select antenna 1–4  (0 = all off)
 *   GET /setband?band=X [&input=1]         — auto-select from band map
 *   GET /kk1l/setband?input=1&band=X       — SmartSDR-compatible alias
 *   GET /names?n1=X&n2=X&n3=X&n4=X        — save all antenna names
 *   GET /bandmap?band=X&port=N             — assign band to port  (port=0 clears)
 *   GET /wifi?ssid=X&pass=Y               — save WiFi credentials + reboot
 *   GET /reset                             — factory reset
 *
 * ── EDIT BEFORE FIRST FLASH ───────────────────────────────────────────────────
 * Change DEFAULT_SSID and DEFAULT_PASSWORD to match your network.
 * These are written to EEPROM on a clean first boot only.
 * You can also update them later via /settings in the web UI.
 */
