# Kebalikan extract_monitor.py: tools/monitor.html -> include/monitor_html.h (dashboard yang dilayani robot).
# Jalankan setelah mengedit tools/monitor.html:  python tools/pack_monitor.py
import pathlib
root = pathlib.Path(__file__).resolve().parent
html = (root / "monitor.html").read_text(encoding="utf-8")
assert ')HTML"' not in html
hdr = ("#pragma once\n#include <pgmspace.h>\n"
       "// Dashboard Omni-3X. HTTP: http://192.168.4.1/  WebSocket: ws://192.168.4.1:81/ws  Kabel: Web Serial (lihat tools/monitor.html)\n"
       "// Tanpa CDN / font eksternal: AP robot tidak punya internet.\n// File ini dibuat dari tools/monitor.html: python tools/pack_monitor.py\n")
(root.parent / "include" / "monitor_html.h").write_text(hdr + 'const char MONITOR_HTML[] PROGMEM = R"HTML(' + html + ')HTML";\n', encoding="utf-8")
print("include/monitor_html.h dibuat")
