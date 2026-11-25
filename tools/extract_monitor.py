# Ambil HTML dari include/monitor_html.h -> tools/monitor.html (untuk dibuka dari PC, jalur kabel via Web Serial).
# Jalankan lagi setiap kali monitor_html.h diubah:  python tools/extract_monitor.py
import pathlib
root = pathlib.Path(__file__).resolve().parent
src = (root.parent / "include" / "monitor_html.h").read_text(encoding="utf-8")
html = src.split('R"HTML(', 1)[1].rsplit(')HTML"', 1)[0]
(root / "monitor.html").write_text(html, encoding="utf-8")
print("tools/monitor.html dibuat")
