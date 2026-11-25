#pragma once
// Gripper 1 solenoid: ON = solenoid dialiri (menjepit / sesuai pemasangan), OFF = dilepas. Hanya dipanggil dari task kontrol (Core 1).
void gripperBegin();            // pin sebagai keluaran, awal OFF
void gripperSet(bool on);
bool gripperOn();
