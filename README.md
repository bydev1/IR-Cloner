# IR-CLoner (ESP32-S3 & FPGA Cyclone IV)

"One Screen to Replace All Remotes" — กล่องรีโมทอัจฉริยะอเนกประสงค์ที่แก้ปัญหาความไม่เสถียรของรีโมท IoT ทั่วไป ด้วยการแยกสถาปัตยกรรมระหว่างการประมวลผลเครือข่ายและการควบคุมเวลาแบบฮาร์ดแวร์ (Hardware Offloading)

---

## 🛠️ System Architecture (ภาพรวมสถาปัตยกรรมระบบ)
* **ESP32-S3 (Master):** ทำหน้าที่จัดการระบบ Wi-Fi (Access Point Mode ที่ `192.168.4.1`), รัน HTTP Web Server, จัดการระบบไฟล์หน่วยความจำถาวร (LittleFS Flash), และส่งคำสั่งควบคุมผ่านบัส SPI
* **FPGA Cyclone IV / EP4CE6E22 (Slave):** ทำหน้าที่เป็นฮาร์ดแวร์ไทม์เมอร์ความละเอียดสูงด้วยสัญญาณนาฬิกา 50 MHz จับเวลาพัลส์ระดับ $1\ \mu\text{s}$, กรองสัญญาณรบกวน (Glitch Filter), เก็บข้อมูลใน M9K Block RAM, และสร้างคลื่นพาหะ 38 kHz ผสมด้วย AND Gate เพื่อขับหลอด IR ผ่านทรานซิสเตอร์ 2N2222A

---

## 📂 Project File Structure & Descriptions (โครงสร้างและหน้าที่ของแต่ละไฟล์)

### 1. ฝั่ง FPGA (Verilog HDL & Quartus)
* **`top.v`**
  * **หน้าที่:** Top-level hardware module ทำหน้าที่เชื่อมโยงโมดูล `spi_slave` เข้ากับ `ir_core`, แมปสัญญาณบัสภายในเข้าสู่ขาพินฮาร์ดแวร์จริง, ควบคุมการซิงโครไนซ์สัญญาณรีเซ็ต, และขับไฟสถานะออนบอร์ด `LED[3:0]` (รวมถึงไฟ Heartbeat 1 Hz)
* **`ir_core.v`**
  * **หน้าที่:** หัวใจหลักของระบบ (Core Transceiver) ประกอบด้วยตัวสร้างจังหวะ 1 µs จาก Clock 50 MHz, วงจรกรองสัญญาณรบกวนสมมาตร (Symmetric Glitch Filter 100 µs), ตัวจัดการ M9K Block RAM (1,024 Words), วงจรตัดจบเฟรม Timeout (60 ms), และวงจรผสมคลื่นพาหะ 38 kHz แบบ Zero-Runt พร้อมระบบล็อกภาครับอัตโนมัติ (RX Holdoff) ตอนกำลังยิง
* **`spi_slave.v`**
  * **หน้าที่:** ตัวถอดรหัสโปรโตคอล SPI (SPI Protocol Decoder) ทำหน้าที่รับ-ส่งข้อมูลกับ ESP32-S3 รองรับ 7 คำสั่งหลัก (`0x01` ถึง `0x07`) เพื่อสั่งการโหมดรับ (RX), โหมดส่ง (TX), และอ่าน/เขียนข้อมูลพัลส์ใน M9K RAM
* **`top.qsf`**
  * **หน้าที่:** ไฟล์กำหนดค่าโปรเจกต์ของ Intel Quartus Prime สำหรับระบุตำแหน่งการเชื่อมต่อขาพินฮาร์ดแวร์ (Pin Assignments) ของชิป FPGA (เช่น ขารับแสง `PIN_115`, ขายิงแสง `PIN_119`, และขา Clock 50 MHz `PIN_23`)

---

### 2. ฝั่ง ESP32-S3 (C++ / Arduino IDE)
* **`ir_transceiver.ino`**
  * **หน้าที่:** ไฟล์หลัก (Main Entry Point) ทำหน้าที่ตั้งค่าเริ่มต้นระบบ (Setup) และลูปการทำงานหลัก (Loop), เริ่มต้นใช้งานระบบไฟล์ LittleFS, ตั้งค่าบัส SPI, เปิดใช้งาน Wi-Fi Access Point, และสั่งรัน HTTP Web Server
* **`spi_master.h` & `spi_master.cpp`**
  * **หน้าที่:** ชุดไดรเวอร์สั่งการ SPI (SPI Master Driver) ทำหน้าที่แปลงคำสั่งควบคุมเป็นชุดข้อมูลไบต์เพื่อส่งคำสั่ง `0x01`–`0x07` ผ่านสาย SPI ไปยัง FPGA พร้อมฟังก์ชันอ่าน-เขียนอาร์เรย์ความกว้างพัลส์
* **`web_server.h` & `web_server.cpp`**
  * **หน้าที่:** โมดูลจัดการเว็บเซิร์ฟเวอร์และระบบจัดเก็บข้อมูล (Web API & Storage Worker) ทำหน้าที่รับ HTTP Requests จากหน้าเว็บ, ประมวลผลปุ่มกดของผู้ใช้ (`Capture`, `Save`, `Send`, `Delete`), และทำหน้าที่อ่าน/เขียนไฟล์โปรไฟล์รหัสแอร์ลงในหน่วยความจำ Flash ผ่านระบบ LittleFS
* **`data/` (LittleFS Data Folder)**
  * **`index.html`:** ไฟล์หน้าเว็บส่วนติดต่อผู้ใช้ (Frontend UI) ที่อัปโหลดเก็บไว้ใน Flash ของ ESP32 ทำหน้าที่แสดงผลปุ่มกด ควบคุมโหมดรับ/ส่ง, วาดกราฟแสดงรูปคลื่นสัญญาณพัลส์แบบเรียลไทม์ด้วย HTML/JS, และสื่อสารผ่าน Fetch API

---

## 🔌 Hardware Pin Connections (การเชื่อมต่อฮาร์ดแวร์)

| ESP32-S3 Pin | SPI Bus Role | FPGA Pin | Description |
| :--- | :--- | :--- | :--- |
| **GPIO 10** | MISO | **PIN 110** | ส่งข้อมูลย้อนกลับจาก FPGA มายัง ESP32 |
| **GPIO 12** | SCK | **PIN 111** | สัญญาณนาฬิกา SPI Clock |
| **GPIO 13** | CS (Chip Select) | **PIN 112** | สัญญาณเลือกชิป (Active-Low) |
| **GPIO 11** | MOSI | **PIN 113** | ส่งคำสั่งและข้อมูลจาก ESP32 ไปยัง FPGA |
| **GND** | Common GND | **GND** | ใช้กราวด์ร่วมกันระหว่างบอร์ดทั้งหมดเพื่อความเสถียรของลอจิก |

* **วงจร IR:** 
  * ภาครับ: โมดูล KY-022 ต่อขาออกเข้าสู่ FPGA **`PIN_115`** (`ir_rx_n`)
  * ภาคส่ง: FPGA ส่งสัญญาณผ่าน **`PIN_119`** (`ir_tx`) ขับผ่านตัวต้านทานและทรานซิสเตอร์ NPN **2N2222A** ออกหลอด IR LED

---

## 🚀 Quick Start
1. **FPGA:** เปิดโปรเจกต์ใน Intel Quartus Prime, โหลดไฟล์การตั้งค่าพินจาก `top.qsf`, ทำการ Compilation แล้วเบิร์นไฟล์ `.sof` ลงบอร์ด Cyclone IV
2. **ESP32-S3:** ติดตั้งบอร์ด ESP32 ใน Arduino IDE, อัปโหลด LittleFS Data Folder (`data/index.html`) ลงใน Flash, จากนั้นคอมไพล์และอัปโหลดโค้ด `ir_transceiver.ino`
3. **Usage:** เชื่อมต่อ Wi-Fi ของบอร์ด (Access Point) แล้วเปิดเบราว์เซอร์ไปที่ `http://192.168.4.1` เพื่อใช้งาน
