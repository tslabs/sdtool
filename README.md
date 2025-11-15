# SD Card info and erase tool

<img src="img\image_2.png" />

### Make adapter:

| SD SPI | RP2040 GPIO | Wire color (photo) |
| ------ | ----------- | ------------------ |
| SCK    | 2           | yellow             |
| MOSI   | 3           | brown              |
| MISO   | 4           | blue               |
| CSn    | 5           | white              |
| GND    | -           | black              |
| 3.3V   | -           | red                |

Below is a photo of SD-microSD adapter used to connect to RP2040 Raspberry Pi Pico board to be used with microSD cards.

<img src="img\image_0.jpg" />

<img src="img\image_1.jpg" />


---

### Install ZephyrSDK and 'zephyrproject' workspace.

Ubuntu WSL (or any other Linux) is fine.

(*Google it*)

---

### Put 'sdtool' folder inside 'zephyrproject' dir.

---

### Fix Zephyr OS SDMMC driver:

In `zephyr\drivers\sdhc\sdhc_spi.c` find `sdhc_spi_wait_unbusy` and remove `static`.

---

### Build:

`west build -p auto -o=-j20 -b rpi_pico`

or make an alias:
`alias b='clear && west build -p auto -o=-j20 -b rpi_pico'`

---

### Flash:

- Press 'RST' and 'BOOT' buttons. Release 'RST' while keeping 'BOOT' pressed.
- Copy `bin/sdtool.uf2` to virtual disk.

---

### Connect serial terminal using RP2040 USB port as virtual COM.

---

### Usage:

**help** - print available shell commands.

**info** - print sd card decoded info.

**erase** - erase (trim) all sectors on sd card. **No confirmation and irreversible!**

Tab key works for commands auto-completion.

---

<img src="img\sticker.png" />
