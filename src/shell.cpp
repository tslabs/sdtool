
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/sd/sd.h>
#include <zephyr/sd/sd_spec.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include "types.h"

extern "C" int sdhc_spi_wait_unbusy(...);

LOG_MODULE_REGISTER(shell);

extern "C" { struct disk_info *disk_access_get_di(const char *name); }

static const char *disk_pdrv = "SD";
const shell *sh = NULL;

// ----- Zephyr OS declarations (will definitely break on SDK update)

enum sd_status
{
  SD_UNINIT,
  SD_ERROR,
  SD_OK,
};

struct sdmmc_data
{
  struct sd_card card;
  enum sd_status status;
  char *name;
};

struct sdmmc_config
{
  const struct device *host_controller;
};

// ----- Functions

void dump(u8 *buf, int n, char c = 0)
{
  for (int i = 0; i < n; i++)
    c ? shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "%02X%c", buf[i], c) : shell_fprintf(sh, SHELL_INFO, "%02X", buf[i]);

  shell_print(sh, "");
}

void make_raw_cxd(const u8 *buf, u32 *raw)  // CSD/CID helper
{
  for (int i = 0; i < 4; i++)
  {
    u32 v = ((u32)buf[i * 4 + 0] << 24) |
            ((u32)buf[i * 4 + 1] << 16) |
            ((u32)buf[i * 4 + 2] << 8)  |
            ((u32)buf[i * 4 + 3] << 0);
    raw[3 - i] = v;  // word order as used by Zephyr sdmmc.c
  }
}

void make_raw32(const u8 *buf, u32 *raw, int words)  // big-endian → host
{
  for (int i = 0; i < words; i++)
  {
    raw[i] = ((u32)buf[i * 4 + 0] << 24) |
             ((u32)buf[i * 4 + 1] << 16) |
             ((u32)buf[i * 4 + 2] << 8)  |
             ((u32)buf[i * 4 + 3] << 0);
  }
}

// Local CSD/CID decoders — app-private, no linkage to Zephyr internals.

void sdmmc_decode_csd(sd_csd *csd,
                          const uint32_t raw[4],
                          uint32_t *blk_count,
                          uint16_t *blk_size)
{
  // Clear flags / struct first
  memset(csd, 0, sizeof(*csd));

  // CSD structure + timing
  csd->csd_structure = (raw[3] >> 30) & 0x3;
  csd->read_time1    = (raw[3] >> 16) & 0xFF;
  csd->read_time2    = (raw[3] >>  8) & 0xFF;
  csd->xfer_rate     =  raw[3]        & 0xFF;

  // Command classes + read block length
  csd->cmd_class   = (raw[2] >> 20) & 0xFFF;
  csd->read_blk_len = (raw[2] >> 16) & 0xF;

  // Flags (see sd_csd_flag in sd_spec.h)
  if (raw[2] & 0x8000) csd->flags |= SD_CSD_READ_BLK_PARTIAL_FLAG;
  if (raw[2] & 0x4000) csd->flags |= SD_CSD_WRITE_BLK_MISALIGN_FLAG;
  if (raw[2] & 0x2000) csd->flags |= SD_CSD_READ_BLK_MISALIGN_FLAG;
  if (raw[2] & 0x1000) csd->flags |= SD_CSD_DSR_IMPLEMENTED_FLAG;

  uint32_t tmp_blk_count = 0;
  uint16_t tmp_blk_size  = 512;

  if (csd->csd_structure == 0)
  {
    // SDSC (v1.x) style capacity
    csd->device_size =
      ((raw[2] & 0x3FF) << 2) |
      ((raw[1] >> 30) & 0x3);

    csd->read_current_min  = (raw[1] >> 27) & 0x7;
    csd->read_current_max  = (raw[1] >> 24) & 0x7;
    csd->write_current_min = (raw[1] >> 20) & 0x7;
    csd->write_current_max = (raw[1] >> 18) & 0x7;
    csd->dev_size_mul      = (raw[1] >> 15) & 0x7;

    tmp_blk_count =
      (csd->device_size + 1u) << (csd->dev_size_mul + 2u);
    tmp_blk_size = 1u << csd->read_blk_len;

    if (tmp_blk_size != SDMMC_DEFAULT_BLOCK_SIZE)
    {
      tmp_blk_count = (tmp_blk_count * tmp_blk_size) /
                      SDMMC_DEFAULT_BLOCK_SIZE;
      tmp_blk_size = SDMMC_DEFAULT_BLOCK_SIZE;
    }
  }
  else if (csd->csd_structure == 1)
  {
    // SDHC/SDXC (v2.0+) style capacity
    csd->device_size =
      ((raw[2] & 0x3F) << 16) |
      ((raw[1] >> 16) & 0xFFFF);

    tmp_blk_size  = SDMMC_DEFAULT_BLOCK_SIZE;
    tmp_blk_count = (csd->device_size + 1u) * 1024u;
  }

  // Remaining flags
  if (raw[1] & 0x4000) csd->flags |= SD_CSD_ERASE_BLK_EN_FLAG;
  csd->erase_size       = (raw[1] >> 7) & 0x7F;
  csd->write_prtect_size =  raw[1]       & 0x7F;

  csd->write_speed_factor = (raw[0] >> 26) & 0x7;
  csd->write_blk_len      = (raw[0] >> 22) & 0xF;

  if (raw[0] & 0x200000) csd->flags |= SD_CSD_WRITE_BLK_PARTIAL_FLAG;
  if (raw[0] & 0x8000)   csd->flags |= SD_CSD_FILE_FMT_GRP_FLAG;
  if (raw[0] & 0x4000)   csd->flags |= SD_CSD_COPY_FLAG;
  if (raw[0] & 0x2000)   csd->flags |= SD_CSD_PERMANENT_WRITE_PROTECT_FLAG;
  if (raw[0] & 0x1000)   csd->flags |= SD_CSD_TMP_WRITE_PROTECT_FLAG;

  csd->file_fmt = (raw[0] >> 10) & 0x3;

  if (blk_count) *blk_count = tmp_blk_count;
  if (blk_size)  *blk_size  = tmp_blk_size;
}

void sdmmc_decode_cid(sd_cid *cid, const uint32_t raw[4])
{
  memset(cid, 0, sizeof(*cid));

  cid->manufacturer = (raw[3] >> 24) & 0xFF;
  cid->application  = (raw[3] >>  8) & 0xFFFF;

  cid->name[0] = (raw[3]      ) & 0xFF;
  cid->name[1] = (raw[2] >> 24) & 0xFF;
  cid->name[2] = (raw[2] >> 16) & 0xFF;
  cid->name[3] = (raw[2] >>  8) & 0xFF;
  cid->name[4] = (raw[2]      ) & 0xFF;

  cid->version = (raw[1] >> 24) & 0xFF;

  cid->ser_num  =  (raw[1] & 0x00FFFFFFu) << 8;
  cid->ser_num |= (raw[0] >> 24) & 0xFF;

  cid->date = (raw[0] >> 8) & 0x0FFF;  // year/month packed
}

const char *mid_to_name(uint8_t mid)
{
  switch (mid)
  {
    case 0x00: return "Generic";
    case 0x01: return "Panasonic";
    case 0x02: return "Toshiba / Kioxia";
    case 0x03: return "SanDisk / WD";
    case 0x05: return "Lenovo";
    case 0x06: return "SanDisk Extreme Pro / Sabrent";
    case 0x09: return "ATP";
    case 0x12: return "Patriot";
    case 0x1B: return "Samsung";
    case 0x1D: return "ADATA";
    case 0x27: return "Phison OEM (Delkin, HP, Integral, Kingston, Lexar, PNY, etc.)";
    case 0x28: return "Lexar (Longsys)";
    case 0x31: return "Silicon Power";
    case 0x41: return "Kingston";
    case 0x45: return "TEAMGROUP";
    case 0x56: return "SanDian / various";
    case 0x6F: return "Hiksemi / HP / Kodak / Lenovo / Netac";
    case 0x74: return "Transcend / Gigastone";
    case 0x76: return "PNY / Patriot";
    case 0x82: return "Sony";
    case 0x89: return "Netac / Intel";
    case 0x90: return "Strontium";
    case 0x92: return "Verbatim";
    case 0x9B: return "Patriot";
    case 0x9C: return "Angelbird / Hoodman";
    case 0xB6: return "Delkin Devices";
    default:   return "Unknown";
  }
}

void print_csd_info(const u8 *buf, u32 disk_block_count, u32 disk_block_size)
{
  u32 raw[4];
  make_raw_cxd(buf, raw);

  sd_csd csd{};
  u32 csd_block_count = 0;
  u16 csd_block_size  = 0;

  sdmmc_decode_csd(&csd, raw, &csd_block_count, &csd_block_size);

  const char *csd_struct =
    csd.csd_structure == 0 ? "CSD v1.0 (SDSC)" :
    csd.csd_structure == 1 ? "CSD v2.0 (SDHC/SDXC)" :
                             "Reserved/Unknown";

  u32 read_blk_len_bytes  = 1u << csd.read_blk_len;
  u32 write_blk_len_bytes = 1u << csd.write_blk_len;

  shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "CSD decode:\n");

  shell_fprintf(sh, SHELL_INFO,               "  CSD structure      : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "%u (%s)\n",
                csd.csd_structure, csd_struct);

  shell_fprintf(sh, SHELL_INFO,               "  Max transfer rate  : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "0x%02X (spec units)\n",
                csd.xfer_rate);

  shell_fprintf(sh, SHELL_INFO,               "  Command classes    : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "0x%03X\n",
                csd.cmd_class);

  shell_fprintf(sh, SHELL_INFO,               "  Read block length  : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "2^%u = %u bytes\n",
                csd.read_blk_len, read_blk_len_bytes);

  shell_fprintf(sh, SHELL_INFO,               "  Write block length : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "2^%u = %u bytes\n",
                csd.write_blk_len, write_blk_len_bytes);

  shell_fprintf(sh, SHELL_INFO,               "  Device size field  : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "0x%06X\n",
                csd.device_size);

  shell_fprintf(sh, SHELL_INFO,               "  Erase sector size  : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "%u\n",
                csd.erase_size);

  shell_fprintf(sh, SHELL_INFO,               "  Write protect size : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "%u\n",
                csd.write_prtect_size);

  shell_fprintf(sh, SHELL_INFO,               "  Flags              : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE,  "0x%04X\n",
                csd.flags);

  if (csd.flags & SD_CSD_READ_BLK_PARTIAL_FLAG)
    shell_print(sh, "    - Partial read blocks allowed");
  if (csd.flags & SD_CSD_WRITE_BLK_PARTIAL_FLAG)
    shell_print(sh, "    - Partial write blocks allowed");
  if (csd.flags & SD_CSD_WRITE_BLK_MISALIGN_FLAG)
    shell_print(sh, "    - Write block misalignment");
  if (csd.flags & SD_CSD_READ_BLK_MISALIGN_FLAG)
    shell_print(sh, "    - Read block misalignment");
  if (csd.flags & SD_CSD_DSR_IMPLEMENTED_FLAG)
    shell_print(sh, "    - DSR implemented");
  if (csd.flags & SD_CSD_ERASE_BLK_EN_FLAG)
    shell_print(sh, "    - Single-block erase enabled");
  if (csd.flags & SD_CSD_WRITE_PROTECT_GRP_EN_FLAG)
    shell_print(sh, "    - Write protect group enabled");
  if (csd.flags & SD_CSD_FILE_FMT_GRP_FLAG)
    shell_print(sh, "    - Non-DOS/Windows file format");
  if (csd.flags & SD_CSD_COPY_FLAG)
    shell_print(sh, "    - Content is copy of original");
  if (csd.flags & SD_CSD_PERMANENT_WRITE_PROTECT_FLAG)
    shell_print(sh, "    - Permanently write-protected");
  if (csd.flags & SD_CSD_TMP_WRITE_PROTECT_FLAG)
    shell_print(sh, "    - Temporarily write-protected");

  shell_fprintf(sh, SHELL_INFO,              "  Capacity from CSD  : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u blocks, %u-byte block\n",
                csd_block_count, csd_block_size);

  shell_fprintf(sh, SHELL_INFO,              "  Capacity from disk : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u blocks, %u-byte block\n",
                disk_block_count, disk_block_size);
}

void print_cid_info(const u8 *buf)
{
  u32 raw[4];
  make_raw_cxd(buf, raw);

  sd_cid cid{};
  sdmmc_decode_cid(&cid, raw);

  char pnm[SD_PRODUCT_NAME_BYTES + 1];
  memcpy(pnm, cid.name, SD_PRODUCT_NAME_BYTES);
  pnm[SD_PRODUCT_NAME_BYTES] = 0;

  char oid[3];
  oid[0] = (char)(cid.application >> 8);
  oid[1] = (char)(cid.application & 0xFF);
  oid[2] = 0;

  u8 prv_major = cid.version >> 4;
  u8 prv_minor = cid.version & 0x0F;

  u16 mdt   = cid.date;
  u16 year  = 2000u + (mdt >> 4);
  u8  month = mdt & 0x0F;

  shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "CID decode:\n");

  shell_fprintf(sh, SHELL_INFO,              "  MID (manufacturer) : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%02X (%s)\n",
                cid.manufacturer, mid_to_name(cid.manufacturer));

  shell_fprintf(sh, SHELL_INFO,              "  OID (OEM/app)      : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%s\n",
                oid);

  shell_fprintf(sh, SHELL_INFO,              "  PNM (product)      : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%s\n",
                pnm);

  shell_fprintf(sh, SHELL_INFO,              "  PRV (revision)     : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u.%u\n",
                prv_major, prv_minor);

  shell_fprintf(sh, SHELL_INFO,              "  PSN (serial)       : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%08X\n",
                cid.ser_num);

  shell_fprintf(sh, SHELL_INFO,              "  MDT (date)         : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%04u-%02u\n",
                (unsigned)year, (unsigned)month);
}

void print_scr_info(const u8 *buf)
{
  u32 raw[2];
  make_raw32(buf, raw, 2);

  sd_scr scr{};
  u8 version_flags = 0;

  scr.flags         = 0;
  scr.scr_structure = (raw[0] >> 28) & 0xF;
  scr.sd_spec       = (raw[0] >> 24) & 0xF;

  if (raw[0] & (1u << 23))
    scr.flags |= SD_SCR_DATA_STATUS_AFTER_ERASE;

  scr.sd_sec     = (raw[0] >> 20) & 0x7;
  scr.sd_width   = (raw[0] >> 16) & 0xF;

  if (raw[0] & (1u << 15))
    scr.flags |= SD_SCR_SPEC3;

  scr.sd_ext_sec  = (raw[0] >> 10) & 0xF;
  scr.cmd_support = (raw[0] & 0x3);
  scr.rsvd        = raw[1];

  switch (scr.sd_spec)
  {
    case 0: version_flags = SD_SPEC_VER1_0; break;
    case 1: version_flags = SD_SPEC_VER1_1; break;
    case 2:
      version_flags = SD_SPEC_VER2_0;
      if (scr.flags & SD_SCR_SPEC3)
        version_flags = SD_SPEC_VER3_0;
      break;
    default:
      break;
  }

  const char *width_str = "unknown";
  if (scr.sd_width & 0x1) width_str = "1-bit";
  if (scr.sd_width & 0x4) width_str = "1-bit & 4-bit";

  shell_fprintf(sh, SHELL_VT100_COLOR_CYAN, "SCR decode:\n");

  shell_fprintf(sh, SHELL_INFO,              "  SCR structure      : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u\n",
                scr.scr_structure);

  shell_fprintf(sh, SHELL_INFO,              "  SD spec            : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u (flags 0x%02X)\n",
                scr.sd_spec, version_flags);

  shell_fprintf(sh, SHELL_INFO,              "  Security           : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%02X\n",
                scr.sd_sec);

  shell_fprintf(sh, SHELL_INFO,              "  Bus widths support : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%02X (%s)\n",
                scr.sd_width, width_str);

  shell_fprintf(sh, SHELL_INFO,              "  Extended security  : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%02X\n",
                scr.sd_ext_sec);

  shell_fprintf(sh, SHELL_INFO,              "  CMD support        : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%02X (bit0=CMD20, bit1=CMD23)\n",
                scr.cmd_support);

  if (scr.flags & SD_SCR_DATA_STATUS_AFTER_ERASE)
    shell_print(sh, "  Flag: data is all '1' after erase");
  if (scr.flags & SD_SCR_SPEC3)
    shell_print(sh, "  Flag: spec version 3.0 or higher");
}

const char *speed_class_str(u8 sc)
{
  switch (sc)
  {
    case 0x00: return "Class 0 (no speed class)";
    case 0x01: return "Class 2";
    case 0x02: return "Class 4";
    case 0x03: return "Class 6";
    case 0x04: return "Class 10";
    default:   return "Reserved/unknown";
  }
}

void print_sd_status_info(const u8 *buf)
{
  u32 w[2];
  make_raw32(buf, w, 2);

  u32 w0 = w[0];
  u32 w1 = w[1];

  u8  dat_bus_width = (w0 >> 30) & 0x3;
  u8  secured_mode  = (w0 >> 29) & 0x1;
  u16 card_type     = (u16)(w0 & 0xFFFF);

  u16 size_prot   = (u16)((w1 >> 16) & 0xFFFF);
  u8  speed_class = (w1 >> 8) & 0xFF;
  u8  perf_move   = (u8)(w1 & 0xFF);

  const char *bus_str =
    dat_bus_width == 0 ? "1-bit" :
    dat_bus_width == 2 ? "4-bit" :
                         "reserved";

  shell_fprintf(sh, SHELL_VT100_COLOR_CYAN,
                "SD Status (ACMD13) decode:\n");

  shell_fprintf(sh, SHELL_INFO,              "  DAT bus width      : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u (%s)\n",
                dat_bus_width, bus_str);

  shell_fprintf(sh, SHELL_INFO,              "  Secured mode       : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%s\n",
                secured_mode ? "ON (CPRM security)" : "OFF");

  shell_fprintf(sh, SHELL_INFO,              "  Card type bits     : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%04X\n",
                card_type);

  shell_fprintf(sh, SHELL_INFO,              "  Protected area raw : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%04X\n",
                size_prot);

  shell_fprintf(sh, SHELL_INFO,              "  Speed class        : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%02X (%s)\n",
                speed_class, speed_class_str(speed_class));

  if (perf_move == 0xFF)
  {
    shell_fprintf(sh, SHELL_INFO,              "  Performance move   : ");
    shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0xFF (infinite / not limited)\n");
  }
  else if (perf_move == 0x00)
  {
    shell_fprintf(sh, SHELL_INFO,              "  Performance move   : ");
    shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0 (not defined)\n");
  }
  else
  {
    shell_fprintf(sh, SHELL_INFO,              "  Performance move   : ");
    shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u MB/s\n",
                  perf_move);
  }
}

void print_switch_info(const u8 *buf)
{
  u8 bus_speed_bits = buf[13];
  u8 drv_bits       = buf[9];
  u8 curr_bits      = buf[7];
  u8 sel_timing     = buf[16] & 0x0F;
  u8 sel_curr       = (buf[15] >> 4) & 0x0F;

  shell_fprintf(sh, SHELL_VT100_COLOR_CYAN,
                "CMD6 switch status decode:\n");

  shell_fprintf(sh, SHELL_INFO,
                "  Supported bus speeds (group 1, status[13]=0x%02X): ",
                bus_speed_bits);
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "\n");

  if (bus_speed_bits & HIGH_SPEED_BUS_SPEED)
    shell_print(sh, "    - High speed (25/50 MHz)");
  if (bus_speed_bits & UHS_SDR12_BUS_SPEED)
    shell_print(sh, "    - UHS SDR12");
  if (bus_speed_bits & UHS_SDR25_BUS_SPEED)
    shell_print(sh, "    - UHS SDR25");
  if (bus_speed_bits & UHS_SDR50_BUS_SPEED)
    shell_print(sh, "    - UHS SDR50");
  if (bus_speed_bits & UHS_SDR104_BUS_SPEED)
    shell_print(sh, "    - UHS SDR104");
  if (bus_speed_bits & UHS_DDR50_BUS_SPEED)
    shell_print(sh, "    - UHS DDR50");

  shell_fprintf(sh, SHELL_INFO,
                "  Supported driver types (status[9]=0x%02X): ",
                drv_bits);
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "\n");

  if (drv_bits & SD_DRIVER_TYPE_B)
    shell_print(sh, "    - Driver type B (default)");
  if (drv_bits & SD_DRIVER_TYPE_A)
    shell_print(sh, "    - Driver type A");
  if (drv_bits & SD_DRIVER_TYPE_C)
    shell_print(sh, "    - Driver type C");
  if (drv_bits & SD_DRIVER_TYPE_D)
    shell_print(sh, "    - Driver type D");

  shell_fprintf(sh, SHELL_INFO,
                "  Supported current limits (status[7]=0x%02X): ",
                curr_bits);
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "\n");

  if (curr_bits & SD_MAX_CURRENT_200MA)
    shell_print(sh, "    - up to 200 mA");
  if (curr_bits & SD_MAX_CURRENT_400MA)
    shell_print(sh, "    - up to 400 mA");
  if (curr_bits & SD_MAX_CURRENT_600MA)
    shell_print(sh, "    - up to 600 mA");
  if (curr_bits & SD_MAX_CURRENT_800MA)
    shell_print(sh, "    - up to 800 mA");

  const char *timing_str = "unknown";
  switch (sel_timing)
  {
    case SD_TIMING_SDR12:  timing_str = "Default / SDR12"; break;
    case SD_TIMING_SDR25:  timing_str = "High speed / SDR25"; break;
    case SD_TIMING_SDR50:  timing_str = "SDR50"; break;
    case SD_TIMING_SDR104: timing_str = "SDR104"; break;
    case SD_TIMING_DDR50:  timing_str = "DDR50"; break;
    default: break;
  }

  shell_fprintf(sh, SHELL_INFO,              "  Selected timing    : ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "%u (%s)\n",
                sel_timing, timing_str);

  shell_fprintf(sh, SHELL_INFO,              "  Selected curr.limit: ");
  shell_fprintf(sh, SHELL_VT100_COLOR_WHITE, "0x%X (CMD6 group 4 value)\n",
                sel_curr);
}

int disk_info(uint64_t &size_mb, uint32_t &block_count, uint32_t &block_size)
{
  int rc;

  // 1) Get internal SDMMC structures (same trick you already use for sd_cmd)
  struct disk_info *disk = disk_access_get_di(disk_pdrv);
  if (disk == NULL)
  {
    LOG_ERR("disk_access_get_di(\"%s\") failed", disk_pdrv);
    return 1;
  }

  const struct device *dev = disk->dev;
  sdmmc_config *cfg      = (sdmmc_config *)dev->config;
  sdmmc_data   *data     = (sdmmc_data   *)dev->data;
  const struct device *sdhc_dev = cfg->host_controller;

  // 2) Check if card is physically present
  if (!sd_is_card_present(sdhc_dev))
  {
    shell_fprintf(sh, SHELL_WARNING, "No SD card present\n");
    data->status = SD_UNINIT;
    return 1;
  }

  // 3) If previous card was initialized, deinit it cleanly
  if (data->status == SD_OK)
  {
    rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_DEINIT, NULL);
    shell_print(sh, "DISK_IOCTL_CTRL_DEINIT rc=%d", rc);
    // driver also sets status = SD_UNINIT, but keep our mirror in sync
    data->status = SD_UNINIT;
  }

  // 4) (Re)init the card via SD subsystem
  rc = sd_init(sdhc_dev, &data->card);
  shell_print(sh, "sd_init rc %d", rc);
  if (rc != 0)
  {
    LOG_ERR("Storage init ERROR! rc=%d", rc);
    data->status = SD_ERROR;
    return 1;
  }

  data->status = SD_OK;
  shell_print(sh, "Storage init OK");

  // 5) Now query geometry via normal disk ioctls
  rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &block_count);
  if (rc)
  {
    LOG_ERR("Unable to get sector count, rc=%d", rc);
    return 2;
  }
  else
  {
    shell_print(sh, "Block count %u", block_count);
  }

  rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &block_size);
  if (rc)
  {
    LOG_ERR("Unable to get sector size, rc=%d", rc);
    return 3;
  }
  else
  {
    shell_print(sh, "Sector size %u", block_size);
    size_mb = (uint64_t)block_count * block_size;
    shell_print(sh, "Bulk size %u MB, %u GB",
                (uint32_t)(size_mb >> 20), (uint32_t)(size_mb >> 30));
    return 0;
  }
}

int sd_cmd(uint32_t opcode, uint32_t arg, uint32_t response_type, uint8_t *buf = NULL, uint32_t size = 1)
{
  int rc;
  struct disk_info *disk = disk_access_get_di(disk_pdrv);
  const struct device *dev = disk->dev;
  struct sdmmc_config *cfg = (sdmmc_config*)dev->config;
  struct sdmmc_data *dat = (sdmmc_data*)dev->data;
  const struct device *sdhc_dev = cfg->host_controller;
  struct sd_card *card = &dat->card;

  struct sdhc_command cmd = {0};
  struct sdhc_data data = {0};

  cmd.opcode = opcode;
  cmd.arg = arg;
  cmd.response_type = response_type;
  cmd.timeout_ms = 30000;

  if (buf)
  {
    data.data = buf;
    data.block_size = size;
    data.blocks = 1U;
    data.timeout_ms = 30000;
    rc = sdhc_request(card->sdhc, &cmd, &data);
    if (rc) return rc;

    if (response_type == SD_SPI_RSP_TYPE_R1b)
      rc = sdhc_spi_wait_unbusy(card->sdhc, 60000, 100);  /* Zephyr timeout bugfix */

    // if (response_type == SD_SPI_RSP_TYPE_R3)
      // buf[0] = cmd.response[1];
  }
  else
  {
    rc = sdhc_request(card->sdhc, &cmd, NULL);
    if (rc) return rc;

    if (response_type == SD_SPI_RSP_TYPE_R1b)
      rc = sdhc_spi_wait_unbusy(card->sdhc, 60000, 100);  /* Zephyr timeout bugfix */
  }

  return rc;
}

int sd_acmd(uint32_t opcode, uint32_t arg, uint32_t response_type, uint8_t *buf = NULL, uint32_t size = 1)
{
  int rc = sd_cmd(SD_APP_CMD, 0, SD_SPI_RSP_TYPE_R1);
  if (rc) return rc;
  return sd_cmd(opcode, arg, response_type, buf, size);
}

// ----- Shell commands

int cmd_info(const shell *sh_, size_t argc, char **argv)
{
  sh = sh_;
  uint32_t block_count;
  uint32_t block_size;
  uint64_t size_mb;
  int rc;

  rc = disk_info(size_mb, block_count, block_size);
  if (rc) return rc;

  uint8_t buf[64];

  rc = sd_cmd(SD_SEND_CID, 0, SD_SPI_RSP_TYPE_R1, buf, 16);
  shell_print(sh, "SD_SEND_CID, rc %d", rc);
  if (rc == 0)
  {
    // dump(buf, 16);
    // dump(buf, 16, ' ');

    print_cid_info(buf);
  }

  rc = sd_cmd(SD_SEND_CSD, 0, SD_SPI_RSP_TYPE_R1, buf, 16);
  shell_print(sh, "SD_SEND_CSD, rc %d", rc);
  if (rc == 0)
  {
    // dump(buf, 16);
    // dump(buf, 16, ' ');

    print_csd_info(buf, block_count, block_size);
  }

  rc = sd_acmd(SD_APP_SEND_SCR, 0, SD_SPI_RSP_TYPE_R1, buf, 8);
  shell_print(sh, "SD_APP_SEND_SCR, rc %d", rc);
  if (rc == 0)
  {
    // dump(buf, 8);
    // dump(buf, 8, ' ');

    print_scr_info(buf);
  }

  // rc = sd_acmd(SD_APP_SEND_OP_COND, 0, SD_SPI_RSP_TYPE_R3, buf);
  // shell_print(sh, "SD_APP_SEND_OP_COND (read OCR), rc %d", rc);
  // if (rc == 0)
  // {
    // dump(buf, 1);
    // dump(buf, 1, ' ');
  // }

  rc = sd_acmd(SD_APP_SEND_STATUS, 0, SD_SPI_RSP_TYPE_R2, buf, 64);
  shell_print(sh, "SD_APP_SEND_STATUS, rc %d", rc);
  if (rc == 0)
  {
    // dump(buf, 64);
    // dump(buf, 64, ' ');

    print_sd_status_info(buf);
  }

  rc = sd_cmd(SD_SWITCH, 0x00FFFFFF, SD_SPI_RSP_TYPE_R1, buf, 64);
  shell_print(sh, "SD_SWITCH, rc %d", rc);
  if (rc == 0)
  {
    // dump(buf, 64);
    // dump(buf, 64, ' ');

    print_switch_info(buf);
  }

  return 0;
}

SHELL_CMD_ARG_REGISTER(info, NULL, "Card info", cmd_info, 1, 0);

// -------------

int cmd_erase(const shell *sh_, size_t argc, char **argv)
{
  sh = sh_;
  uint64_t size_mb;
  uint32_t block_count;
  uint32_t block_size;
  int rc;

  rc = disk_info(size_mb, block_count, block_size);
  if (rc != 0) return rc;

  shell_fprintf(sh, SHELL_WARNING, "Erasing SD card\n");

  rc = sd_cmd(SD_ERASE_BLOCK_START, 0, SD_SPI_RSP_TYPE_R1);
  shell_print(sh, "SD_ERASE_BLOCK_START, rc %d", rc);

  rc = sd_cmd(SD_ERASE_BLOCK_END, block_count - 1, SD_SPI_RSP_TYPE_R1);
  shell_print(sh, "SD_ERASE_BLOCK_END, rc %d", rc);

  rc = sd_cmd(SD_ERASE_BLOCK_OPERATION, 0, SD_SPI_RSP_TYPE_R1b);
  shell_print(sh, "SD_ERASE_BLOCK_OPERATION, rc %d", rc);

  return 0;
}

SHELL_CMD_ARG_REGISTER(erase, NULL, "Erase full card", cmd_erase, 1, 0);
