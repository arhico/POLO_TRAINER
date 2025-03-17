/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/* DESCRIPTION:
 * This example contains code to make ESP32 based device recognizable by USB-hosts as a USB Mass Storage Device.
 * It either allows the embedded application i.e. example to access the partition or Host PC accesses the partition over USB MSC.
 * They can't be allowed to access the partition at the same time.
 * For different scenarios and behaviour, Refer to README of this example.
 */

#include <errno.h>
#include <dirent.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "esp_console.h"
#include "esp_check.h"
#include "esp_partition.h"
#include "driver/gpio.h"
#include "driver/dac_continuous.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#ifdef CONFIG_EXAMPLE_STORAGE_MEDIA_SDMMC
#include "diskio_impl.h"
#include "diskio_sdmmc.h"
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#endif // CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
#endif
#include "math.h"
#include "driver/touch_sensor.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"


static QueueHandle_t que_touch = NULL;
typedef struct touch_msg {
    touch_pad_intr_mask_t intr_mask;
    uint32_t pad_num;
    uint32_t pad_status;
    uint32_t pad_val;
} touch_event_t;

RTC_DATA_ATTR static float normalizing_coeff = -1.0;
/*
 * We warn if a secondary serial console is enabled. A secondary serial console is always output-only and
 * hence not very useful for interactive console applications. If you encounter this warning, consider disabling
 * the secondary serial console in menuconfig unless you know what you are doing.
 */
#if SOC_USB_SERIAL_JTAG_SUPPORTED
#if !CONFIG_ESP_CONSOLE_SECONDARY_NONE
#warning "A secondary serial console is not useful when using the console component. Please disable it in menuconfig."
#endif
#endif

static const char* TAG = "polo_trainer";
static esp_console_repl_t* repl = NULL;

/* TinyUSB descriptors
   ********************************************************************* */
#define EPNUM_MSC       1
#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0 ,
    ITF_NUM_TOTAL
};

enum {
    EDPT_CTRL_OUT = 0x00 ,
    EDPT_CTRL_IN = 0x80 ,

    EDPT_MSC_OUT = 0x01 ,
    EDPT_MSC_IN = 0x81 ,
};

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // This is Espressif VID. This needs to be changed according to Users / Customers
    .idProduct = 0x4666,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static uint8_t const msc_fs_configuration_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64),
};

#if (TUD_OPT_HIGH_SPEED)
static const tusb_desc_device_qualifier_t device_qualifier = {
    .bLength = sizeof(tusb_desc_device_qualifier_t),
    .bDescriptorType = TUSB_DESC_DEVICE_QUALIFIER,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .bNumConfigurations = 0x01,
    .bReserved = 0
};

static uint8_t const msc_hs_configuration_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 512),
};
#endif // TUD_OPT_HIGH_SPEED

static char const* string_desc_arr[] = {
    (const char[]) {
 0x09, 0x04
},  // 0: is supported language is English (0x0409)
"AUTOMIND.TOP",                      // 1: Manufacturer
"POLO TRAINER",               // 2: Product
"000001",                       // 3: Serials
"POLO TRAINER",                  // 4. MSC
};
/*********************************************************************** TinyUSB descriptors*/

#define BASE_PATH "/data" // base path to mount the partition

// #define PROMPT_STR CONFIG_IDF_TARGET
// static int console_unmount(int argc , char** argv);
// static int console_read(int argc , char** argv);
// static int console_write(int argc , char** argv);
// static int console_size(int argc , char** argv);
// static int console_status(int argc , char** argv);
// static int console_exit(int argc , char** argv);
// const esp_console_cmd_t cmds[] = {
//     {
//         .command = "read",
//         .help = "read BASE_PATH/README.MD and print its contents",
//         .hint = NULL,
//         .func = &console_read,
//     },
//     {
//         .command = "write",
//         .help = "create file BASE_PATH/README.MD if it does not exist",
//         .hint = NULL,
//         .func = &console_write,
//     },
//     {
//         .command = "size",
//         .help = "show storage size and sector size",
//         .hint = NULL,
//         .func = &console_size,
//     },
//     {
//         .command = "expose",
//         .help = "Expose Storage to Host",
//         .hint = NULL,
//         .func = &console_unmount,
//     },
//     {
//         .command = "status",
//         .help = "Status of storage exposure over USB",
//         .hint = NULL,
//         .func = &console_status,
//     },
//     {
//         .command = "exit",
//         .help = "exit from application",
//         .hint = NULL,
//         .func = &console_exit,
//     }
// };

// mount the partition and show all the files in BASE_PATH

char name[33];
char full_name[512];

IRAM_ATTR static esp_err_t _mount_and_find_1_wav() {
    static bool name_obtained = false;
    ESP_LOGI(TAG , "Mount storage...");
    ESP_ERROR_CHECK(tinyusb_msc_storage_mount(BASE_PATH));

    // List all the files in this directory
    ESP_LOGI(TAG , "\nls command output:");
    struct dirent* d;
    DIR* dh = opendir(BASE_PATH);
    if (!dh) {
        if (errno == ENOENT) {
            //If the directory is not found
            ESP_LOGE(TAG , "Directory doesn't exist %s" , BASE_PATH);
        } else {
            //If the directory is not readable then throw error and exit
            ESP_LOGE(TAG , "Unable to read directory %s" , BASE_PATH);
        }
        return -1;
    }

    char ext_buf[4] = {0};
        //While the next entry is not readable we will print directory files
    while ((d = readdir(dh)) != NULL) {
        printf("%s, %d\n" , d->d_name , (int)d->d_type);

        if (!name_obtained) {
            for (int i = 0; ; i++) {
                if (d->d_name[i] == '\0') {
                    ext_buf[3] = d->d_name[i - 1];
                    ext_buf[2] = d->d_name[i - 2];
                    ext_buf[1] = d->d_name[i - 3];
                    ext_buf[0] = d->d_name[i - 4];
                    if (ext_buf[0] == '.' &&
                        (ext_buf[1] == 'w' || ext_buf[1] == 'W') &&
                        (ext_buf[2] == 'a' || ext_buf[2] == 'A') &&
                        (ext_buf[3] == 'v' || ext_buf[3] == 'V')) {
                        name_obtained = true;
                        int ext_idx = i - 4;
                        for (size_t k = 0; k < 32; k++) {
                            if (k == ext_idx) break;
                            name[k] = d->d_name[k];
                        }

                        sprintf(full_name , "%s/%s" , BASE_PATH , d->d_name);

                        ESP_LOGW(TAG , "name obtained: %s\nfullname: %s" , name , full_name);
                    }

                    break;
                }
            }
        }
    }
    return !name_obtained;
}

// // unmount storage
// static int console_unmount(int argc , char** argv) {
//     if (tinyusb_msc_storage_in_use_by_usb_host()) {
//         ESP_LOGE(TAG , "storage is already exposed");
//         return -1;
//     }
//     ESP_LOGI(TAG , "Unmount storage...");
//     ESP_ERROR_CHECK(tinyusb_msc_storage_unmount());
//     return 0;
// }

// // read BASE_PATH/README.MD and print its contents
// static int console_read(int argc , char** argv) {
//     if (tinyusb_msc_storage_in_use_by_usb_host()) {
//         ESP_LOGE(TAG , "storage exposed over USB. Application can't read from storage.");
//         return -1;
//     }
//     ESP_LOGD(TAG , "read from storage:");
//     const char* filename = BASE_PATH "/README.MD";
//     FILE* ptr = fopen(filename , "r");
//     if (ptr == NULL) {
//         ESP_LOGE(TAG , "Filename not present - %s" , filename);
//         return -1;
//     }
//     char buf[1024];
//     while (fgets(buf , 1000 , ptr) != NULL) {
//         printf("%s" , buf);
//     }
//     fclose(ptr);
//     return 0;
// }

// // create file BASE_PATH/README.MD if it does not exist
// static int console_write(int argc , char** argv) {
//     if (tinyusb_msc_storage_in_use_by_usb_host()) {
//         ESP_LOGE(TAG , "storage exposed over USB. Application can't write to storage.");
//         return -1;
//     }
//     ESP_LOGD(TAG , "write to storage:");
//     const char* filename = BASE_PATH "/README.MD";
//     FILE* fd = fopen(filename , "r");
//     if (!fd) {
//         ESP_LOGW(TAG , "README.MD doesn't exist yet, creating");
//         fd = fopen(filename , "w");
//         fprintf(fd , "Mass Storage Devices are one of the most common USB devices. It use Mass Storage Class (MSC) that allow access to their internal data storage.\n");
//         fprintf(fd , "In this example, ESP chip will be recognised by host (PC) as Mass Storage Device.\n");
//         fprintf(fd , "Upon connection to USB host (PC), the example application will initialize the storage module and then the storage will be seen as removable device on PC.\n");
//         fclose(fd);
//     }
//     return 0;
// }

// // Show storage size and sector size
// static int console_size(int argc , char** argv) {
//     if (tinyusb_msc_storage_in_use_by_usb_host()) {
//         ESP_LOGE(TAG , "storage exposed over USB. Application can't access storage");
//         return -1;
//     }
//     uint32_t sec_count = tinyusb_msc_storage_get_sector_count();
//     uint32_t sec_size = tinyusb_msc_storage_get_sector_size();
//     printf("Storage Capacity %lluMB\n" , ((uint64_t)sec_count) * sec_size / (1024 * 1024));
//     return 0;
// }

// // exit from application
// static int console_status(int argc , char** argv) {
//     printf("storage exposed over USB: %s\n" , tinyusb_msc_storage_in_use_by_usb_host() ? "Yes" : "No");
//     return 0;
// }

// // exit from application
// static int console_exit(int argc , char** argv) {
//     tinyusb_msc_unregister_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED);
//     tinyusb_msc_storage_deinit();
//     tinyusb_driver_uninstall();
//     printf("Application Exit\n");
//     repl->del(repl);
//     return 0;
// }

// callback that is delivered when storage is mounted/unmounted by application.
static void storage_mount_changed_cb(tinyusb_msc_event_t* event) {
    ESP_LOGI(TAG , "Storage mounted to application: %s" , event->mount_changed_data.is_mounted ? "Yes" : "No");
}

#ifdef CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH
static esp_err_t storage_init_spiflash(wl_handle_t* wl_handle) {
    ESP_LOGI(TAG , "Initializing wear levelling");

    const esp_partition_t* data_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA , ESP_PARTITION_SUBTYPE_DATA_FAT , NULL);
    if (data_partition == NULL) {
        ESP_LOGE(TAG , "Failed to find FATFS partition. Check the partition table.");
        return ESP_ERR_NOT_FOUND;
    }

    return wl_mount(data_partition , wl_handle);
}
#else  // CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH
static esp_err_t storage_init_sdmmc(sdmmc_card_t** card) {
    esp_err_t ret = ESP_OK;
    bool host_init = false;
    sdmmc_card_t* sd_card;

    ESP_LOGI(TAG , "Initializing SDCard");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config , &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG , "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // For SD Card, set bus width to use
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
    slot_config.width = 4;
#else
    slot_config.width = 1;
#endif  // CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4

    // On chips where the GPIOs used for SD card can be configured, set the user defined values
#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
#endif  // CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4

#endif  // CONFIG_SOC_SDMMC_USE_GPIO_MATRIX

    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // not using ff_memalloc here, as allocation in internal RAM is preferred
    sd_card = (sdmmc_card_t*)malloc(sizeof(sdmmc_card_t));
    ESP_GOTO_ON_FALSE(sd_card , ESP_ERR_NO_MEM , clean , TAG , "could not allocate new sdmmc_card_t");

    ESP_GOTO_ON_ERROR((*host.init)() , clean , TAG , "Host Config Init fail");
    host_init = true;

    ESP_GOTO_ON_ERROR(sdmmc_host_init_slot(host.slot , (const sdmmc_slot_config_t*)&slot_config) ,
        clean , TAG , "Host init slot fail");

    while (sdmmc_card_init(&host , sd_card)) {
        ESP_LOGE(TAG , "The detection pin of the slot is disconnected(Insert uSD card). Retrying...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout , sd_card);
    *card = sd_card;

    return ESP_OK;

clean:
    if (host_init) {
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
            host.deinit_p(host.slot);
        } else {
            (*host.deinit)();
        }
    }
    if (sd_card) {
        free(sd_card);
        sd_card = NULL;
    }
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    // We don't need to duplicate error here as all error messages are handled via sd_pwr_* call
    sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
#endif // CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    return ret;
}
#endif  // CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH



// const double nanosec_per_cpu_tick = 1000 / 240.0; // 4 ns @ 240 MHz
// const uint32_t cpu_ticks_per_period = (uint32_t)(aud_sample_len_ns / nanosec_per_cpu_tick);
const uint32_t OVERSAMPLING = 4;

IRAM_ATTR static esp_err_t dac_output_with_dither(float gain , uint8_t* buf , uint16_t bitwidth , dac_continuous_handle_t dac_contin , size_t len) {
    // size_t bytes_written = 0;

        // uint16_t value_16_bit = 0;
        // uint32_t cpu_tick_count = portGET_RUN_TIME_COUNTER_VALUE();

    const int byteskip_div = bitwidth / 8;
    size_t samples_count = len / byteskip_div;
    int8_t buf_8_bit_msb[samples_count];
    int8_t buf_8_bit_lsb[samples_count];
    const int32_t dither_amplitude = (20);
    int8_t buf_8_bit_oversampled[samples_count * OVERSAMPLING];
    // const int truncate_coeff = (128 / OVERSAMPLING);
    for (int i = 0; i < len; i += byteskip_div) {
        size_t cur_sample = i / byteskip_div;

        if (byteskip_div == 1) { // 8 bits is only format with unsigned data
            buf_8_bit_msb[cur_sample] = (int8_t)(buf[i] - 128);
            buf_8_bit_lsb[cur_sample] = 0;


        } else {
            buf_8_bit_msb[cur_sample] = (int8_t)buf[i + 1];
            buf_8_bit_lsb[cur_sample] = (int8_t)buf[i];
        }
        // TODO separate by bits

        if (gain != 1.0) {
            float word = ((buf_8_bit_msb[cur_sample] << 8) + (buf_8_bit_lsb[cur_sample])) / (float)INT16_MAX;

            word *= gain;

            // union processed_union {
            //     int16_t integ_16;
            //     int8_t integ_8_msb;
            //     int8_t integ_8_lsb;
            // } processed;
            int32_t ooouut = (int16_t)round(word * INT16_MAX);

            if (ooouut > INT16_MAX) ooouut = INT16_MAX;
            if (ooouut < INT16_MIN) ooouut = INT16_MIN;

            buf_8_bit_msb[cur_sample] = ooouut >> 8;
            buf_8_bit_lsb[cur_sample] = ooouut & 0xFF;
        }

        // buf_8_bit_msb[cur_sample] = (int8_t)buf[i + 1] + 128;
        // buf_8_bit_lsb[cur_sample] = (int8_t)buf[i] + 128;

        for (size_t k = 0; k < OVERSAMPLING; k++) {
            int32_t pwm_ovsmpl_val = 0;
            int32_t dither = rand() % dither_amplitude - dither_amplitude / 2;
            if (buf_8_bit_lsb[cur_sample] + dither >= 127) {
                pwm_ovsmpl_val = 1;
            }
            if (buf_8_bit_lsb[cur_sample] + dither <= -128) {
                pwm_ovsmpl_val = -1;
            }

            buf_8_bit_oversampled[cur_sample * OVERSAMPLING + k] = buf_8_bit_msb[cur_sample] + pwm_ovsmpl_val;
        }
    }
    dac_continuous_write(dac_contin , (uint8_t*)buf_8_bit_oversampled , samples_count * OVERSAMPLING , NULL , -1);

    return ESP_OK;
}

// uint32_t AUDIO_SAMPLE_RATE = 44100;

#define AUDIO_BUFFER 512

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "mp3_decoder.h"


IRAM_ATTR esp_err_t play_wav(char* fp) {

    ESP_LOGI(TAG , "playing: %s" , fp);
    FILE* fh = fopen(fp , "rb");
    if (fh == NULL) {
        ESP_LOGE(TAG , "Failed to open file");
        return ESP_ERR_INVALID_ARG;
    }

    // read sample rate
    union four_bytes_union_t {
        uint8_t bytes[4];
        uint32_t glued;
    } samplerate_union;

    union two_bytes_union_t {
        uint8_t bytes[2];
        uint16_t glued;
    } bitdepth_union;

    two_bytes_union_t channels_union;

    fseek(fh , 22 , SEEK_SET);
    fread(channels_union.bytes , sizeof(uint8_t) , 2 , fh);

    fseek(fh , 24 , SEEK_SET);
    fread(samplerate_union.bytes , sizeof(uint8_t) , 4 , fh);

    fseek(fh , 34 , SEEK_SET);
    fread(bitdepth_union.bytes , sizeof(uint8_t) , 2 , fh);

    ESP_LOGI(TAG , "wave header samplerate: %lu, bitdepth: %u, channels %u" , samplerate_union.glued , bitdepth_union.glued , channels_union.glued);

    // after the header
    fseek(fh , 44 , SEEK_SET);

    if (normalizing_coeff < 0) {
        ESP_LOGW(TAG , "CALCULATING GAIN (not yet really)"); // TODO
        // while (bytes_read > 0) {
        //     bytes_read = fread(buf , sizeof(uint8_t) , AUDIO_BUFFER , fh);
        // }
        // calc
        normalizing_coeff = 1.0;
        fclose(fh);
        return play_wav(fp);
    }

    dac_continuous_handle_t dac_handle;
    dac_continuous_config_t cont_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_ALL,
        .desc_num = 8,
        .buf_size = 4096,
        .freq_hz = samplerate_union.glued * OVERSAMPLING,
        .offset = -128,
        .clk_src = DAC_DIGI_CLK_SRC_APLL,   // Using APLL as clock source to get a wider frequency range
        /* Assume the data in buffer is 'A B C D E F'
         * DAC_CHANNEL_MODE_SIMUL:
         *      - channel 0: A B C D E F
         *      - channel 1: A B C D E F
         * DAC_CHANNEL_MODE_ALTER:
         *      - channel 0: A C E
         *      - channel 1: B D F
         */
        .chan_mode = (dac_continuous_channel_mode_t)(DAC_CHANNEL_MODE_ALTER & (channels_union.glued == 2)),
        // .chan_mode = DAC_CHANNEL_MODE_SIMUL,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&cont_cfg , &dac_handle));

// #if CONFIG_EXAMPLE_DAC_WRITE_ASYNC
//     /* Create a queue to transport the interrupt event data */
//     QueueHandle_t que = xQueueCreate(10 , sizeof(dac_event_data_t));
//     assert(que);
//     dac_event_callbacks_t cbs = {
//         .on_convert_done = dac_on_convert_done_callback,
//         .on_stop = NULL,
//     };
//     /* Must register the callback if using asynchronous writing */
//     ESP_ERROR_CHECK(dac_continuous_register_event_callback(dac_handle , &cbs , que));
// #endif
    /* Enable the continuous channels */
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
    ESP_LOGI(TAG , "DAC initialized success, DAC DMA is ready");


    // create a writer buffer
    uint8_t* buf = (uint8_t*)calloc(AUDIO_BUFFER , sizeof(uint8_t));
    size_t bytes_read = 0;
    size_t bytes_written = 0;

    bytes_read = fread(buf , sizeof(uint8_t) , AUDIO_BUFFER , fh);

  //   i2s_channel_enable(tx_handle);
//   ESP_LOGV(TAG , "Bytes read: %d" , bytes_read);



    // normalizing_coeff = (float)rand() / UINT32_MAX;
    ESP_LOGI(TAG , "normalizing_coeff: % f" , normalizing_coeff);

    // uint16_t bitwidth = ;

    while (bytes_read > 0) {
        bytes_read = fread(buf , sizeof(uint8_t) , AUDIO_BUFFER , fh);
        dac_output_with_dither(normalizing_coeff , buf , bitdepth_union.glued , dac_handle , bytes_read);
    }


  //   i2s_channel_disable(tx_handle);
    ESP_LOGI(TAG , "done! cleaning");

    dac_continuous_disable(dac_handle);
    dac_continuous_del_channels(dac_handle);
    free(buf);
    fclose(fh);
    return ESP_OK;
}

#define TOUCH_1 TOUCH_PAD_NUM2

static void touchsensor_filter_set(touch_filter_mode_t mode) {
    /* Filter function */
    touch_filter_config_t filter_info = {
        .mode = mode,           // Test jitter and filter 1/4.
        .debounce_cnt = 1,      // 1 time count.
        .noise_thr = 0,         // 50%
        .jitter_step = 4,       // use for jitter mode.
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2,
    };
    touch_pad_filter_set_config(&filter_info);
    touch_pad_filter_enable();
    ESP_LOGI(TAG , "touch pad filter init");
}

static void touchsensor_interrupt_cb(void* arg) {
    int task_awoken = pdFALSE;
    touch_event_t evt;

    evt.intr_mask = (touch_pad_intr_mask_t)touch_pad_read_intr_status_mask();
    evt.pad_status = touch_pad_get_status();
    evt.pad_num = touch_pad_get_current_meas_channel();

    xQueueSendFromISR(que_touch , &evt , &task_awoken);
    if (task_awoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void tp_example_set_thresholds(void) {
    RTC_DATA_ATTR static bool calibrated = false;
    RTC_DATA_ATTR static uint32_t cal_value;

    if (calibrated) {
        ESP_LOGI(TAG , "skipped calibration");
        touch_pad_set_thresh(TOUCH_1 , cal_value * 0.2);

        // return;
    } else {
        vTaskDelay(50 / portTICK_PERIOD_MS);

    // for (int i = 0; i < TOUCH_BUTTON_NUM; i++) {
        //read benchmark value
        touch_pad_read_benchmark(TOUCH_1 , &cal_value);
    //set interrupt threshold.
        touch_pad_set_thresh(TOUCH_1 , cal_value * 0.2);
        ESP_LOGW(TAG , "touch %d calib %"PRIu32", thresh %"PRIu32 , TOUCH_1 , cal_value , (uint32_t)(cal_value * 0.2));
        calibrated = true;
    }
}

static void tp_example_read_task(void* pvParameter) {
    touch_event_t evt;
    static uint8_t guard_mode_flag = 0;
    /* Wait touch sensor init done */

    tp_example_set_thresholds();

    while (1) {
        int ret = xQueueReceive(que_touch , &evt , (TickType_t)portMAX_DELAY);
        if (ret != pdTRUE) {
            continue;
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_ACTIVE) {
            /* if guard pad be touched, other pads no response. */
            // if (evt.pad_num == button[3]) {
            //     guard_mode_flag = 1;
            //     ESP_LOGW(TAG, "TouchSensor [%"PRIu32"] be activated, enter guard mode", evt.pad_num);
            // } else {
            //     if (guard_mode_flag == 0) {
            ESP_LOGI(TAG , "TouchSensor [%"PRIu32"] be activated, status mask 0x%"PRIu32"" , evt.pad_num , evt.pad_status);
        // } else {
            // ESP_LOGW(TAG, "In guard mode. No response");
        // }
    // }
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_INACTIVE) {
            /* if guard pad be touched, other pads no response. */
            // if (evt.pad_num == button[3]) {
                // guard_mode_flag = 0;
                // ESP_LOGW(TAG, "TouchSensor [%"PRIu32"] be inactivated, exit guard mode", evt.pad_num);
            // } else {
                // if (guard_mode_flag == 0) {
            ESP_LOGI(TAG , "TouchSensor [%"PRIu32"] be inactivated, status mask 0x%"PRIu32 , evt.pad_num , evt.pad_status);
        // }
    // }
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_SCAN_DONE) {
            ESP_LOGI(TAG , "The touch sensor group measurement is done [%"PRIu32"]." , evt.pad_num);
        }
        if (evt.intr_mask & TOUCH_PAD_INTR_MASK_TIMEOUT) {
            /* Add your exception handling in here. */
            ESP_LOGI(TAG , "Touch sensor channel %"PRIu32" measure timeout. Skip this exception channel!!" , evt.pad_num);
            touch_pad_timeout_resume(); // Point on the next channel to measure.
        }
    }
}

esp_err_t properly_inited = ESP_OK;

extern "C" void app_main(void) {



    ESP_LOGI(TAG , "initializing touch");
    if (que_touch == NULL) {
        que_touch = xQueueCreate(1 , sizeof(touch_event_t));
    }

    touch_pad_init();
    touch_pad_config(TOUCH_1);

    touchsensor_filter_set(TOUCH_PAD_FILTER_IIR_16);
    touch_pad_timeout_set(true , TOUCH_PAD_THRESHOLD_MAX);
    /* Register touch interrupt ISR, enable intr type. */
    touch_pad_isr_register(touchsensor_interrupt_cb , NULL , (touch_pad_intr_mask_t)TOUCH_PAD_INTR_MASK_ALL);
    /* If you have other touch algorithm, you can get the measured value after the `TOUCH_PAD_INTR_MASK_SCAN_DONE` interrupt is generated. */
    touch_pad_intr_enable((touch_pad_intr_mask_t)(TOUCH_PAD_INTR_MASK_ACTIVE | TOUCH_PAD_INTR_MASK_INACTIVE | TOUCH_PAD_INTR_MASK_TIMEOUT));

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    // vTaskDelay(pdMS_TO_TICKS(1000));
    tp_example_set_thresholds();

    // uint32_t tres;
    // touch_pad_sleep_channel_enable(TOUCH_1 , true);

    // touch_pad_get_thresh(TOUCH_1 , &tres);

    // touch_pad_sleep_set_threshold(TOUCH_1 , tres);

    // Start a task to show what pads have been touched
    // xTaskCreate(&tp_example_read_task , "touch_pad_read_task" , 4096 , NULL , 5 , NULL);



    // uint32_t touch_value;
    // touch_pad_read_benchmark(TOUCH_1 , &touch_value);
    //set interrupt threshold.
    // touch_pad_set_thresh(TOUCH_1 , touch_value * 0.2);
    // ESP_LOGI(TAG, "touch pad [%d] base %"PRIu32", thresh %"PRIu32,

    ESP_LOGI(TAG , "initializing storage...");

#ifdef CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH
    static wl_handle_t wl_handle = WL_INVALID_HANDLE;
    ESP_ERROR_CHECK(storage_init_spiflash(&wl_handle));

    const tinyusb_msc_spiflash_config_t config_spi = {
        .wl_handle = wl_handle,
        .callback_mount_changed = storage_mount_changed_cb,  /* First way to register the callback. This is while initializing the storage. */
        .mount_config = {.max_files = 5},
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_spiflash(&config_spi));
    ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED , storage_mount_changed_cb)); /* Other way to register the callback i.e. registering using separate API. If the callback had been already registered, it will be overwritten. */
#else // CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH
    static sdmmc_card_t* card = NULL;
    ESP_ERROR_CHECK(storage_init_sdmmc(&card));

    const tinyusb_msc_sdmmc_config_t config_sdmmc = {
        .card = card,
        .callback_mount_changed = storage_mount_changed_cb,  /* First way to register the callback. This is while initializing the storage. */
        .mount_config.max_files = 5,
    };
    ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&config_sdmmc));
    ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED , storage_mount_changed_cb)); /* Other way to register the callback i.e. registering using separate API. If the callback had been already registered, it will be overwritten. */
#endif  // CONFIG_EXAMPLE_STORAGE_MEDIA_SPIFLASH

    //mounted in the app by default
    properly_inited = _mount_and_find_1_wav();

    esp_sleep_wakeup_cause_t  cause = esp_sleep_get_wakeup_cause();
    ESP_LOGW(TAG , "wakeup cause: %d" , cause);

    esp_sleep_enable_touchpad_wakeup();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0 , 0);
    gpio_set_direction(GPIO_NUM_0 , GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_0 , GPIO_PULLUP_ONLY);
    gpio_sleep_set_direction(GPIO_NUM_0 , GPIO_MODE_INPUT);
    gpio_sleep_set_pull_mode(GPIO_NUM_0 , GPIO_PULLUP_ONLY);

    rtc_gpio_pullup_en(GPIO_NUM_0);
    rtc_gpio_pulldown_dis(GPIO_NUM_0);
    // gpio_deep_sleep_hold_en();
    // if (gpio_get_level(GPIO_NUM_0) == 0) {
    //     properly_inited = -1;
    // }

    int timeout = 0;




#define TIMEOUT 20

    if (properly_inited == ESP_OK && cause == ESP_SLEEP_WAKEUP_TOUCHPAD) {
        // play audio
        play_wav(full_name);

        // esp_deep_sleep_start();
    }
    if (properly_inited != ESP_OK || cause == ESP_SLEEP_WAKEUP_EXT0) {
       // mount usb to OS
        ESP_LOGI(TAG , "USB MSC initialization");
        const tinyusb_config_t tusb_cfg = {
            .device_descriptor = &descriptor_config,
            .string_descriptor = string_desc_arr,
            .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
            .external_phy = false,
    #if (TUD_OPT_HIGH_SPEED)
            .fs_configuration_descriptor = msc_fs_configuration_desc,
            .hs_configuration_descriptor = msc_hs_configuration_desc,
            .qualifier_descriptor = &device_qualifier,
    #else
            .configuration_descriptor = msc_fs_configuration_desc,
    #endif // TUD_OPT_HIGH_SPEED
        };
        esp_err_t err = (tinyusb_driver_install(&tusb_cfg));
        normalizing_coeff = -1;
        while (gpio_get_level(GPIO_NUM_0) == 0) { vTaskDelay(100); }
        while (gpio_get_level(GPIO_NUM_0) == 1) { vTaskDelay(100); }
        tinyusb_driver_uninstall();
        ESP_LOGW(TAG , "restarting system");
        vTaskDelay(pdMS_TO_TICKS(50));
        while (gpio_get_level(GPIO_NUM_0) == 0) { vTaskDelay(100); }
        esp_restart();
    // } else {

    }

    ESP_LOGW(TAG , "going to deep sleep");
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_deep_sleep_start();

    // }



    // while (1) {
    //     // while (tud_connected()) {
    //     vTaskDelay(100);
    // }

    // while (1) {
        // if (timeout >= TIMEOUT) {
        //     ESP_LOGI(TAG , "USB MSC initialization");
        //     const tinyusb_config_t tusb_cfg = {
        //         .device_descriptor = &descriptor_config,
        //         .string_descriptor = string_desc_arr,
        //         .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        //         .external_phy = false,
        // #if (TUD_OPT_HIGH_SPEED)
        //         .fs_configuration_descriptor = msc_fs_configuration_desc,
        //         .hs_configuration_descriptor = msc_hs_configuration_desc,
        //         .qualifier_descriptor = &device_qualifier,
        // #else
        //         .configuration_descriptor = msc_fs_configuration_desc,
        // #endif // TUD_OPT_HIGH_SPEED
        //     };
        //     esp_err_t err = (tinyusb_driver_install(&tusb_cfg));
        //     if (err != ESP_OK) {
        //         normalizing_coeff = 0;
        //         tinyusb_driver_uninstall();
        //         esp_restart();
        //     }
        //     timeout = 0;
        //     while (!gpio_get_level(GPIO_NUM_0)) {
        //         vTaskDelay(pdMS_TO_TICKS(100));
        //     }

        // }
        // if (!gpio_get_level(GPIO_NUM_0)) {
        //     timeout++;
        // } else {
        //     if (timeout) {
        //         play_wav(full_name);
        //     }
        //     timeout = 0;
        // }
        // vTaskDelay(pdMS_TO_TICKS(100));
    // }




}
