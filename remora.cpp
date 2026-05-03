/*
Remora, RP2040 with Wiznet Ethernet, firmware for LinuxCNC
Copyright (C) 2023  Scott Alford (scotta)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License version 3
of the License.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

// PicoBOB has UART on different pins to the standard Pico
//#define PICOBOB

#ifdef PICOBOB
#define PICO_DEFAULT_UART 0
#define PICO_DEFAULT_UART_TX_PIN 28
#define PICO_DEFAULT_UART_RX_PIN 29
#endif


#include <stdio.h>
#include <cstring>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "hardware/regs/busctrl.h"
#include "hardware/structs/bus_ctrl.h"

#include "configuration.h"
#include "remora.h"

#include "crc32.h"

// WIZnet
extern "C"
{
#include "wizchip_conf.h"
#include "socket.h"
#include "wizchip_spi.h"
#include "w5x00_lwip.h"
}

// Ethenet (LWIP)
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/etharp.h"
#include "tftpserver.h"

// libraries
#include "lib/ArduinoJson6/ArduinoJson.h"

// drivers
#include "drivers/pin/pin.h"

// threads
#include "thread/pruThread.h"
#include "thread/createThreads.h"

// modules
#include "modules/module.h"
#include "modules/blink/blink.h"
#include "modules/comms/RemoraComms.h"
#include "modules/stepgen/stepgen.h"
#include "modules/digitalPin/digitalPin.h"


/***********************************************************************
*                STRUCTURES AND GLOBAL VARIABLES                       *
************************************************************************/

// boolean
bool configError = false;

// pointers to objects with global scope
pruThread* servoThread;
RemoraComms* comms;
Stepgen *stepGenerators[JOINTS] = {};
DigitalPin *inputs[sizeof(txData_t::inputs)*8] = {};
DigitalPin *outputs[sizeof(rxData_t::outputs)*8] = {};

// Json config file stuff
const char defaultConfig[] = DEFAULT_CONFIG;

// 512 bytes of metadata in front of actual JSON file
typedef struct
{
  uint32_t crc32;           // crc32 of JSON
  uint32_t length;          // length in words for CRC calculation
  uint32_t jsonLength;      // length in of JSON config in bytes
  uint8_t padding[500];
} metadata_t;
#define METADATA_LEN    512

volatile bool newJson;
uint32_t crc32;
string strJson;
DynamicJsonDocument doc(JSON_BUFF_SIZE);
JsonObject thread;
JsonObject module;


static void set_clock_khz(void);
void EthernetInit();
void udpServerInit();
void EthernetTasks();
void udp_data_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);


/* Network */
extern uint8_t mac[6];
static ip_addr_t g_ip;
static ip_addr_t g_mask;
static ip_addr_t g_gateway;

/* LWIP */
struct netif g_netif;


int8_t checkJson()
{
    metadata_t* meta = (metadata_t*)(XIP_BASE + JSON_UPLOAD_ADDRESS);
    uint32_t* json = (uint32_t*)(XIP_BASE + JSON_UPLOAD_ADDRESS + METADATA_LEN);

    uint32_t table[256];
    crc32::generate_table(table);
    int mod, padding;

    // Check length is reasonable
    if (meta->length > (32/4) * FLASH_SECTOR_SIZE)
    {
        newJson = false;
        printf("JSON Config length incorrect\n");
        return -1;
    }

    // for compatability with STM32 hardware CRC32, the config is padded to a 32 byte boundary
    mod = meta->jsonLength % 4;
    if (mod > 0)
    {
        padding = 4 - mod;
    }
    else
    {
        padding = 0;
    }
    printf("mod = %d, padding = %d\n", mod, padding);

    // Compute CRC
    char* ptr = (char *)(XIP_BASE + JSON_UPLOAD_ADDRESS + METADATA_LEN);
    for (int i = 0; i < meta->jsonLength + padding; i++)
    {
        crc32 = crc32::update(table, crc32, ptr, 1);
        ptr++;
    }

    printf("Length (words) = %d\n", meta->length);
    printf("JSON length (bytes) = %d\n", meta->jsonLength);
    printf("crc32 = %x\n", crc32);

    // Check CRC
    if (crc32 != meta->crc32)
    {
        newJson = false;
        printf("JSON Config file CRC incorrect\n");
        return -1;
    }

    // JSON is OK, don't check it again
    newJson = false;
    printf("JSON Config file received Ok\n");
    return 1;
}


void moveJson()
{
    uint8_t pages;
    uint32_t i = 0;
    metadata_t* meta = (metadata_t*)(XIP_BASE + JSON_UPLOAD_ADDRESS);;

    uint16_t jsonLength = meta->jsonLength;

    // erase the old JSON config file
    uint32_t status = save_and_disable_interrupts();
    flash_range_erase(JSON_STORAGE_ADDRESS, (32/4) * FLASH_SECTOR_SIZE);
    restore_interrupts(status);

    // how many pages are needed to be written. The first 4 bytes of the storage location will contain the length of the JSON file
    pages = (meta->jsonLength + 4) / FLASH_PAGE_SIZE;
    if ((meta->jsonLength + 4) / FLASH_PAGE_SIZE > 0)
    {
        pages++;
    }

    printf("pages = %d\n", pages);

    uint8_t data[pages * 256] = {0};

    // store the length of the file in the 0th word
    data[0] = (uint8_t)((jsonLength & 0x00FF));
    data[1] = (uint8_t)((jsonLength & 0xFF00) >> 8);

    //The buffer argument points to the data to be written, which is of size size.
    //This size must be a multiple of the "page size", which is defined as the constant FLASH_PAGE_SIZE, with a value of 256 bytes.

    for (i = 0; i < jsonLength; i++)
    {
        data[i + 4] = *((uint8_t*)(XIP_BASE + JSON_UPLOAD_ADDRESS + METADATA_LEN + i));
    }

    status = save_and_disable_interrupts();
    flash_range_program(JSON_STORAGE_ADDRESS, data, (pages * 256));
    restore_interrupts(status);
}


void jsonFromFlash()
{
    printf("\n1. Loading JSON configuration file from Flash memory\n");

    // read byte 0 to determine length to read
    uint32_t jsonLength = *(uint32_t*)(XIP_BASE + JSON_STORAGE_ADDRESS);
    if (jsonLength == 0xFFFFFFFF)
    {
        printf("Flash storage location is empty - no config file\n");
        printf("Using default configuration\n\n");
        strJson = defaultConfig;
    }
    else
    {
        const char *p = (const char*)(XIP_BASE + JSON_STORAGE_ADDRESS + 4);
        strJson = std::string(p, p+jsonLength);
    }
    printf("\n%s\n\n", strJson.c_str());
}


void deserialiseJSON()
{
    printf("\n2. Parsing JSON configuration file\n");

    const char *json = strJson.c_str();

    // parse the json configuration file
    DeserializationError error = deserializeJson(doc, json);

    printf("Config deserialisation - ");

    switch (error.code())
    {
        case DeserializationError::Ok:
            printf("Deserialization succeeded\n");
            break;
        case DeserializationError::InvalidInput:
            printf("Invalid input!\n");
            configError = true;
            break;
        case DeserializationError::NoMemory:
            printf("Not enough memory\n");
            configError = true;
            break;
        default:
            printf("Deserialization failed\n");
            configError = true;
            break;
    }

    printf("\n");
}


void loadModules()
{
    printf("\n4. Loading modules\n");

    // Ethernet communication monitoring
    comms = new RemoraComms();
    servoThread->registerModule(comms);

    if (configError) return;

    JsonArray Modules = doc["Modules"];

    // create objects from JSON data
    for (JsonArray::iterator it=Modules.begin(); it!=Modules.end(); ++it)
    {
        module = *it;

        const char* type = module["Type"];
        if (!strcmp(type,"Stepgen"))
        {
            int joint = module["Joint Number"];
            if (NULL != stepGenerators[joint])
            {
                printf("ERROR!  Joint Number %d specified more than once.\n", joint);
                configError = true;
            }
            stepGenerators[joint] = Stepgen::load(module);
        }
        else if (!strcmp(type,"Blink"))
        {
            createBlink();
        }
        else if (!strcmp(type,"Digital Pin"))
        {
            const char *mode = module["Mode"];
            int dataBit = module["Data Bit"];
            if (!strcmp(mode, "Input"))
                inputs[dataBit] = DigitalPin::load(module);
            else if (!strcmp(mode, "Output"))
                outputs[dataBit] = DigitalPin::load(module);
        }
        else if (!strcmp(type,"Spindle PWM"))
        {
            //createSpindlePWM();
        }
    }
}


void core1_entry()
{
    printf("\nRemora for RP2040 starting (core1)...\n\r");
    printf("\n## Entering SETUP state\n\n");

    jsonFromFlash();
    deserialiseJSON();
    createThreads();
    loadModules();

    printf("\n## Entering START state\n");

    printf("\nStarting the SERVO thread\n");
    servoThread->startThread();

    for (;;)
    {
        printf("\n## Entering IDLE state\n");
        do
        {
            servoThread->run();
        }
        while (!comms->getStatus());

        printf("\n## Entering RUNNING state\n");
        do
        {
            servoThread->run();
        }
        while (comms->getStatus());

        printf("\n## Entering RESET state\n");
        // Stop all movement
        for (int i = 0; i < JOINTS; i++)
            stepGenerators[i]->frequencyCommand(PRU_BASEFREQ, false, 0);
    }
}

int main()
{
    // Network configuration
    IP4_ADDR(&g_ip, 10, 10, 10, 10);
    IP4_ADDR(&g_mask, 255, 255, 255, 0);
    IP4_ADDR(&g_gateway, 10, 10, 10, 1);

    stdio_init_all();

    sleep_ms(1000 * 3); // wait for 3 seconds

    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\nRemora for RP2040 starting (core0, clock=%d)...\n\n\r", clock_get_hz(clk_sys));

    EthernetInit();
    udpServerInit();
    IAP_tftpd_init();

    // launch main Remora code on the second core
    multicore_launch_core1(core1_entry);
    /* Grant high bus priority to the second core. */
    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_PROC1_BITS;


    while (1)
    {
        EthernetTasks();
        sys_check_timeouts();

        if (newJson)
        {
            printf("\n\nChecking new configuration file\n");
            if (checkJson() > 0)
            {
            printf("Moving new config file to Flash storage\n");
            moveJson();

            // force a reset to load new JSON configuration
            printf("Forceing a reboot now....\n");
            watchdog_reboot(0, SRAM_END, 0);
            for (;;) {
                __wfi();
            }
            }
        }
    }
}


void EthernetInit()
{
    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    // Set ethernet chip MAC address
    setSHAR(mac);
    ctlwizchip(CW_RESET_PHY, 0);

    // Initialize LWIP in NO_SYS mode
    lwip_init();

    netif_add(&g_netif, &g_ip, &g_mask, &g_gateway, NULL, netif_initialize, netif_input);
    g_netif.name[0] = 'e';
    g_netif.name[1] = '0';

    // Assign callbacks for link and status
    netif_set_link_callback(&g_netif, netif_link_callback);
    netif_set_status_callback(&g_netif, netif_status_callback);

    // MACRAW socket open
    int8_t retval = socket(SOCKET_MACRAW, Sn_MR_MACRAW, PORT_LWIPERF, 0x00);
    if (retval < 0)
    {
        printf(" MACRAW socket open failed\n");
    }

    // Set the default interface and bring it up
    netif_set_link_up(&g_netif);
    netif_set_up(&g_netif);
}


void EthernetTasks()
{
    uint16_t pack_len = 0;
    getsockopt(SOCKET_MACRAW, SO_RECVBUF, &pack_len);
    if (0 == pack_len)
        return;

    static uint8_t pack[ETHERNET_MTU];
    pack_len = recv_lwip(SOCKET_MACRAW, (uint8_t *)pack, pack_len);
    if (0 == pack_len)
    {
        printf(" No packet received\n");
        return;
    }

    struct pbuf *p = pbuf_alloc(PBUF_RAW, pack_len, PBUF_POOL);
    if (NULL == p)
    {
        printf("discarding packet: buffer pool exhausted\n");
        return;
    }

    pbuf_take(p, pack, pack_len);
    LINK_STATS_INC(link.recv);
    if (ERR_OK == g_netif.input(p, &g_netif))
        return;

    pbuf_free(p);
}


void udpServerInit(void)
{
    struct udp_pcb *upcb = udp_new();
    if (ERR_OK != udp_bind(upcb, &g_ip, 27181))  // 27181 is the server UDP port
    {
        udp_remove(upcb);
        return;
    }
    udp_recv(upcb, udp_data_callback, NULL);
}

void reply(struct udp_pcb *upcb, const ip_addr_t *addr, u16_t port, char* data, size_t len)
{
    struct pbuf *txBuf = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    pbuf_take(txBuf, data, len);

    // Connect to the remote client
    udp_connect(upcb, addr, port);
    udp_send(upcb, txBuf);

    // free the UDP connection, so we can accept new clients
    udp_disconnect(upcb);
    pbuf_free(txBuf);
}

void udp_data_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    static rxData_t rxBuffer = {};
    memcpy(&rxBuffer.rxBuffer, p->payload, p->len);
    pbuf_free(p);

    switch (rxBuffer.header)
    {
    case PRU_READ:
        {
            txData_t txBuffer = {};
            txBuffer.header = PRU_DATA;
            comms->dataReceived();

            for (int i = 0; i < JOINTS; i++)
            {
                if (NULL == stepGenerators[i])
                    continue;
                txBuffer.jointFeedback[i] = stepGenerators[i]->jointFeedback();
            }
            for (int i = 0; i < sizeof(inputs)/sizeof(inputs[0]); ++i)
            {
                if (NULL == inputs[i])
                    continue;
                if (inputs[i]->read())
                    txBuffer.inputs |= (1 << i);
            }

            reply(upcb, addr, port, (char*)&txBuffer.txBuffer, BUFFER_SIZE);
        }
        break;

    case PRU_WRITE:
        comms->dataReceived();

        for (int i = 0; i < JOINTS; i++)
        {
            if (NULL == stepGenerators[i])
                continue;
            bool isEnabled = (rxBuffer.jointEnable & (1 << i)) != 0;
            int32_t frequencyCmd = rxBuffer.jointFreqCmd[i];
            stepGenerators[i]->frequencyCommand(PRU_BASEFREQ, isEnabled, frequencyCmd);
        }
        for (int i = 0; i < sizeof(outputs)/sizeof(outputs[0]); ++i)
        {
            if (NULL == outputs[i])
                continue;
            outputs[i]->write(rxBuffer.outputs & (1 << i));
        }

        int32_t header = PRU_ACKNOWLEDGE;
        reply(upcb, addr, port, (char*)&header, sizeof(header));
        break;
    }
}