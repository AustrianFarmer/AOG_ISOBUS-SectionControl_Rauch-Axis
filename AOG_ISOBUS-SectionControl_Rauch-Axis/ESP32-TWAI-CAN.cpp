#include "ESP32-TWAI-CAN.hpp"


//---- Filtermask for CAN IDs -----------------------------------------------------------------------------------

uint32_t ID[] = { 0x1CE726AA, 0x0CCBF7AA };    //IDs to be filtered. Change/Add/Remove as required.

int8_t NumberOfID = sizeof(ID) / sizeof(ID[0]);
int8_t Filter = 0;
uint32_t Filtermask; 


void Filtermask_CAN_ID() {

 for (uint8_t i = 0; i < 29; i++)
  {
   Filter = 0;
     for (uint8_t n = 0; n < NumberOfID; n++)
      {
        Filter += bitRead(ID[n], i);
      }
    if(Filter == 0 || Filter == NumberOfID) bitWrite(Filtermask, i, 0);
    else bitWrite(Filtermask, i, 1);
  }

//  Serial.print("Acceptance Code   ;");
//  Serial.println(ID[0],HEX);
//  Serial.print("Filtermask   ;");
//  Serial.println(Filtermask,HEX);
//  Serial.print("Number   ;");
//  Serial.println(NumberOfID,DEC);
}
//-----------------------------------------------------------------------------------------------------------------



void TwaiCAN::setSpeed(TwaiSpeed twaiSpeed) {
    if(twaiSpeed < TWAI_SPEED_SIZE) speed = twaiSpeed;
}

TwaiSpeed TwaiCAN::convertSpeed(uint16_t canSpeed) { 
    TwaiSpeed actualSpeed = getSpeed();
  switch(canSpeed) {
        default: break;
        #if (SOC_TWAI_BRP_MAX > 256)
        case 1:     actualSpeed = TWAI_SPEED_1KBPS;     break;
        case 5:     actualSpeed = TWAI_SPEED_5KBPS;     break;
        case 10:    actualSpeed = TWAI_SPEED_10KBPS;    break;
        #endif
        #if (SOC_TWAI_BRP_MAX > 128) || (CONFIG_ESP32_REV_MIN_FULL >= 200)
        case 13:    // Just to handle those who would round upwards..
        case 12:    actualSpeed = TWAI_SPEED_12_5KBPS;  break;
        case 16:    actualSpeed = TWAI_SPEED_16KBPS;    break;
        case 20:    actualSpeed = TWAI_SPEED_20KBPS;    break;
        #endif
    case 100:   actualSpeed = TWAI_SPEED_100KBPS;   break;
    case 125:   actualSpeed = TWAI_SPEED_125KBPS;   break;
    case 250:   actualSpeed = TWAI_SPEED_250KBPS;   break;
    case 500:   actualSpeed = TWAI_SPEED_500KBPS;   break;
    case 800:   actualSpeed = TWAI_SPEED_800KBPS;   break;
    case 1000:  actualSpeed = TWAI_SPEED_1000KBPS;  break;
  }
    return actualSpeed;
}

void TwaiCAN::setTxQueueSize(uint16_t txQueue) { if(txQueue != 0xFFFF) txQueueSize = txQueue; }
void TwaiCAN::setRxQueueSize(uint16_t rxQueue) { if(rxQueue != 0xFFFF) rxQueueSize = rxQueue; }

bool TwaiCAN::getStatusInfo() {
    return ESP_OK == twai_get_status_info(&status);
}

uint32_t TwaiCAN::inTxQueue() {
    uint32_t ret = 0;
    if(getStatusInfo()) {
        ret = status.msgs_to_tx;
    }
    return ret;
};

uint32_t TwaiCAN::inRxQueue() {
    uint32_t ret = 0;
    if(getStatusInfo()) {
        ret = status.msgs_to_rx;
    }
    return ret;
};


bool TwaiCAN::setPins(int8_t txPin, int8_t rxPin) {
    bool ret = !init;

    if(txPin >= 0) tx = txPin;
    else ret = false;
    if(rxPin >= 0) rx = rxPin;
    else ret = false;

    LOG_TWAI("Wrong pins or CAN bus running already!");
    return ret;
}

bool TwaiCAN::begin(TwaiSpeed twaiSpeed, 
                    int8_t txPin, int8_t rxPin,
                    uint16_t txQueue, uint16_t rxQueue,
                    twai_filter_config_t*  fConfig,
                    twai_general_config_t* gConfig,
                    twai_timing_config_t*  tConfig)
                    {

    bool ret = false;
    if(end()) {
        init = true;
        setSpeed(twaiSpeed);
        setPins(txPin, rxPin);
        setTxQueueSize(txQueue);
        setRxQueueSize(rxQueue);

        twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL, .tx_io = (gpio_num_t) tx, .rx_io = (gpio_num_t) rx, \
                                                .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,      \
                                                .tx_queue_len = txQueueSize, .rx_queue_len = rxQueueSize,       \
                                                .alerts_enabled = TWAI_ALERT_NONE,  .clkout_divider = 0,        \
                                                .intr_flags = ESP_INTR_FLAG_LEVEL1};

        twai_timing_config_t t_config[TWAI_SPEED_SIZE] = {
            #if (SOC_TWAI_BRP_MAX > 256)
            TWAI_TIMING_CONFIG_1KBITS(),
            TWAI_TIMING_CONFIG_5KBITS(),
            TWAI_TIMING_CONFIG_10KBITS(),
            #endif
            #if (SOC_TWAI_BRP_MAX > 128) || (CONFIG_ESP32_REV_MIN_FULL >= 200)
            TWAI_TIMING_CONFIG_12_5KBITS(),
            TWAI_TIMING_CONFIG_16KBITS(),
            TWAI_TIMING_CONFIG_20KBITS(),
            #endif
            TWAI_TIMING_CONFIG_100KBITS(),
            TWAI_TIMING_CONFIG_125KBITS(),
            TWAI_TIMING_CONFIG_250KBITS(),
            TWAI_TIMING_CONFIG_500KBITS(),
            TWAI_TIMING_CONFIG_800KBITS(),
            TWAI_TIMING_CONFIG_1MBITS()
        };


//        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        Filtermask_CAN_ID(); 
        twai_filter_config_t f_config = {.acceptance_code = ID[0] <<3, .acceptance_mask = Filtermask <<3, .single_filter = true};    //Filter CAN ID



        if(!gConfig) gConfig = &g_config;
        if(!tConfig) tConfig = &t_config[speed];
        if(!fConfig) fConfig = &f_config;

            //Install TWAI driver
        if (twai_driver_install(gConfig, tConfig, fConfig) == ESP_OK) {
            LOG_TWAI("Driver installed");
        } else {
            LOG_TWAI("Failed to install driver");
        }

        //Start TWAI driver
        if (twai_start() == ESP_OK) {
            LOG_TWAI("Driver started");
            ret = true;
        } else {
            LOG_TWAI("Failed to start driver");
        }
        if(!ret) end();
    }
    return ret;
}


bool TwaiCAN::end() {
    bool ret = false;
    if(init) {
        //Stop the TWAI driver
        if (twai_stop() == ESP_OK) {
            LOG_TWAI("Driver stopped\n");
            ret = true;
        } else {
            LOG_TWAI("Failed to stop driver\n");
        }

        //Uninstall the TWAI driver
        if (twai_driver_uninstall() == ESP_OK) {
            LOG_TWAI("Driver uninstalled\n");
            ret &= true;
        } else {
            LOG_TWAI("Failed to uninstall driver\n");
            ret &= false;
        }
        init = !ret;
    } else ret = true;
    return ret;
}

TwaiCAN ESP32Can;
