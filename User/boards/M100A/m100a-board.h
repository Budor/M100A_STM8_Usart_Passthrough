#ifndef __M100A_BOARD_H__
#define __M100A_BOARD_H__

#define WORK_CONFIG_MODE                            0
#define WORK_STANDARD_MODE                          1

#define LORA_MAC_PUBLIC_SYNCWORD                    0x34
#define LORA_MAC_PRIVATE_SYNCWORD                   0x12
 
#define CONFIG_PARAM_SIZE                           27

#define MAX_RX_WINDOW                               0   // 一直接收

#define SEND_OK_FLAG			            0xFF
#define SEND_FAIL_FLAG                              0x00


#define MAC_MAXPAYLOAD                              255
#define SERIAL_PACKAGE_MAX_SIZE                     255

enum SerialMessageType {
    SET_M100A = 0xF1,
    SET_M100A_ACK = 0xF2,
    GET_M100A = 0xF3,
    GET_M100A_ACK = 0xF4
};

enum ParseErrorCode {
    FINISH = 0,
    CRC_ERROR,
    MSG_TYPE_ERROR,
    PYLOAD_ERROR,
    TO_ADDRESS_ERROR,
};

typedef struct serial_frame_t_
{
    uint8_t type;
    uint8_t payload_length;
    uint8_t* pdata;
    uint16_t crc16;
} serial_frame_t;

typedef enum e_mac_status
{
    LOFIMAC_STATUS_OK,
    LOFIMAC_STATUS_BUSY
} mac_status_t;

struct prepare_frame_t_
{
    uint32_t rx_window1_delay;
    uint32_t rx_window2_delay;
    uint8_t lofi_mac_buffer[MAC_MAXPAYLOAD];
    uint16_t lofi_mac_buffer_pkt_len;
    uint8_t uart_buffer[MAC_MAXPAYLOAD];
    uint16_t uart_buffer_pkt_len;
    bool life_state_tx;
    bool sensor_state_tx;
    bool sensor_state_tx_ack;
    bool node_ack_requested;
    bool ack_timeout_retry;
};

struct m100a_config_t_
{
// RF Config
    uint32_t rf_frequency;
    uint8_t rf_syncword;
    uint8_t rf_preamble_len;
    uint8_t rf_tx_power;
    uint8_t rf_datarate;        // SF 
    uint8_t rf_bandwidth;
    uint8_t rf_header_mode;     //1: Implicit, 0: Explicit(明确)      
    uint8_t rf_coderate;
    bool rf_crc_enable;
    bool rf_rx_iq_signal;
    bool rf_tx_iq_signal;

// Usart Config
    uint8_t com_baud;
    uint8_t com_parity;
    uint8_t com_data_bits;
    uint8_t com_stop_bits;

    uint8_t work_mode;                 
    uint8_t deveui[8];           
    uint8_t EnableDebugPrintf;
};


typedef struct prepare_frame_t_ *prepare_frame_t;
typedef struct m100a_config_t_ *m100a_config_t;
void print_message(const char *title, uint8_t *message, uint8_t length, uint8_t format);

void node_device_process(void);

#endif  // __M100A_BOARD_H__