#include "board.h"
#include "stm8l_eeprom.h"

#define PUTCHAR_PROTOTYPE int putchar (int c)
#define USE_DEFAULT_CONFIG      0

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };

static TimerEvent_t tx_delay_timer;
static TimerEvent_t rx_window_timer1;
static TimerEvent_t rx_window_timer2;

static RadioEvents_t radio_events;

struct prepare_frame_t_ prepare_frame_malloc = {0};
prepare_frame_t prepare_frame = &prepare_frame_malloc;

struct m100a_config_t_ m100a_config_malloc = {0};
m100a_config_t m100a_config = &m100a_config_malloc;

enum ParseErrorCode parse_serial_frame(uint8_t *frame, uint8_t frame_length);

static enum e_mac_state
{
    MAC_IDLE          = 0x00,
    MAC_TX_RUNNING    = 0x01,
    MAC_RX            = 0x02,
    MAC_ACK_REQ       = 0x04,
    MAC_ACK_RETRY     = 0x08,
    MAC_TX_DELAYED    = 0x10,
}mac_state;

static enum e_device_state
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_RECV,
    DEVICE_STATE_SLEEP
}device_state;

bool usart_received_data_flag = FALSE;

void usart_init( void )
{
    uint32_t baudrate;
    USART_DeInit(UART_1);
    
    UartInit( &Uart1, UART_1, UART1_TX, UART1_RX );

    switch(m100a_config->com_baud)
    {
    case 0:  baudrate = 1200; DEBUG("UART1 Baud: 1200\r\n"); break;
    case 1:  baudrate = 2400; DEBUG("UART1 Baud: 2400\r\n"); break;
    case 2:  baudrate = 4800; DEBUG("UART1 Baud: 4800\r\n"); break;        
    case 3:  baudrate = 9600; DEBUG("UART1 Baud: 9600\r\n"); break;       
    case 4:  baudrate = 19200; DEBUG("UART1 Baud: 19200\r\n"); break;  
    case 5:  baudrate = 115200; DEBUG("UART1 Baud: 115200\r\n"); break;
    default: baudrate = 115200; DEBUG("UART1 Baud: 115200\r\n"); break;
    }
    DEBUG("UART1 Data Bits: %d\r\n", m100a_config->com_data_bits);
    DEBUG("UART1 Stop Bits: %d\r\n", m100a_config->com_stop_bits);
    DEBUG("UART1 Parity: %d\r\n", m100a_config->com_parity);

    UartConfig( &Uart1, RX_TX, baudrate, 
               (WordLength_t)m100a_config->com_data_bits, 
               (StopBits_t)m100a_config->com_stop_bits, 
               (Parity_t)m100a_config->com_parity, NO_FLOW_CTRL );
}

void clear_usart_received_data_flag(void)
{
    usart_received_data_flag = FALSE;
}

bool check_usart_received_data_flag(void)
{
    return usart_received_data_flag;
}

/**
  * @brief Retargets the C library printf function to the USART.
  * @param[in] c Character to send
  * @retval char Character sent
  * @par Required preconditions:
  * - None
  */
PUTCHAR_PROTOTYPE
{
    if(m100a_config->EnableDebugPrintf)
    {
        /* Write a character to the USART */
        USART_SendData8(USART3, c);
        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
    }
    return (c);
}

void print_message(const char *title, uint8_t *message, uint8_t length, uint8_t format)
{
    DEBUG(title);
    if(length == 0) 
    { 
        DEBUG("[]\r\n");  
    }
    else
    {
        DEBUG("%dB\r\n[", length);
        for(uint8_t i = 0; i < (length - 1); i++)
        {
            if(format == 'X')
            {
                DEBUG("%02X ", message[i]);
            }
            else if(format == 'x')
            {
                DEBUG("%02x ", message[i]);
            }
            else if(format == 'd')
            {
                DEBUG("%d ", message[i]);
            }
            else
            {
                DEBUG("%d ", message[i]);
            }
        }
        if(format == 'X')
        {
            DEBUG("%02X]\r\n", message[length - 1]);
        }
        else if(format == 'x')
        {
            DEBUG("%02x]\r\n", message[length - 1]);
        }
        else if(format == 'd')
        {
            DEBUG("%d]\r\n", message[length - 1]);
        }
        else
        {
            DEBUG("%d]\r\n", message[length - 1]);
        }
    }
}

void enable_usart_state(bool state)
{
    if(state)
    {
        USART_Cmd(USART1, ENABLE);
    }
    else
    {
        USART_Cmd(USART1, DISABLE);
    }
}

void send_rf_data(uint8_t *data, uint8_t size)
{
    uint8_t buffer[255] = {0};
    
    buffer[0] = 0xFE;
    buffer[1] = size;
    memcpy1(buffer+2, data, size);
    buffer[2+size] = 0xEF;
    
    UartPutBuffer(&Uart1, buffer, size+3);
}

void rf_send_ok_ack(void)
{
    uint8_t buffer[4] = {0xFE, 0x01, SEND_OK_FLAG, 0xEF};

    UartPutBuffer( &Uart1, buffer, 4 );
}

void rf_send_fail_ack(void)
{
    uint8_t buffer[4] = {0xFE, 0x01, SEND_FAIL_FLAG, 0xEF};

    UartPutBuffer( &Uart1, buffer, 4 );
}

void use_config_param(uint8_t *param)	// param length must is CONFIG_PARAM_SIZE
{
    uint8_t index = 0;
    
    index = 0;
    memcpy1((uint8_t *)&m100a_config->rf_frequency, param+index, 4);
    index += 4;
    m100a_config->rf_tx_power = param[index];
    index += 1;
    m100a_config->rf_datarate = param[index];
    index += 1;
    m100a_config->rf_bandwidth = param[index];
    index += 1;
    m100a_config->rf_syncword = param[index];
    index += 1;
    memcpy1((uint8_t *)&m100a_config->deveui, param+index, 8);
    index += 8;
    m100a_config->EnableDebugPrintf = param[index];
    index += 1;
    
    m100a_config->rf_preamble_len = param[index];
    index += 1;
    m100a_config->rf_header_mode = param[index];
    index += 1;
    m100a_config->rf_coderate = param[index];
    index += 1;
    m100a_config->rf_crc_enable = param[index]?TRUE:FALSE;
    index += 1;
    m100a_config->com_baud = param[index];
    index += 1;
    m100a_config->com_parity = param[index];
    index += 1;
    m100a_config->com_data_bits = param[index];
    index += 1;
    m100a_config->com_stop_bits = param[index];
    index += 1;
    m100a_config->rf_rx_iq_signal = param[index]?TRUE:FALSE;
    index += 1;
    m100a_config->rf_tx_iq_signal = param[index]?TRUE:FALSE;
}

uint8_t set_m100a_config(uint8_t *buffer, uint8_t size)
{  
    uint8_t result = 0;
    
    if(size == CONFIG_PARAM_SIZE)
    {
        if( save_config_param(buffer, size) )
        {
            result = 1;
            DEBUG("Save config Failed\r\n");
        }
        else
        {
            result = 0;
            DEBUG("Save config OK\r\n");
        }
    }
    else
    {
        result = 1;
        DEBUG("set all param size != %d\r\n", CONFIG_PARAM_SIZE);
    }
    
    return result;
}

void set_rf_buffer(uint8_t *buffer, uint8_t size)
{
    prepare_frame->lofi_mac_buffer_pkt_len = size;
    memcpy1(prepare_frame->lofi_mac_buffer, buffer, prepare_frame->lofi_mac_buffer_pkt_len);
}

void config_mode_process(uint8_t *data, uint8_t size)
{
    parse_serial_frame(data+1, size-2);
}

void standard_mode_process(uint8_t *data, uint8_t size)
{
    print_message("Serial Data:", data, size, 'X');
    set_rf_buffer(data+2, size-3);
    usart_received_data_flag = TRUE;
}

void usart_irq_notify( UartNotifyId_t id )
{
    uint8_t data;
    static uint8_t start_flag = 0, frame_length = 0, frame_index = 0;
    
    if( id == UART_NOTIFY_RX )
    {
        if( UartGetChar( &Uart1, &data ) == 0 )
        {
            if(prepare_frame->uart_buffer_pkt_len >= SERIAL_PACKAGE_MAX_SIZE)
            {
                prepare_frame->uart_buffer_pkt_len = 0;
                frame_length = 0;
                frame_index = 0;
                start_flag = 0;
                rf_send_fail_ack();
                DEBUG("Serial Data >= 255, set buffer_size = 0.\r\n");
            }
            
            if(( start_flag == 0) && ( data == 0xFE )) { start_flag = 1; }
            
            if( start_flag ) 
            { 
                prepare_frame->uart_buffer[prepare_frame->uart_buffer_pkt_len++] = data;
                frame_index++;
            }
            
            if(m100a_config->work_mode == WORK_STANDARD_MODE)
            {
              if(frame_index == 2) { frame_length = data + 3; }
            }
            else if(m100a_config->work_mode == WORK_CONFIG_MODE)
            {
                if(frame_index == 3) { frame_length = data + 6; }
            }
            
            if((frame_length != 0) && (frame_length == frame_index))
            {
                if( data == 0xEF )
                {
                    if(m100a_config->work_mode == WORK_CONFIG_MODE)
                    {
                        config_mode_process(prepare_frame->uart_buffer, prepare_frame->uart_buffer_pkt_len);
                    }
                    else if(m100a_config->work_mode == WORK_STANDARD_MODE)
                    {
                        standard_mode_process(prepare_frame->uart_buffer, prepare_frame->uart_buffer_pkt_len);
                    }
                    else
                    {}
                }
                else
                {
                    DEBUG("Frame end is error:%02X\r\n", data);
                    print_message("\r\nStandard Serial data error:", prepare_frame->uart_buffer, prepare_frame->uart_buffer_pkt_len, 'X');
                }
                prepare_frame->uart_buffer_pkt_len = 0;
                frame_length = 0;
                frame_index = 0;
                start_flag = 0;
            }
        }
    }
}

void debug_print_com_init(void)
{
    UartInit( &Uart3, UART_3, UART3_TX, UART3_RX );
    
    UartConfig( &Uart3, RX_TX, 115200, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL );
}

void m100a_init( void )
{  
    usart_init( );

    Uart1.IrqNotify = usart_irq_notify;
}

bool get_config_param(void)
{
    uint8_t buffer[CONFIG_PARAM_SIZE] = {0};
  
    if(SET_CONFIGURE_FLAG != FLASH_ReadByte((uint32_t)EEPROM_FLAG_ADDR))
    {
        m100a_config->com_baud = 5;
        m100a_config->com_data_bits = 0;
        m100a_config->com_stop_bits = 0;
        m100a_config->com_parity = 0;
        m100a_init();
        enable_usart_state(TRUE);
        DEBUG("The M100A is not config, so stop work in config mode.\r\nPlease use com config: 115200 8E1 no parity config to set\r\n");
        while(1);
    }
    else
    {
        get_eeprom_data(buffer, CONFIG_PARAM_SIZE);
        use_config_param(buffer);
        DEBUG("Get Config Finish\r\n");        
    }
    return TRUE;
}

//param: RFLR_MODEMCONFIG1_IMPLICITHEADER_ON or RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF
void set_lora_header_mode(uint8_t mode)
{
    Radio.SetModem(MODEM_LORA);
    
    if(mode == RFLR_MODEMCONFIG1_IMPLICITHEADER_ON)
    {
      Radio.Write( REG_LR_MODEMCONFIG1, 
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_ON );
    }
    else
    {
      Radio.Write( REG_LR_MODEMCONFIG1, 
                         ( SX1276Read( REG_LR_MODEMCONFIG1 ) &
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK ) |
                           RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF );
    }
}

void set_lora_syncword(uint8_t syncword)
{
    Radio.SetModem(MODEM_LORA);

    Radio.Write(REG_LR_SYNCWORD, syncword);
}

mac_status_t send_frame_on_channel(uint32_t freq, uint32_t datarate, uint32_t bandwidth, uint8_t tx_power)
{
    Radio.SetChannel( freq );
    Radio.SetMaxPayloadLength( MODEM_LORA, prepare_frame->lofi_mac_buffer_pkt_len );
    Radio.SetTxConfig( MODEM_LORA, tx_power, 0, m100a_config->rf_bandwidth, 
                      datarate, m100a_config->rf_coderate, m100a_config->rf_preamble_len, 
                      FALSE, m100a_config->rf_crc_enable, FALSE, 0, m100a_config->rf_tx_iq_signal, 10e6 );

    Radio.Send( prepare_frame->lofi_mac_buffer, prepare_frame->lofi_mac_buffer_pkt_len );

    mac_state |= MAC_TX_RUNNING;
    
    switch(bandwidth)
    {
    case 0:
        DEBUG("[%d00000Hz, SF%dBW125K, SyncWord:0x%02X, Power:%ddBm]", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_syncword, tx_power);
        break;
    case 1:
        DEBUG("[%d00000Hz, SF%dBW250K, SyncWord:0x%02X, Power:%ddBm]", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_syncword, tx_power);
        break;
    case 2:
        DEBUG("[%d00000Hz, SF%dBW500K, SyncWord:0x%02X, Power:%ddBm]", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_syncword, tx_power);
        break;
    default:
        DEBUG("[%d00000Hz, SF%dBW%d, SyncWord:0x%02X, Power:%ddBm]", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_bandwidth, m100a_config->rf_syncword, tx_power);
        break;
    }
    
    print_message("Send Frame:", prepare_frame->lofi_mac_buffer, prepare_frame->lofi_mac_buffer_pkt_len, 'X');

    return LOFIMAC_STATUS_OK;
}

void rx_window_setup(uint32_t freq, uint32_t datarate)
{
    if(Radio.GetStatus() == RF_IDLE)
    {
        Radio.SetChannel(freq);

        Radio.SetRxConfig( MODEM_LORA, m100a_config->rf_bandwidth, datarate, m100a_config->rf_coderate, 
                          0, m100a_config->rf_preamble_len, 5, FALSE, 0, 
                          m100a_config->rf_crc_enable, FALSE, 0, m100a_config->rf_rx_iq_signal, TRUE ); 

        Radio.SetMaxPayloadLength( MODEM_LORA, MAC_MAXPAYLOAD );
        
        Radio.Rx(MAX_RX_WINDOW);
    }
    switch(m100a_config->rf_bandwidth)
    {
    case 0:
        DEBUG("[%d00000Hz, SF%dBW125K, SyncWord:0x%02X] Start RF Recv...\r\n", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_syncword);
        break;
    case 1:
        DEBUG("[%d00000Hz, SF%dBW250K, SyncWord:0x%02X] Start RF Recv...\r\n", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_syncword);
        break;
    case 2:
        DEBUG("[%d00000Hz, SF%dBW500K, SyncWord:0x%02X] Start RF Recv...\r\n", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_syncword);
        break;
    default:
        DEBUG("[%d00000Hz, SF%dBW%d, SyncWord:0x%02X] Start RF Recv...\r\n", (uint16_t)(freq / 100000), (uint8_t)datarate, m100a_config->rf_bandwidth, m100a_config->rf_syncword);
        break;
    }
}

uint32_t set_next_channel(void)
{
    uint32_t next_tx_delay = 0;

    // 计算下一次发送等待时间

    return next_tx_delay;
}

mac_status_t schedule_tx(void)
{
    uint32_t duty_cycle_time_off = 0;

    duty_cycle_time_off = set_next_channel();

    if(duty_cycle_time_off)
    {
        mac_state |= MAC_TX_DELAYED;
        TimerSetValue(&tx_delay_timer, duty_cycle_time_off);
        TimerStart(&tx_delay_timer);
        return LOFIMAC_STATUS_OK;
    }
    else
    {
        return send_frame_on_channel(m100a_config->rf_frequency, Datarates[m100a_config->rf_datarate], m100a_config->rf_bandwidth, TxPowers[m100a_config->rf_tx_power]);
    }
}

void build_serial_frame(uint8_t msg_type, uint8_t *payload, uint8_t pyaload_length)
{
    uint16_t crc16 = 0;

    prepare_frame->uart_buffer[0] = 0xFE;
    prepare_frame->uart_buffer[1] = msg_type;
    prepare_frame->uart_buffer[2] = pyaload_length;
    memcpy1(prepare_frame->uart_buffer+3, payload, pyaload_length);
    crc16 = util_CRC16(prepare_frame->uart_buffer+1, pyaload_length+2);
    prepare_frame->uart_buffer[pyaload_length + 3] = (uint8_t)(crc16 & 0x00FF);
    prepare_frame->uart_buffer[pyaload_length + 4] = (uint8_t)((crc16 >> 8) & 0x00FF);
    prepare_frame->uart_buffer[pyaload_length + 5] = 0xEF;
    prepare_frame->uart_buffer_pkt_len = pyaload_length + 6;
}

void respone_config(uint8_t msg_type)
{
    uint8_t buffer[CONFIG_PARAM_SIZE+1] = {0};

    buffer[0] = 0xAB;
    get_eeprom_data(buffer+1, CONFIG_PARAM_SIZE);

    build_serial_frame(msg_type, buffer, CONFIG_PARAM_SIZE+1);
    
    if(UartPutBuffer( &Uart1, prepare_frame->uart_buffer, prepare_frame->uart_buffer_pkt_len ))
    {
        DEBUG("UART1 send Error\r\n");
    }
    else
    {
        DEBUG("UART1 send OK\r\n");
    }
}

enum ParseErrorCode parse_serial_frame(uint8_t *frame, uint8_t frame_length)
{
    uint8_t command = 0;
 
    serial_frame_t message_frame = {0};
    
    message_frame.crc16 = ((uint16_t)(frame[frame_length -1] << 8)) | frame[frame_length - 2];

    if(util_CRC16(frame, frame_length - 2) == message_frame.crc16)
    {
        message_frame.type = frame[0];
        message_frame.payload_length = frame[1];
        message_frame.pdata = frame + 2;
        
        switch(message_frame.type)
        {
        case SET_M100A: 
        case GET_M100A: 
            command = message_frame.pdata[0];
            switch(command)
            {
            case 0xAB:
                if(message_frame.type == SET_M100A)
                {
                    DEBUG("Set All Param %dByte\r\n", message_frame.payload_length-1);
                    if( set_m100a_config(message_frame.pdata+1, CONFIG_PARAM_SIZE) == 0 )
                    {
                    }
                    respone_config(SET_M100A_ACK);
                }
                else
                {
                    DEBUG("Get All Param %dByte\r\n", CONFIG_PARAM_SIZE);
                    respone_config(GET_M100A_ACK);
                }
                break;
            default:
                DEBUG("No define Command:%d\r\n", command);
                printf("Set failed\r\n");
                break;
            }
            break;
        default: 
            DEBUG("Error Serial Type:%d\r\n", message_frame.type); break;
        }
    }
    else
    {
        DEBUG("ERROR CRC:0x%04x->0x%04x\r\n", util_CRC16(frame, frame_length - 2), message_frame.crc16);
        return CRC_ERROR;
    }
    
    return FINISH;
}

void on_radio_tx_done(void)
{
    DEBUG("On Tx Done\r\n");
    rf_send_ok_ack();
    device_state = DEVICE_STATE_RECV;
}

void on_radio_rx_done( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    TimerStop(&rx_window_timer1);
    TimerStop(&rx_window_timer2);
    
    Radio.Sleep( );
    
    DEBUG("On Rx Done [%dB, RSSI:%d, SNR:%d]\r\n", size, rssi, snr);

    print_message("Received Frame:", payload, size, 'X');

    send_rf_data(payload, size );
    
    DEBUG("Send To Usart Done.\r\n");

    device_state = DEVICE_STATE_RECV;
}

void on_radio_tx_timeout(void)
{	
    Radio.Sleep();
    
    DEBUG("On Tx Timeout\r\n");
    rf_send_fail_ack();
    device_state = DEVICE_STATE_RECV;
}

void on_radio_rx_error(void)
{
    Radio.Sleep();
    
    DEBUG("On Rx Error\r\n");

    device_state = DEVICE_STATE_RECV;
}

void on_radio_rx_timeout(void)
{
    Radio.Sleep();
    
    DEBUG("On Rx Timeout\r\n");

    device_state = DEVICE_STATE_RECV;
}

void on_tx_delay_timer_event(void)
{
    TimerStop(&tx_delay_timer);
    mac_state &= ~MAC_TX_DELAYED;

    schedule_tx();
}

void on_rx_window1_timer_event(void)
{
    TimerStop(&rx_window_timer1);
    //rx window setup and rx runing

    rx_window_setup(m100a_config->rf_frequency, Datarates[m100a_config->rf_datarate]);
}

void on_rx_window2_timer_event(void)
{
    TimerStop(&rx_window_timer2);
    //rx window setup and rx runing

    rx_window_setup(m100a_config->rf_frequency, Datarates[m100a_config->rf_datarate]);
}

void led_blink(void)
{
    GPIO_Init(GPIOB, GPIO_Pin_3|GPIO_Pin_2, GPIO_Mode_Out_PP_High_Fast);
    Delay(1);
    GPIO_Init(GPIOB, GPIO_Pin_3|GPIO_Pin_2, GPIO_Mode_Out_PP_Low_Fast);
}

void show_non_lorawan_config_param(void)
{
    DEBUG("======================================================================\r\n");					
    print_message("DevEui:", m100a_config->deveui, 8, 'X');
    DEBUG("Freq:%d\r\n", (uint16_t)(m100a_config->rf_frequency/100000));
    DEBUG("Channel Datarate Index:%d\r\n", m100a_config->rf_datarate);
    DEBUG("Channel TxPower Index:%d\r\n", m100a_config->rf_tx_power);
    DEBUG("Bandwidth:%d\r\n", m100a_config->rf_bandwidth);
    DEBUG("Syncword:%02X\r\n", m100a_config->rf_syncword);
    DEBUG("Preamble_Length:%d\r\n", m100a_config->rf_preamble_len);
    DEBUG("Header Mode:%d\r\n", m100a_config->rf_header_mode);
    DEBUG("Code Rate:4/%d\r\n", m100a_config->rf_coderate+4);
    DEBUG("CRC Endble:%d\r\n", m100a_config->rf_crc_enable);
    DEBUG("RX IQ Signal:%d\r\n", m100a_config->rf_rx_iq_signal);
    DEBUG("TX IQ Signal:%d\r\n", m100a_config->rf_tx_iq_signal);
    DEBUG("Debug Switch:%d\r\n", m100a_config->EnableDebugPrintf);
    DEBUG("======================================================================\r\n");
}

void m100a_mac_initialization(void)
{
    m100a_config->EnableDebugPrintf = 1;
    
    BoardInitMcu( );
    
    debug_print_com_init();
    
    DEBUG("\r\nFirmware version: v1.0.0\r\n");
#ifdef CLEAR_CONFIG
    earse_eeprom(CONFIG_PARAM_SIZE);
#endif
    get_config_param();
    
    m100a_init( );
    
    led_blink();
    
    GPIO_Init(GPIOA, GPIO_Pin_6, GPIO_Mode_In_FL_No_IT);

    if((GPIOA->IDR & GPIO_Pin_6) >> 6)
    {
        DEBUG("In Config Mode\r\n");
        m100a_config->work_mode = WORK_CONFIG_MODE;
        while(1);
    }
    else
    {
        DEBUG("In Standard Mode\r\n");
        show_non_lorawan_config_param();
        m100a_config->work_mode = WORK_STANDARD_MODE;
    }

    TimerInit(&tx_delay_timer, on_tx_delay_timer_event);
    TimerInit(&rx_window_timer1, on_rx_window1_timer_event);
    TimerInit(&rx_window_timer2, on_rx_window2_timer_event);

    radio_events.TxDone = on_radio_tx_done;
    radio_events.RxDone = on_radio_rx_done;
    radio_events.RxError = on_radio_rx_error;
    radio_events.TxTimeout = on_radio_tx_timeout;
    radio_events.RxTimeout = on_radio_rx_timeout;
    Radio.Init(&radio_events);
    
    set_lora_header_mode(m100a_config->rf_header_mode);
    set_lora_syncword(m100a_config->rf_syncword);
    
    Radio.Sleep();

    mac_state = MAC_IDLE;
    
    DEBUG("MCU Init OK\r\n");
}

void node_device_process(void)
{
    device_state = DEVICE_STATE_INIT;
    
    while(1)
    {
        switch(device_state)
        {
        case DEVICE_STATE_INIT:
            m100a_mac_initialization();
            device_state = DEVICE_STATE_RECV;
            break;
        case DEVICE_STATE_SEND:
            schedule_tx();
            clear_usart_received_data_flag();
            device_state = DEVICE_STATE_SLEEP;
            break;
        case DEVICE_STATE_RECV:
            enable_usart_state(TRUE);
            rx_window_setup(m100a_config->rf_frequency, Datarates[m100a_config->rf_datarate]);
            device_state = DEVICE_STATE_SLEEP;
            break;
        case DEVICE_STATE_SLEEP:
            TimerLowPowerHandler( );
            break;
        default:
            device_state = DEVICE_STATE_INIT;
            break;
        }
        if( check_usart_received_data_flag( ) == TRUE )
        {
            enable_usart_state(FALSE);
            device_state = DEVICE_STATE_SEND;
        }
    }
}