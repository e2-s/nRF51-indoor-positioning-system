#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_trace.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_trace.h"
#include "app_util.h"
#include "app_error.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "nrf_delay.h"

#define CENTRAL_LINK_COUNT      1                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   0                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                             /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */

#define APPL_LOG                app_trace_log                   /**< Debug logger macro that will be used in this file to do logging of debug information over UART. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(75, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */


/* Added by Kumar for Indoor positioning */
#define NO_OF_REF_NODES				4
//#define ENVI_CONST				1.8
//#define AVG_1M				(-61) /* Absolute value of engergy */
#define AVG_SAMPLE_COUNT			20
#define CONST_SAMPLE_COUNT			100

/* Below variables starts with B denotes the X and Y coordinates of Beacons 1 to 4 */
#define B1X											1.5 
#define B1Y											0
#define B2X											0
#define B2Y											3.35 
#define B3X											3.65
#define B3Y											3 
#define B4X											3
#define B4Y											3
/* Added by Kumar for Indoor positioning */

static ble_nus_c_t              m_ble_nus_c;                    /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery;             /**< Instance of database discovery module. Must be passed to all db_discovert API calls */

/* Added by Kumar for using RSSI  */
int8_t rssi_inst[NO_OF_REF_NODES];
float dist_rssi[NO_OF_REF_NODES]; //Distance from each reference nodes
float alpha = 0.75; //only for RSSI smoothing
float dev_x,dev_y;
ble_gap_addr_t ref_node_addr[NO_OF_REF_NODES];
uint8_t ref_node_entry[NO_OF_REF_NODES][BLE_GAP_ADDR_LEN] = {	{0xC7,0x54,0x63,0x63,0xA1,0xD9},
																															
/* Added by Kumar for for taking avg of RSSI */

//int8_t no_avg = 100;
uint8_t count=0;
//uint8_t calc_dist=0; /* Flag to enable distance calculation - in case of moving avg or specific interval etc., */
int16_t rssi_sum[NO_OF_REF_NODES] = {0, 0, 0, 0};
int8_t rssi_samples[NO_OF_REF_NODES][AVG_SAMPLE_COUNT+1];																																
		int8_t rssi_avg[NO_OF_REF_NODES],rssi_avg_prev[NO_OF_REF_NODES];																											
		//int8_t 	rssi_samples[100];
	
		/* for sliding window */
		int8_t rssi_slide[NO_OF_REF_NODES];
																								
/* Added by Kumar for envinronmental factors */
	int8_t RSSI_AVG_1M[NO_OF_REF_NODES] = {-61, -56, -61, 0};
	uint8_t sample_count = 0;
	float ENVI_CONST[NO_OF_REF_NODES] = {1.6, 2.0, 1.8, 0};
	float estimated_envi_const[NO_OF_REF_NODES];
	float calibration_distance = 2;
	float const_sum[NO_OF_REF_NODES] = {0, 0, 0, 0};
	float const_avg[NO_OF_REF_NODES];
	float const_samples[CONST_SAMPLE_COUNT][NO_OF_REF_NODES];
	
/* Addded by Kumar for Kalman Filtering	*/
	
 /* Initial values for the Kalman filter  */
    float RSSI_est_prev[NO_OF_REF_NODES] = {0, 0, 0, 0};
    float P_prev[NO_OF_REF_NODES] = {0, 0, 0, 0}; 
    /* The noise in the system  (to be updated by repetitive testing/ trail and error method) */
    float const_Q = 0.022; //0.065
    float const_R = 0.617; //1.4
     
    float K[NO_OF_REF_NODES]; //Kalman_Gain
    float P[NO_OF_REF_NODES];  //Estimate_error
    float P_temp[NO_OF_REF_NODES]; 
    float RSSI_temp_est[NO_OF_REF_NODES]; 
    float RSSI_est[NO_OF_REF_NODES]; 
    float z_measured[NO_OF_REF_NODES]; //the RSSI value measured
	
	
/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
  };

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params = 
  {
    .active      = SCAN_ACTIVE,
    .selective   = SCAN_SELECTIVE,
    .p_whitelist = NULL,
    .interval    = SCAN_INTERVAL,
    .window      = SCAN_WINDOW,
    .timeout     = SCAN_TIMEOUT
  };

/**
 * @brief NUS uuid
 */
static const ble_uuid_t m_nus_uuid = 
  {
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
  };

/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    uint32_t err_code;
    
    err_code = sd_ble_gap_scan_start(&m_scan_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */ 
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                while (ble_nus_c_string_send(&m_ble_nus_c, data_array, index) != NRF_SUCCESS)
                {
                    // repeat until sent.
                }
                index = 0;
            }
            break;
        /**@snippet [Handling data from UART] */ 
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}


/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */ 
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            APPL_LOG("The device has the Nordic UART Service\r\n");
            break;
        
        case BLE_NUS_C_EVT_NUS_RX_EVT:
            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
            {
                while(app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
            }
            break;
        
        case BLE_NUS_C_EVT_DISCONNECTED:
            APPL_LOG("Disconnected\r\n");
            scan_start();
            break;
    }
}
/**@snippet [Handling events from the ble_nus_c module] */ 

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Reads an advertising report and checks if a uuid is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service uuids. 
 *          To see the format of a advertisement packet, see 
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The uuid to search fir
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false  
 */
static bool is_uuid_present(const ble_uuid_t *p_target_uuid, 
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;
    
    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index+1];
                
        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
            for (uint32_t u_index = 0; u_index < (field_length/UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE, 
                                                &p_data[u_index * UUID16_SIZE + index + 2], 
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
            for (uint32_t u_index = 0; u_index < (field_length/UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE, 
                &p_data[u_index * UUID32_SIZE + index + 2], 
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }
        
        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE, 
                                          &p_data[index + 2], 
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
		uint8_t		index; //Added by Kumar
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
		
		switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
/* Added by Kumar - for printing RSSI Value */

		/*
rssi_inst = p_adv_report->rssi;
printf("\nRSSI : %d",p_adv_report->rssi);
printf("Ref Node MAC %02x%02x%02x%02x%02x%02x\r\n",
p_adv_report->peer_addr.addr[0],
p_adv_report->peer_addr.addr[1],
p_adv_report->peer_addr.addr[2],
p_adv_report->peer_addr.addr[3],
p_adv_report->peer_addr.addr[4],
p_adv_report->peer_addr.addr[5]
                             );
		*/
						for (uint8_t cntn = 0; cntn < NO_OF_REF_NODES; cntn++)
						{
								if ( (ref_node_entry[cntn][0] == p_adv_report->peer_addr.addr[0]) && 
											(ref_node_entry[cntn][1] == p_adv_report->peer_addr.addr[1]) && 
											(ref_node_entry[cntn][2] == p_adv_report->peer_addr.addr[2]) && 
											(ref_node_entry[cntn][3] == p_adv_report->peer_addr.addr[3]) && 
											(ref_node_entry[cntn][4] == p_adv_report->peer_addr.addr[4]) && 
											(ref_node_entry[cntn][5] == p_adv_report->peer_addr.addr[5]) )
								{
										//printf("Device %d",(index+1));
										rssi_inst[cntn] = p_adv_report->rssi;
								}
							}
/* Added by Kumar - for printing RSSI Value */
		
            if (is_uuid_present(&m_nus_uuid, p_adv_report))
            {

                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param);

                if (err_code == NRF_SUCCESS)
                {
                    // scan is automatically stopped by the connect
                    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                    APP_ERROR_CHECK(err_code);
                    APPL_LOG("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
                }
            }
            break;
        }

        case BLE_GAP_EVT_CONNECTED:
            APPL_LOG("Connected to target\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            // start discovery of services. The NUS Client waits for a discovery result
            err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                APPL_LOG("[APPL]: Scan timed out.\r\n");
                scan_start();
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                APPL_LOG("[APPL]: Connection Request timed out.\r\n");
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break;
    
        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    ble_db_discovery_on_ble_evt(&m_ble_db_discovery, p_ble_evt);
    ble_nus_c_on_ble_evt(&m_ble_nus_c,p_ble_evt);
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}

/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOW,
                        err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client.
 */
static void nus_c_init(void)
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;
    
    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;
    
    err_code = ble_nus_c_init(&m_ble_nus_c, &nus_c_init_t);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(void)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

/* 
* Author : Kumar, e2-s
* Purpose: Kalman filtering for measuring RSSI value
*/
static void Kalman_RSSI(void)
{
	for(uint8_t cnt_node = 0; cnt_node < NO_OF_REF_NODES; cnt_node++)
	{
				/* Prediction  phase */
        RSSI_temp_est[cnt_node] = RSSI_est_prev[cnt_node]; 
        P_temp[cnt_node] = P_prev[cnt_node] + const_Q; 
        /* calculate the Kalman gain */
        K[cnt_node] = P_temp[cnt_node] * (1.0/(P_temp[cnt_node] + const_R));
        /* measure RSSI */
        z_measured[cnt_node] = rssi_inst[cnt_node];
        /* correct phase */
        RSSI_est[cnt_node] = RSSI_temp_est[cnt_node] + K[cnt_node] * (z_measured[cnt_node] - RSSI_temp_est[cnt_node]);  
        P[cnt_node] = (1 - K[cnt_node]) * P_temp[cnt_node]; 

         
        printf("\r\n Mesaured RSSI of Node %d : %6.3f",(cnt_node+1),z_measured[cnt_node]); 
        printf("\r\n Kalman   RSSI of Node %d : %6.3f",(cnt_node+1),RSSI_est[cnt_node]); 
         
                
        /* update previous values */
        P_prev[cnt_node] = P[cnt_node]; 
        RSSI_est_prev[cnt_node] = RSSI_est[cnt_node]; 
	}
}

/* 
* Author : Kumar, e2-s
* Purpose: Function to calculate environmental constant 'n'
*/
static void calc_envi_const()
{
	
		float num,den;
		
		if ( sample_count == CONST_SAMPLE_COUNT)
		{
				for(uint8_t index = 0;index < CONST_SAMPLE_COUNT;index++)
						for(uint8_t cnt=0 ; cnt < NO_OF_REF_NODES ; cnt++)
								const_sum[cnt] += const_samples[index][cnt];
			
				for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
				{
						//printf("\r\n Envi Constant Sum of node %d : %f",(cnt_node+1),const_sum[cnt_node]);
						const_avg[cnt_node] = const_sum[cnt_node] / CONST_SAMPLE_COUNT;
						printf("\r\n ***Constant Average of node %d : %f",(cnt_node+1),const_avg[cnt_node]);	
						const_sum[cnt_node] = 0;
				}
				sample_count = 0;
			
		}
		else
		{
			for (uint8_t index1=0 ; index1 < NO_OF_REF_NODES ; index1++)
			{
				num = abs(rssi_inst[index1]) - abs(RSSI_AVG_1M[index1]);
				den = 10 * log10(calibration_distance);	
				estimated_envi_const[index1] = num / den;
				const_samples[sample_count][index1]= estimated_envi_const[index1];
				//printf("\r\n Environmental constant of Node %d: %f",(index+1),estimated_envi_const[index]);
																		
			}
			sample_count++;
			
		}
			
}

/* 
* Author : Kumar, e2-s
* Purpose: To calculate average of RSSI
*/
static void avg_rssi(uint8_t no_avg)
{
	
	int16_t rssi_sum[NO_OF_REF_NODES] = {0, 0, 0, 0};
	int16_t rssi_samples[no_avg][NO_OF_REF_NODES];
	
		if(count == no_avg)
		{
				
				for(uint8_t index = 0;index < no_avg;index++)
						for(uint8_t cnt=0 ; cnt < NO_OF_REF_NODES ; cnt++)
								rssi_sum[cnt] += rssi_samples[index][cnt];
				for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
				{
						printf("\r\n RSSI Sum of node %d : %d",(cnt_node+1),rssi_sum[cnt_node]);
						rssi_avg_prev[cnt_node] = rssi_avg[cnt_node];
						rssi_avg[cnt_node] = rssi_sum[cnt_node] / no_avg;
						printf("\r\n ***RSSI Average of node %d : %d",(cnt_node+1),rssi_avg[cnt_node]);	
				}
				count = 0;
				//calc_dist = 1;
//				calc_envi_const(rssi_inst);
		}
		else
		{
				
				for(uint8_t cnt=0 ; cnt < NO_OF_REF_NODES ; cnt++)
					rssi_samples[count][cnt]= rssi_inst[cnt];
					
				count++;
		}
		
}						

/* 
* Author : Kumar, e2-s
* Purpose: To calculate moving average
*/
static void mov_avg_rssi()
{
	
	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
		for(uint8_t cnt_sample=0 ; cnt_sample < AVG_SAMPLE_COUNT ; cnt_sample++)
			rssi_samples[cnt_node][cnt_sample] = rssi_samples[cnt_node][cnt_sample+1];
	
	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
			rssi_samples[cnt_node][AVG_SAMPLE_COUNT] = rssi_inst[cnt_node];

	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
		for(uint8_t cnt_sample=0 ; cnt_sample < AVG_SAMPLE_COUNT ; cnt_sample++)
			rssi_sum[cnt_node] += rssi_samples[cnt_node][cnt_sample];

	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
	{
			//printf("\r\n RSSI Sum of node %d : %d",(cnt_node+1),rssi_sum[cnt_node]);
			rssi_avg[cnt_node] = rssi_sum[cnt_node] / AVG_SAMPLE_COUNT;
			//printf("\r\n ***RSSI Average of node %d : %d",(cnt_node+1),rssi_avg[cnt_node]);	
	}
	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
		rssi_sum[cnt_node] = 0;

				
		
		
}						

/* 
* Author : Kumar, e2-s
* Purpose: To calculate average - sliding window
*/
static void avg_sliding_window()
{
	
	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
		for(uint8_t cnt_sample=0 ; cnt_sample < AVG_SAMPLE_COUNT ; cnt_sample++)
			rssi_samples[cnt_node][cnt_sample] = rssi_samples[cnt_node][cnt_sample+1];
	
	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
			rssi_samples[cnt_node][AVG_SAMPLE_COUNT] = rssi_inst[cnt_node];

	for(uint8_t cnt_node=0 ; cnt_node < NO_OF_REF_NODES ; cnt_node++)
	{
			printf("\r\n RSSI present of node %d : %d",(cnt_node+1),rssi_samples[cnt_node][AVG_SAMPLE_COUNT]);
			rssi_slide[cnt_node] = ( 0.5 * rssi_samples[cnt_node][AVG_SAMPLE_COUNT] ) + 
															( 0.2 * rssi_samples[cnt_node][AVG_SAMPLE_COUNT - 1] )	+
															( 0.2 * rssi_samples[cnt_node][AVG_SAMPLE_COUNT - 2] ) + 
															( 0.1 * rssi_samples[cnt_node][AVG_SAMPLE_COUNT - 3] );
			printf("\r\n ***RSSI slide of node %d : %d",(cnt_node+1),rssi_slide[cnt_node]);	
	}

}						

/* 
* Author : Kumar, e2-s
* Purpose: To calculate distance from RSSI
*/
static void distance_estimate()
{
		float num[NO_OF_REF_NODES],dem[NO_OF_REF_NODES];
		int8_t rssi_smooth[NO_OF_REF_NODES];
	
		//if(calc_dist)
		{
				for(uint8_t cnt=0;cnt < NO_OF_REF_NODES; cnt++)
				{
					//num[cnt] = RSSI_AVG_1M[cnt] - rssi_slide[cnt]; //using sliding window estimate
					num[cnt] = RSSI_AVG_1M[cnt] - RSSI_est[cnt]; //using Kalman Filtered RSSI
					//rssi_smooth[cnt] = ((alpha * rssi_inst[cnt]) + ((1 - alpha) * rssi_avg[cnt])); //before Kalman
					//num[cnt] = RSSI_AVG_1M[cnt] - rssi[cnt]; //before smoothing
					//num[cnt] = RSSI_AVG_1M[nt] - rssi_smooth[cnt]; //RSSI smoothing 
					dem[cnt] = 10*ENVI_CONST[cnt];
				
		//		printf("\nNumerator : %f",num[cnt]);
		//		printf("\nDenominator : %f",dem[cnt]);
					dist_rssi[cnt] = pow(10,(num[cnt]/dem[cnt]));
//					printf("\r\n RSSI Instantaneous of reference node %d : %d",(cnt+1),rssi_inst[cnt]);
//					printf("\r\n RSSI AVG of reference node %d : %d",(cnt+1),rssi_avg[cnt]);
//					printf("\r\n RSSI Smooth of reference node %d : %d",(cnt+1),rssi_smooth[cnt]);
					printf("\r\n Distance from reference node %d : %f",(cnt+1),dist_rssi[cnt]);
				}
				//calc_dist = 0;
		}
}

/* 
* Author : Kumar, e2-s
* Purpose: Function uses Trilateration algorithm to estimate the distance of the device
*/
static void locate_device(void)
{
		float VarA,VarB,VarC;
		float VarX32, VarX13, VarX21, VarY32, VarY13, VarY21;
		float xden,xnum,yden,ynum;
		
		VarA = ((B1X * B1X) + (B1Y * B1Y) - (dist_rssi[0] * dist_rssi[0]));
		VarB = ((B2X * B2X) + (B2Y * B2Y) - (dist_rssi[1] * dist_rssi[1]));
		VarC = ((B3X * B3X) + (B3Y * B3Y) - (dist_rssi[2] * dist_rssi[2]));
	
		VarX32 = B3X - B2X;
		VarX13 = B1X - B3X;
		VarX21 = B2X - B1X;
		
		VarY32 = B3Y - B2Y;
		VarY13 = B1Y - B3Y;
		VarY21 = B2Y - B1Y;
		
		xnum = ((VarA * VarY32) + (VarB * VarY13) + (VarC * VarY21));
		xden = 2 * ((B1X * VarY32) + (B2X * VarY13) + (B3X * VarY21));
	
		ynum = ((VarA * VarX32) + (VarB * VarX13) + (VarC * VarX21));
		yden = 2 * ((B1Y * VarX32) + (B2Y * VarX13) + (B3Y * VarX21));
	
		dev_x = xnum / xden;
		dev_y = ynum / yden;
		printf("\n\r Location of device :(%f,%f)",dev_x,dev_y); 
}

int main(void)
{

		/* Added by Kumar for calculating Averages */
		/*
		int8_t no_avg = 100;
		uint8_t count=0,index;
		int16_t rssi_avg,rssi_sum = 0;
		int8_t 	rssi_samples[100];
		int ref_qty;
		*/
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    uart_init();
    buttons_leds_init();
    db_discovery_init();
    ble_stack_init();
    nus_c_init();

    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    scan_start();
    //printf("Scan started\r\n");
		
	/* Added By Kumar for Indoor Positioning */
	/*
	printf("Enter the number of Reference nodes:\n");
	scanf("%d",&ref_qty);
	config_ref_node();
	*/
    for (;;)
    {
        //power_manage();
	//avg_rssi(100);
	//mov_avg_rssi();
	//avg_sliding_window();
	Kalman_RSSI();
	//calc_envi_const();
	distance_estimate();
	locate_device();
	nrf_delay_ms(3000);
    }
}
