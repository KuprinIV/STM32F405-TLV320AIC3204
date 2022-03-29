#ifndef dumo_bglib_h
#define dumo_bglib_h
#define VERSION -e

/*****************************************************************************
 *
 *
 *  Header only interface for BGLIB
 *
 *
 ****************************************************************************/

/*****************************************************************************
 *
 *  BGLIB usage:
 *      Define library, it should be defined globally to allow usage 
 *      from every function:
 *          BGLIB_DEFINE();
 *
 *      Declare and define output function, prototype is:
 *          void my_output(uint8 len1,uint8* data1,uint16 len2,uint8* data2);
 *
 *          function sends len1 amount of data from pointer data1 to device, and then sends
 *          len2 amount of data from pointer data2 to device
 *
 *      Initialize library,and provide output function:
 *          BGLIB_INITIALIZE(my_output);
 *
 *      Receiving messages:
 *          Allocate buffer of size BGLIB_MSG_MAXLEN for message, can be less but reader shall then drop too long packets from module
 *          Read BGLIB_MSG_HEADER_LEN amount of bytes from device to buffer
 *          Read BGLIB_MSG_LEN(buffer) amount of bytes from device to buffer after the header
 *          Get message ID by BGLIB_MSG_ID(buffer), use switch or if to compare to message_ids:
 *                  if(BGLIB_MSG_ID(buffer)==wifi_rsp_sme_wifi_on_id)
 *              Cast buffer to msg_packet structure by BGLIB_MSG(buffer) and use it to access message structure:
 *                  if(BGLIB_MSG(buffer)->rsp_sme_wifi_on.result != wifi_err_success)
 *
 *      Sending messages:
 *          Call macros which build message to temporary buffer and calls my_output to send message.
 *              wifi_cmd_sme_connect_bssid(&selected_bssid[0]);
 *          
 *
 ****************************************************************************/

/*****************************************************************************
 *
 *      Notes about SPI interface:
 *          Byte zero is sent by module when there is no valid data to send.
 *          Host shall drop bytes from module until first nonzero byte is received.
 *          Host shall send zero bytes to module when it is reading data from module
 *          and there is no valid data to send.
 *
 ****************************************************************************/

/*****************************************************************************
 *
 *      Support for compilers other than GCC and MSVC:
 *      
 *      To support specific compiler add compiler's structure packing directives to following macro:
 *          PACKSTRUCT( decl )
 *          
 *      BGLIB uses PACKSTRUCT macro to add packing information for structures:
 *      PACKSTRUCT(struct wifi_msg_dfu_reset_cmd_t
 *      {
 *          uint8	dfu;
 *      });
 *
 *
 ****************************************************************************/

/*lint -save --e{528,572,778,845,835,849}*/
#include "apitypes.h"
#ifdef __cplusplus
extern "C" {
#endif

/* Compability */
#ifndef PACKSTRUCT
/*Default packed configuration*/
#ifdef __GNUC__
#ifdef _WIN32
#define PACKSTRUCT( decl ) decl __attribute__((__packed__,gcc_struct))
#else
#define PACKSTRUCT( decl ) decl __attribute__((__packed__))
#endif
#define ALIGNED __attribute__((aligned(0x4)))
#elif __IAR_SYSTEMS_ICC__

#define PACKSTRUCT( decl ) __packed decl

#define ALIGNED
#elif _MSC_VER  //msvc

#define PACKSTRUCT( decl ) __pragma( pack(push, 1) ) decl __pragma( pack(pop) )
#define ALIGNED
#else 
#define PACKSTRUCT(a) a PACKED 
#endif
#endif


struct dumo_header
{
    uint8  type_hilen;
    uint8  lolen;
    uint8  cls;
    uint8  command;
};

#define BGLIB_DEFINE() struct dumo_cmd_packet bglib_temp_msg;\
void (*bglib_output)(uint8 len1,uint8* data1,uint16 len2,uint8* data2);
#define BGLIB_INITIALIZE(X) bglib_output=X;
#define BGLIB_MSG_ID(BUF) ((*((uint32*)BUF))&0xffff00f8)
#define BGLIB_MSG(X) ((struct dumo_cmd_packet*)X)
#define BGLIB_MSG_HEADER_LEN (4)
#define BGLIB_MSG_LEN(BUF) ((((*((uint32*)BUF))&0x7)<<8)|(((*((uint32*)BUF))&0xff00)>>8))
#define BGLIB_MSG_MAXLEN 2052
extern void (*bglib_output)(uint8 len1,uint8* data1,uint16 len2,uint8* data2);

enum bt_gap_discover_mode
{
	bt_gap_discover_generic = 0,
	bt_gap_discover_limited = 1,
	bt_gap_discover_mode_max= 2
};

enum le_gap_address_type
{
	le_gap_address_type_public          = 0,
	le_gap_address_type_random          = 1,
	le_gap_address_type_public_identity = 2,
	le_gap_address_type_random_identity = 3,
	le_gap_address_type_bredr           = 16,
	le_gap_address_type_max             = 17
};

enum le_gap_connectable_mode
{
	le_gap_non_connectable           = 0,
	le_gap_directed_connectable      = 1,
	le_gap_undirected_connectable    = 2,
	le_gap_scannable_non_connectable = 3,
	le_gap_connectable_mode_max      = 4
};

enum le_gap_discoverable_mode
{
	le_gap_non_discoverable     = 0,
	le_gap_limited_discoverable = 1,
	le_gap_general_discoverable = 2,
	le_gap_broadcast            = 3,
	le_gap_user_data            = 4,
	le_gap_discoverable_mode_max= 5
};

enum le_gap_discover_mode
{
	le_gap_discover_limited     = 0,
	le_gap_discover_generic     = 1,
	le_gap_discover_observation = 2,
	le_gap_discover_mode_max    = 3
};

enum bt_connection_direction
{
	bt_connection_outgoing = 0,
	bt_connection_incoming = 1,
	bt_connection_direction_max= 2
};

enum bt_connection_powermode
{
	bt_connection_active = 0,
	bt_connection_hold   = 1,
	bt_connection_sniff  = 2,
	bt_connection_park   = 3,
	bt_connection_powermode_max= 4
};

enum bt_connection_role
{
	bt_connection_central    = 0,
	bt_connection_peripheral = 1,
	bt_connection_role_max   = 2
};

enum bt_connection_encryption
{
	bt_connection_plain     = 0,
	bt_connection_encrypted = 1,
	bt_connection_encryption_max= 2
};

enum le_connection_security
{
	le_connection_mode1_level1 = 0,
	le_connection_mode1_level2 = 1,
	le_connection_mode1_level3 = 2,
	le_connection_security_max = 3
};

enum gatt_att_opcode
{
	gatt_read_by_type_request      = 0x08,
	gatt_read_by_type_response     = 0x09,
	gatt_read_request              = 0x0a,
	gatt_read_response             = 0x0b,
	gatt_read_blob_request         = 0x0c,
	gatt_read_blob_response        = 0x0d,
	gatt_read_multiple_request     = 0x0e,
	gatt_read_multiple_response    = 0x0f,
	gatt_write_request             = 0x12,
	gatt_write_response            = 0x13,
	gatt_write_command             = 0x52,
	gatt_prepare_write_request     = 0x16,
	gatt_prepare_write_response    = 0x17,
	gatt_execute_write_request     = 0x18,
	gatt_execute_write_response    = 0x19,
	gatt_handle_value_notification = 0x1b,
	gatt_handle_value_indication   = 0x1d,
	gatt_att_opcode_max            = 30
};

enum gatt_client_config_flag
{
	gatt_disable      = 0x0,
	gatt_notification = 0x1,
	gatt_indication   = 0x2,
	gatt_client_config_flag_max= 3
};

enum gatt_execute_write_flag
{
	gatt_cancel = 0x0,
	gatt_commit = 0x1,
	gatt_execute_write_flag_max= 2
};

enum gatt_server_characteristic_status_flag
{
	gatt_server_client_config = 0x1,
	gatt_server_confirmation  = 0x2,
	gatt_server_characteristic_status_flag_max= 3
};

#define ENDPOINT_FLAG_UPDATED    0x01
#define ENDPOINT_FLAG_ACTIVE     0x02
#define ENDPOINT_FLAG_STREAMING  0x04
#define ENDPOINT_FLAG_BGAPI      0x08
#define ENDPOINT_FLAG_WAIT_CLOSE 0x10
#define ENDPOINT_FLAG_CLOSING    0x20
#define ENDPOINT_FLAG_FULL       0x40

enum endpoint_types
{
	endpoint_free       = 0x0,
	endpoint_uart       = 0x1,
	endpoint_script     = 0x2,
	endpoint_reserved   = 0x4,
	endpoint_drop       = 0x10,
	endpoint_rfcomm     = 0x20,
	endpoint_spi        = 0x40,
	endpoint_connection = 0x80,
	endpoint_iap        = 0x200,
	endpoint_l2cap      = 0x400,
	endpoint_hid        = 0x800,
	endpoint_leserial   = 0x1000,
	endpoint_types_max  = 4097
};

enum hardware_gpio_configuration
{
	hardware_gpio_float    = 0x0,
	hardware_gpio_pullup   = 0x1,
	hardware_gpio_pulldown = 0x2,
	hardware_gpio_configuration_max= 3
};

enum hardware_gpio_mode
{
	hardware_gpio_input    = 0x0,
	hardware_gpio_output   = 0x1,
	hardware_gpio_function = 0x2,
	hardware_gpio_analog   = 0x3,
	hardware_gpio_mode_max = 4
};

enum hardware_uartparity
{
	hardware_none = 0,
	hardware_odd  = 1,
	hardware_even = 2,
	hardware_uartparity_max= 3
};

enum hardware_cspolarity
{
	hardware_low       = 0,
	hardware_high      = 1,
	hardware_opendrain = 2,
	hardware_cspolarity_max= 3
};

#define FLASH_PS_KEY_BD_ADDR                     1
#define FLASH_PS_KEY_LOCAL_NAME                  4
#define FLASH_PS_KEY_BT_CLASS                    5
#define FLASH_PS_KEY_IWRAP_PAGE_MODE             6
#define FLASH_PS_KEY_IWRAP_PAGE_TIMEOUT          7
#define FLASH_PS_KEY_IWRAP_PAGE_SCAN_MODE        8
#define FLASH_PS_KEY_IWRAP_ALT_PAGE_MODE         9
#define FLASH_PS_KEY_IWRAP_PAGE_CONN_COUNT       10
#define FLASH_PS_KEY_IWRAP_MS_POLICY             11
#define FLASH_PS_KEY_IWRAP_LINK_POLICY           12
#define FLASH_PS_KEY_IWRAP_SUPERVISION_TIMEOUT   13
#define FLASH_PS_KEY_SNIFF_MAX_INTERVAL          14
#define FLASH_PS_KEY_SNIFF_MIN_INTERVAL          15
#define FLASH_PS_KEY_SNIFF_ATTEMPTS              16
#define FLASH_PS_KEY_SNIFF_TIMEOUT               17
#define FLASH_PS_KEY_IO_CAPABILITIES             18
#define FLASH_PS_KEY_MITM                        19
#define FLASH_PS_KEY_IWRAP_CONTROL_ECHO          20
#define FLASH_PS_KEY_UART_SPEED                  21
#define FLASH_PS_KEY_PROFILE_SPP                 22
#define FLASH_PS_KEY_PORTA_PIO_DIRECTION         23
#define FLASH_PS_KEY_PORTB_PIO_DIRECTION         24
#define FLASH_PS_KEY_PORTA_PIO_BIAS              25
#define FLASH_PS_KEY_PORTB_PIO_BIAS              26
#define FLASH_PS_KEY_PORTA_PIO_PULL_UP           27
#define FLASH_PS_KEY_PORTB_PIO_PULL_UP           28
#define FLASH_PS_KEY_IWRAP_I2C_STATUS            29
#define FLASH_PS_KEY_IWRAP_SPI_STATUS            30
#define FLASH_PS_KEY_MAXPAIRCOUNT                31
#define FLASH_PS_KEY_LAST_DELETED_PAIR           32
#define FLASH_PS_KEY_IWRAP_CD_MASK               33
#define FLASH_PS_KEY_IWRAP_CD_DATA_MASK          34
#define FLASH_PS_KEY_IWRAP_CD_MODE               35
#define FLASH_PS_KEY_IWRAP_AUTH_PIN_VISIBLE      36
#define FLASH_PS_KEY_AUTH_PIN                    37
#define FLASH_PS_KEY_IWRAP_READY_MASK            38
#define FLASH_PS_KEY_IWRAP_ESC_CHAR              39
#define FLASH_PS_KEY_IWRAP_DTR_PIO_MASK          40
#define FLASH_PS_KEY_IWRAP_DTR_MODE              41
#define FLASH_PS_KEY_MINIMUM_ENCRYPTION_KEY_SIZE 42
#define FLASH_PS_KEY_INQUIRY_SCAN_LAP            43
#define FLASH_PS_KEY_LOCAL_BD_ADDR               44
#define FLASH_PS_KEY_AUTOCALL                    45
#define FLASH_PS_KEY_AUTOCALL_DUMMY              46
#define FLASH_PS_KEY_LAP                         47
#define FLASH_PS_KEY_CONNAUTH                    48
#define FLASH_PS_KEY_TX_POWER                    49
#define FLASH_PS_KEY_BONDING_MODE                50
#define FLASH_PS_KEY_DID_RECORD                  51
#define FLASH_PS_KEY_MAX_INCOMING_ACL            52
#define FLASH_PS_KEY_RFCOMM_CREDITS_MAX          53
#define FLASH_PS_KEY_RFCOMM_MTU                  54

enum test_packet_type
{
	test_pkt_prbs9    = 0x0,
	test_pkt_11110000 = 0x1,
	test_pkt_10101010 = 0x2,
	test_pkt_carrier  = 0x3,
	test_packet_type_max= 4
};

enum sm_bonding_key
{
	sm_bonding_key_ltk         = 0x01,
	sm_bonding_key_addr_public = 0x02,
	sm_bonding_key_addr_static = 0x04,
	sm_bonding_key_irk         = 0x08,
	sm_bonding_key_edivrand    = 0x10,
	sm_bonding_key_csrk        = 0x20,
	sm_bonding_key_centralid   = 0x40,
	sm_bonding_key_max         = 65
};

enum sm_io_capability
{
	sm_io_capability_displayonly     = 0,
	sm_io_capability_displayyesno    = 1,
	sm_io_capability_keyboardonly    = 2,
	sm_io_capability_noinputnooutput = 3,
	sm_io_capability_keyboarddisplay = 4,
	sm_io_capability_max             = 5
};

enum bt_hid_state
{
	bt_hid_state_boot_mode            = 0,
	bt_hid_state_report_mode          = 1,
	bt_hid_state_suspend              = 3,
	bt_hid_state_exit_suspend         = 4,
	bt_hid_state_virtual_cable_unplug = 5,
	bt_hid_state_max                  = 6
};

enum bt_hid_get_report_params
{
	bt_hid_params_no_id_no_max_size     = 0,
	bt_hid_params_with_id_no_max_size   = 1,
	bt_hid_params_no_id_with_max_size   = 2,
	bt_hid_params_with_id_with_max_size = 3,
	bt_hid_get_report_params_max        = 4
};

enum bt_hid_report_type
{
	bt_hid_report_type_other   = 0,
	bt_hid_report_type_input   = 1,
	bt_hid_report_type_output  = 2,
	bt_hid_report_type_feature = 3,
	bt_hid_report_type_max     = 4
};


enum dumo_parameter_types
{
    dumo_msg_parameter_uint8=2,
    dumo_msg_parameter_int8=3,
    dumo_msg_parameter_uint16=4,
    dumo_msg_parameter_int16=5,
    dumo_msg_parameter_uint32=6,
    dumo_msg_parameter_int32=7,
    dumo_msg_parameter_uint8array=8,
    dumo_msg_parameter_string=9,
    dumo_msg_parameter_hwaddr=10,
    dumo_msg_parameter_uint16array=11
};

enum dumo_msg_types
{
    dumo_msg_type_cmd=0x00,
    dumo_msg_type_rsp=0x00,
    dumo_msg_type_evt=0x80
};
enum dumo_dev_types
{
    dumo_dev_type_ble    =0x00,
    dumo_dev_type_wifi   =0x08,
    dumo_dev_type_iwrap  =0x10,
    dumo_dev_type_dumo   =0x20,
    dumo_dev_type_test   =0x20,
    dumo_dev_type_gecko  =0x20
};

#define	dumo_cmd_dfu_reset_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00000000)
#define	dumo_cmd_dfu_flash_set_address_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01000000)
#define	dumo_cmd_dfu_flash_upload_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02000000)
#define	dumo_cmd_dfu_flash_upload_finish_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03000000)
#define	dumo_cmd_system_hello_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00010000)
#define	dumo_cmd_system_reset_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01010000)
#define	dumo_cmd_system_set_max_power_mode_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02010000)
#define	dumo_cmd_system_get_bt_address_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03010000)
#define	dumo_cmd_system_set_bt_address_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04010000)
#define	dumo_cmd_system_get_class_of_device_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05010000)
#define	dumo_cmd_system_set_class_of_device_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06010000)
#define	dumo_cmd_system_reset_factory_settings_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07010000)
#define	dumo_cmd_system_set_local_name_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x08010000)
#define	dumo_cmd_system_get_local_name_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x09010000)
#define	dumo_cmd_system_write_bt_config_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A010000)
#define	dumo_cmd_bt_gap_discover_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00020000)
#define	dumo_cmd_bt_gap_cancel_discovery_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01020000)
#define	dumo_cmd_bt_gap_get_remote_name_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02020000)
#define	dumo_cmd_bt_gap_set_mode_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03020000)
#define	dumo_cmd_bt_gap_get_mode_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04020000)
#define	dumo_cmd_bt_gap_set_policy_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05020000)
#define	dumo_cmd_bt_gap_set_parameters_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06020000)
#define	dumo_cmd_bt_gap_set_max_power_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07020000)
#define	dumo_cmd_bt_gap_open_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x08020000)
#define	dumo_cmd_bt_gap_set_auto_sniff_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x09020000)
#define	dumo_cmd_bt_gap_set_discovery_mode_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A020000)
#define	dumo_cmd_bt_gap_set_host_channel_classification_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0B020000)
#define	dumo_cmd_le_gap_open_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00030000)
#define	dumo_cmd_le_gap_set_mode_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01030000)
#define	dumo_cmd_le_gap_discover_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02030000)
#define	dumo_cmd_le_gap_end_procedure_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03030000)
#define	dumo_cmd_le_gap_set_adv_parameters_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04030000)
#define	dumo_cmd_le_gap_set_conn_parameters_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05030000)
#define	dumo_cmd_le_gap_set_scan_parameters_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06030000)
#define	dumo_cmd_le_gap_set_adv_data_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07030000)
#define	dumo_cmd_le_gap_set_max_power_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x08030000)
#define	dumo_cmd_le_gap_set_host_channel_classification_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x09030000)
#define	dumo_cmd_le_gap_set_scan_result_filter_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A030000)
#define	dumo_cmd_le_gap_scan_filter_clear_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0B030000)
#define	dumo_cmd_le_gap_set_random_static_address_id                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0C030000)
#define	dumo_cmd_bt_rfcomm_open_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00040000)
#define	dumo_cmd_bt_rfcomm_start_server_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01040000)
#define	dumo_cmd_bt_rfcomm_open_port_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02040000)
#define	dumo_cmd_bt_rfcomm_stop_server_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03040000)
#define	dumo_cmd_bt_rfcomm_modem_status_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04040000)
#define	dumo_cmd_bt_rfcomm_start_server_port_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05040000)
#define	dumo_cmd_bt_rfcomm_stop_server_port_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06040000)
#define	dumo_cmd_bt_rfcomm_set_modem_status_default_id              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07040000)
#define	dumo_cmd_identity_remote_identity_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00050000)
#define	dumo_cmd_identity_local_identity_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01050000)
#define	dumo_cmd_identity_modify_local_identity_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02050000)
#define	dumo_cmd_bt_sdp_search_rfcomm_port_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00060000)
#define	dumo_cmd_bt_sdp_load_entry_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01060000)
#define	dumo_cmd_bt_sdp_add_entry_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02060000)
#define	dumo_cmd_bt_sdp_delete_entry_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03060000)
#define	dumo_cmd_bt_connection_list_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00070000)
#define	dumo_cmd_bt_connection_set_active_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01070000)
#define	dumo_cmd_bt_connection_set_sniff_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02070000)
#define	dumo_cmd_bt_connection_set_role_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03070000)
#define	dumo_cmd_bt_connection_set_supervision_timeout_id           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04070000)
#define	dumo_cmd_bt_connection_get_rssi_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05070000)
#define	dumo_cmd_bt_connection_read_clock_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06070000)
#define	dumo_cmd_le_connection_set_parameters_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00080000)
#define	dumo_cmd_le_connection_list_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01080000)
#define	dumo_cmd_gatt_set_max_mtu_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00090000)
#define	dumo_cmd_gatt_discover_primary_services_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01090000)
#define	dumo_cmd_gatt_discover_primary_services_by_uuid_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02090000)
#define	dumo_cmd_gatt_discover_characteristics_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03090000)
#define	dumo_cmd_gatt_discover_characteristics_by_uuid_id           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04090000)
#define	dumo_cmd_gatt_set_characteristic_notification_id            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05090000)
#define	dumo_cmd_gatt_discover_descriptors_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06090000)
#define	dumo_cmd_gatt_read_characteristic_value_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07090000)
#define	dumo_cmd_gatt_read_characteristic_value_by_uuid_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x08090000)
#define	dumo_cmd_gatt_write_characteristic_value_id                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x09090000)
#define	dumo_cmd_gatt_write_characteristic_value_without_response_id	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A090000)
#define	dumo_cmd_gatt_prepare_characteristic_value_write_id         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0B090000)
#define	dumo_cmd_gatt_execute_characteristic_value_write_id         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0C090000)
#define	dumo_cmd_gatt_send_characteristic_confirmation_id           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0D090000)
#define	dumo_cmd_gatt_read_descriptor_value_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0E090000)
#define	dumo_cmd_gatt_write_descriptor_value_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0F090000)
#define	dumo_cmd_gatt_find_included_services_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x10090000)
#define	dumo_cmd_gatt_read_multiple_characteristic_values_id        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x11090000)
#define	dumo_cmd_gatt_find_information_request_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x12090000)
#define	dumo_cmd_gatt_server_read_attribute_value_id                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x000A0000)
#define	dumo_cmd_gatt_server_read_attribute_type_id                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x010A0000)
#define	dumo_cmd_gatt_server_write_attribute_value_id               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x020A0000)
#define	dumo_cmd_gatt_server_send_user_read_response_id             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x030A0000)
#define	dumo_cmd_gatt_server_send_user_write_response_id            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x040A0000)
#define	dumo_cmd_gatt_server_send_characteristic_notification_id    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x050A0000)
#define	dumo_cmd_gatt_server_set_service_status_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x060A0000)
#define	dumo_cmd_endpoint_send_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x000B0000)
#define	dumo_cmd_endpoint_set_streaming_destination_id              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x010B0000)
#define	dumo_cmd_endpoint_close_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x020B0000)
#define	dumo_cmd_endpoint_set_flags_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x030B0000)
#define	dumo_cmd_endpoint_clr_flags_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x040B0000)
#define	dumo_cmd_endpoint_read_counters_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x050B0000)
#define	dumo_cmd_endpoint_set_active_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x060B0000)
#define	dumo_cmd_hardware_set_soft_timer_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x000C0000)
#define	dumo_cmd_hardware_configure_gpio_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x010C0000)
#define	dumo_cmd_hardware_write_gpio_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x020C0000)
#define	dumo_cmd_hardware_read_gpio_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x030C0000)
#define	dumo_cmd_hardware_read_adc_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x040C0000)
#define	dumo_cmd_hardware_read_i2c_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x050C0000)
#define	dumo_cmd_hardware_write_i2c_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x060C0000)
#define	dumo_cmd_hardware_stop_i2c_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x070C0000)
#define	dumo_cmd_hardware_set_uart_configuration_id                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x080C0000)
#define	dumo_cmd_hardware_read_vdd_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x090C0000)
#define	dumo_cmd_hardware_read_junction_temperature_id              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A0C0000)
#define	dumo_cmd_hardware_read_write_spi_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0B0C0000)
#define	dumo_cmd_hardware_set_uart_bgapi_mode_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0C0C0000)
#define	dumo_cmd_flash_ps_dump_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x000D0000)
#define	dumo_cmd_flash_ps_erase_all_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x010D0000)
#define	dumo_cmd_flash_ps_save_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x020D0000)
#define	dumo_cmd_flash_ps_load_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x030D0000)
#define	dumo_cmd_flash_ps_erase_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x040D0000)
#define	dumo_cmd_test_dtm_tx_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x000E0000)
#define	dumo_cmd_test_dtm_rx_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x010E0000)
#define	dumo_cmd_test_dtm_end_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x020E0000)
#define	dumo_cmd_test_tx_test_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x030E0000)
#define	dumo_cmd_test_debug_enable_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x040E0000)
#define	dumo_cmd_test_enable_uart_loopback_mode_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x050E0000)
#define	dumo_cmd_test_ssp_debug_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x060E0000)
#define	dumo_cmd_test_debug_command_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x070E0000)
#define	dumo_cmd_test_device_under_test_mode_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x080E0000)
#define	dumo_cmd_test_rx_test_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x090E0000)
#define	dumo_cmd_test_packet_test_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A0E0000)
#define	dumo_cmd_test_debug_counter_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0B0E0000)
#define	dumo_cmd_sm_set_bondable_mode_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x000F0000)
#define	dumo_cmd_sm_configure_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x010F0000)
#define	dumo_cmd_sm_store_bonding_configuration_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x020F0000)
#define	dumo_cmd_sm_read_bonding_configuration_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x030F0000)
#define	dumo_cmd_sm_increase_security_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x040F0000)
#define	dumo_cmd_sm_read_bonding_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x050F0000)
#define	dumo_cmd_sm_delete_bonding_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x060F0000)
#define	dumo_cmd_sm_delete_bondings_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x070F0000)
#define	dumo_cmd_sm_enter_passkey_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x080F0000)
#define	dumo_cmd_sm_passkey_confirm_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x090F0000)
#define	dumo_cmd_sm_set_oob_data_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A0F0000)
#define	dumo_cmd_sm_list_all_bondings_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0B0F0000)
#define	dumo_cmd_sm_confirm_bonding_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0C0F0000)
#define	dumo_cmd_sm_enter_pin_code_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0D0F0000)
#define	dumo_cmd_sm_set_le_bondable_mode_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0E0F0000)
#define	dumo_cmd_sm_set_le_key_enc_only_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0F0F0000)
#define	dumo_cmd_sm_set_minimum_key_size_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x100F0000)
#define	dumo_cmd_iap_open_id                                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00100000)
#define	dumo_cmd_iap_start_server_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01100000)
#define	dumo_cmd_iap_set_param_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02100000)
#define	dumo_cmd_iap_set_protocol_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03100000)
#define	dumo_cmd_iap_get_param_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04100000)
#define	dumo_cmd_iap_get_protocol_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05100000)
#define	dumo_cmd_iap_dump_id                                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06100000)
#define	dumo_cmd_iap_validate_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07100000)
#define	dumo_cmd_iap_erase_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x08100000)
#define	dumo_cmd_iap_launch_app_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x09100000)
#define	dumo_cmd_iap_stop_server_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x0A100000)
#define	dumo_cmd_util_atoi_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00110000)
#define	dumo_cmd_util_itoa_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01110000)
#define	dumo_cmd_util_debug_print_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02110000)
#define	dumo_cmd_bt_l2cap_open_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00120000)
#define	dumo_cmd_bt_l2cap_start_server_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01120000)
#define	dumo_cmd_bt_hid_open_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00130000)
#define	dumo_cmd_bt_hid_start_server_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01130000)
#define	dumo_cmd_bt_hid_stop_server_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x02130000)
#define	dumo_cmd_bt_hid_send_input_report_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x03130000)
#define	dumo_cmd_bt_hid_virtual_cable_unplug_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x04130000)
#define	dumo_cmd_bt_hid_get_report_response_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x05130000)
#define	dumo_cmd_bt_hid_set_report_response_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x06130000)
#define	dumo_cmd_bt_hid_get_set_report_reject_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x07130000)
#define	dumo_cmd_le_serial_open_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x00140000)
#define	dumo_cmd_le_serial_listen_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_cmd|0x01140000)
#define	dumo_rsp_dfu_reset_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00000000)
#define	dumo_rsp_dfu_flash_set_address_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01000000)
#define	dumo_rsp_dfu_flash_upload_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02000000)
#define	dumo_rsp_dfu_flash_upload_finish_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03000000)
#define	dumo_rsp_system_hello_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00010000)
#define	dumo_rsp_system_reset_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01010000)
#define	dumo_rsp_system_set_max_power_mode_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02010000)
#define	dumo_rsp_system_get_bt_address_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03010000)
#define	dumo_rsp_system_set_bt_address_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04010000)
#define	dumo_rsp_system_get_class_of_device_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05010000)
#define	dumo_rsp_system_set_class_of_device_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06010000)
#define	dumo_rsp_system_reset_factory_settings_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07010000)
#define	dumo_rsp_system_set_local_name_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x08010000)
#define	dumo_rsp_system_get_local_name_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x09010000)
#define	dumo_rsp_system_write_bt_config_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A010000)
#define	dumo_rsp_bt_gap_discover_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00020000)
#define	dumo_rsp_bt_gap_cancel_discovery_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01020000)
#define	dumo_rsp_bt_gap_get_remote_name_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02020000)
#define	dumo_rsp_bt_gap_set_mode_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03020000)
#define	dumo_rsp_bt_gap_get_mode_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04020000)
#define	dumo_rsp_bt_gap_set_policy_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05020000)
#define	dumo_rsp_bt_gap_set_parameters_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06020000)
#define	dumo_rsp_bt_gap_set_max_power_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07020000)
#define	dumo_rsp_bt_gap_open_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x08020000)
#define	dumo_rsp_bt_gap_set_auto_sniff_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x09020000)
#define	dumo_rsp_bt_gap_set_discovery_mode_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A020000)
#define	dumo_rsp_bt_gap_set_host_channel_classification_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0B020000)
#define	dumo_rsp_le_gap_open_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00030000)
#define	dumo_rsp_le_gap_set_mode_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01030000)
#define	dumo_rsp_le_gap_discover_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02030000)
#define	dumo_rsp_le_gap_end_procedure_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03030000)
#define	dumo_rsp_le_gap_set_adv_parameters_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04030000)
#define	dumo_rsp_le_gap_set_conn_parameters_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05030000)
#define	dumo_rsp_le_gap_set_scan_parameters_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06030000)
#define	dumo_rsp_le_gap_set_adv_data_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07030000)
#define	dumo_rsp_le_gap_set_max_power_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x08030000)
#define	dumo_rsp_le_gap_set_host_channel_classification_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x09030000)
#define	dumo_rsp_le_gap_set_scan_result_filter_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A030000)
#define	dumo_rsp_le_gap_scan_filter_clear_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0B030000)
#define	dumo_rsp_le_gap_set_random_static_address_id                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0C030000)
#define	dumo_rsp_bt_rfcomm_open_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00040000)
#define	dumo_rsp_bt_rfcomm_start_server_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01040000)
#define	dumo_rsp_bt_rfcomm_open_port_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02040000)
#define	dumo_rsp_bt_rfcomm_stop_server_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03040000)
#define	dumo_rsp_bt_rfcomm_modem_status_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04040000)
#define	dumo_rsp_bt_rfcomm_start_server_port_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05040000)
#define	dumo_rsp_bt_rfcomm_stop_server_port_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06040000)
#define	dumo_rsp_bt_rfcomm_set_modem_status_default_id              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07040000)
#define	dumo_rsp_identity_remote_identity_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00050000)
#define	dumo_rsp_identity_local_identity_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01050000)
#define	dumo_rsp_identity_modify_local_identity_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02050000)
#define	dumo_rsp_bt_sdp_search_rfcomm_port_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00060000)
#define	dumo_rsp_bt_sdp_load_entry_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01060000)
#define	dumo_rsp_bt_sdp_add_entry_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02060000)
#define	dumo_rsp_bt_sdp_delete_entry_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03060000)
#define	dumo_rsp_bt_connection_list_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00070000)
#define	dumo_rsp_bt_connection_set_active_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01070000)
#define	dumo_rsp_bt_connection_set_sniff_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02070000)
#define	dumo_rsp_bt_connection_set_role_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03070000)
#define	dumo_rsp_bt_connection_set_supervision_timeout_id           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04070000)
#define	dumo_rsp_bt_connection_get_rssi_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05070000)
#define	dumo_rsp_bt_connection_read_clock_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06070000)
#define	dumo_rsp_le_connection_set_parameters_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00080000)
#define	dumo_rsp_le_connection_list_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01080000)
#define	dumo_rsp_gatt_set_max_mtu_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00090000)
#define	dumo_rsp_gatt_discover_primary_services_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01090000)
#define	dumo_rsp_gatt_discover_primary_services_by_uuid_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02090000)
#define	dumo_rsp_gatt_discover_characteristics_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03090000)
#define	dumo_rsp_gatt_discover_characteristics_by_uuid_id           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04090000)
#define	dumo_rsp_gatt_set_characteristic_notification_id            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05090000)
#define	dumo_rsp_gatt_discover_descriptors_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06090000)
#define	dumo_rsp_gatt_read_characteristic_value_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07090000)
#define	dumo_rsp_gatt_read_characteristic_value_by_uuid_id          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x08090000)
#define	dumo_rsp_gatt_write_characteristic_value_id                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x09090000)
#define	dumo_rsp_gatt_write_characteristic_value_without_response_id	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A090000)
#define	dumo_rsp_gatt_prepare_characteristic_value_write_id         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0B090000)
#define	dumo_rsp_gatt_execute_characteristic_value_write_id         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0C090000)
#define	dumo_rsp_gatt_send_characteristic_confirmation_id           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0D090000)
#define	dumo_rsp_gatt_read_descriptor_value_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0E090000)
#define	dumo_rsp_gatt_write_descriptor_value_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0F090000)
#define	dumo_rsp_gatt_find_included_services_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x10090000)
#define	dumo_rsp_gatt_read_multiple_characteristic_values_id        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x11090000)
#define	dumo_rsp_gatt_find_information_request_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x12090000)
#define	dumo_rsp_gatt_server_read_attribute_value_id                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x000A0000)
#define	dumo_rsp_gatt_server_read_attribute_type_id                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x010A0000)
#define	dumo_rsp_gatt_server_write_attribute_value_id               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x020A0000)
#define	dumo_rsp_gatt_server_send_user_read_response_id             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x030A0000)
#define	dumo_rsp_gatt_server_send_user_write_response_id            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x040A0000)
#define	dumo_rsp_gatt_server_send_characteristic_notification_id    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x050A0000)
#define	dumo_rsp_gatt_server_set_service_status_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x060A0000)
#define	dumo_rsp_endpoint_send_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x000B0000)
#define	dumo_rsp_endpoint_set_streaming_destination_id              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x010B0000)
#define	dumo_rsp_endpoint_close_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x020B0000)
#define	dumo_rsp_endpoint_set_flags_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x030B0000)
#define	dumo_rsp_endpoint_clr_flags_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x040B0000)
#define	dumo_rsp_endpoint_read_counters_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x050B0000)
#define	dumo_rsp_endpoint_set_active_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x060B0000)
#define	dumo_rsp_hardware_set_soft_timer_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x000C0000)
#define	dumo_rsp_hardware_configure_gpio_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x010C0000)
#define	dumo_rsp_hardware_write_gpio_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x020C0000)
#define	dumo_rsp_hardware_read_gpio_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x030C0000)
#define	dumo_rsp_hardware_read_adc_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x040C0000)
#define	dumo_rsp_hardware_read_i2c_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x050C0000)
#define	dumo_rsp_hardware_write_i2c_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x060C0000)
#define	dumo_rsp_hardware_stop_i2c_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x070C0000)
#define	dumo_rsp_hardware_set_uart_configuration_id                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x080C0000)
#define	dumo_rsp_hardware_read_vdd_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x090C0000)
#define	dumo_rsp_hardware_read_junction_temperature_id              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A0C0000)
#define	dumo_rsp_hardware_read_write_spi_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0B0C0000)
#define	dumo_rsp_hardware_set_uart_bgapi_mode_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0C0C0000)
#define	dumo_rsp_flash_ps_dump_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x000D0000)
#define	dumo_rsp_flash_ps_erase_all_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x010D0000)
#define	dumo_rsp_flash_ps_save_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x020D0000)
#define	dumo_rsp_flash_ps_load_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x030D0000)
#define	dumo_rsp_flash_ps_erase_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x040D0000)
#define	dumo_rsp_test_dtm_tx_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x000E0000)
#define	dumo_rsp_test_dtm_rx_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x010E0000)
#define	dumo_rsp_test_dtm_end_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x020E0000)
#define	dumo_rsp_test_tx_test_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x030E0000)
#define	dumo_rsp_test_debug_enable_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x040E0000)
#define	dumo_rsp_test_enable_uart_loopback_mode_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x050E0000)
#define	dumo_rsp_test_ssp_debug_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x060E0000)
#define	dumo_rsp_test_debug_command_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x070E0000)
#define	dumo_rsp_test_device_under_test_mode_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x080E0000)
#define	dumo_rsp_test_rx_test_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x090E0000)
#define	dumo_rsp_test_packet_test_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A0E0000)
#define	dumo_rsp_test_debug_counter_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0B0E0000)
#define	dumo_rsp_sm_set_bondable_mode_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x000F0000)
#define	dumo_rsp_sm_configure_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x010F0000)
#define	dumo_rsp_sm_store_bonding_configuration_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x020F0000)
#define	dumo_rsp_sm_read_bonding_configuration_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x030F0000)
#define	dumo_rsp_sm_increase_security_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x040F0000)
#define	dumo_rsp_sm_read_bonding_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x050F0000)
#define	dumo_rsp_sm_delete_bonding_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x060F0000)
#define	dumo_rsp_sm_delete_bondings_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x070F0000)
#define	dumo_rsp_sm_enter_passkey_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x080F0000)
#define	dumo_rsp_sm_passkey_confirm_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x090F0000)
#define	dumo_rsp_sm_set_oob_data_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A0F0000)
#define	dumo_rsp_sm_list_all_bondings_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0B0F0000)
#define	dumo_rsp_sm_confirm_bonding_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0C0F0000)
#define	dumo_rsp_sm_enter_pin_code_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0D0F0000)
#define	dumo_rsp_sm_set_le_bondable_mode_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0E0F0000)
#define	dumo_rsp_sm_set_le_key_enc_only_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0F0F0000)
#define	dumo_rsp_sm_set_minimum_key_size_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x100F0000)
#define	dumo_rsp_iap_open_id                                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00100000)
#define	dumo_rsp_iap_start_server_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01100000)
#define	dumo_rsp_iap_set_param_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02100000)
#define	dumo_rsp_iap_set_protocol_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03100000)
#define	dumo_rsp_iap_get_param_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04100000)
#define	dumo_rsp_iap_get_protocol_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05100000)
#define	dumo_rsp_iap_dump_id                                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06100000)
#define	dumo_rsp_iap_validate_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07100000)
#define	dumo_rsp_iap_erase_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x08100000)
#define	dumo_rsp_iap_launch_app_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x09100000)
#define	dumo_rsp_iap_stop_server_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x0A100000)
#define	dumo_rsp_util_atoi_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00110000)
#define	dumo_rsp_util_itoa_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01110000)
#define	dumo_rsp_util_debug_print_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02110000)
#define	dumo_rsp_bt_l2cap_open_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00120000)
#define	dumo_rsp_bt_l2cap_start_server_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01120000)
#define	dumo_rsp_bt_hid_open_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00130000)
#define	dumo_rsp_bt_hid_start_server_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01130000)
#define	dumo_rsp_bt_hid_stop_server_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x02130000)
#define	dumo_rsp_bt_hid_send_input_report_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x03130000)
#define	dumo_rsp_bt_hid_virtual_cable_unplug_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x04130000)
#define	dumo_rsp_bt_hid_get_report_response_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x05130000)
#define	dumo_rsp_bt_hid_set_report_response_id                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x06130000)
#define	dumo_rsp_bt_hid_get_set_report_reject_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x07130000)
#define	dumo_rsp_le_serial_open_id                                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x00140000)
#define	dumo_rsp_le_serial_listen_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_rsp|0x01140000)
#define	dumo_evt_dfu_boot_id                                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00000000)
#define	dumo_evt_system_boot_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00010000)
#define	dumo_evt_system_initialized_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01010000)
#define	dumo_evt_system_recovery_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02010000)
#define	dumo_evt_system_script_stopped_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03010000)
#define	dumo_evt_bt_gap_discovery_result_id                         	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00020000)
#define	dumo_evt_bt_gap_discovery_complete_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01020000)
#define	dumo_evt_bt_gap_remote_name_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02020000)
#define	dumo_evt_bt_gap_host_channel_classification_complete_id     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03020000)
#define	dumo_evt_le_gap_scan_response_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00030000)
#define	dumo_evt_le_gap_host_channel_classification_complete_id     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01030000)
#define	dumo_evt_bt_rfcomm_opened_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00040000)
#define	dumo_evt_bt_rfcomm_modem_status_id                          	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01040000)
#define	dumo_evt_bt_rfcomm_incoming_rejected_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02040000)
#define	dumo_evt_bt_rfcomm_credit_starvation_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03040000)
#define	dumo_evt_identity_remote_identity_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00050000)
#define	dumo_evt_bt_sdp_search_rfcomm_port_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00060000)
#define	dumo_evt_bt_connection_opened_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00070000)
#define	dumo_evt_bt_connection_closed_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01070000)
#define	dumo_evt_bt_connection_parameters_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02070000)
#define	dumo_evt_bt_connection_list_complete_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03070000)
#define	dumo_evt_bt_connection_rssi_value_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x04070000)
#define	dumo_evt_bt_connection_clock_value_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x05070000)
#define	dumo_evt_le_connection_opened_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00080000)
#define	dumo_evt_le_connection_closed_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01080000)
#define	dumo_evt_le_connection_parameters_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02080000)
#define	dumo_evt_le_connection_list_complete_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03080000)
#define	dumo_evt_gatt_mtu_exchanged_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00090000)
#define	dumo_evt_gatt_service_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01090000)
#define	dumo_evt_gatt_characteristic_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02090000)
#define	dumo_evt_gatt_descriptor_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03090000)
#define	dumo_evt_gatt_characteristic_value_id                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x04090000)
#define	dumo_evt_gatt_descriptor_value_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x05090000)
#define	dumo_evt_gatt_procedure_completed_id                        	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x06090000)
#define	dumo_evt_gatt_find_information_found_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x07090000)
#define	dumo_evt_gatt_server_attribute_value_id                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x000A0000)
#define	dumo_evt_gatt_server_user_read_request_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x010A0000)
#define	dumo_evt_gatt_server_user_write_request_id                  	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x020A0000)
#define	dumo_evt_gatt_server_characteristic_status_id               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x030A0000)
#define	dumo_evt_endpoint_syntax_error_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x000B0000)
#define	dumo_evt_endpoint_data_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x010B0000)
#define	dumo_evt_endpoint_status_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x020B0000)
#define	dumo_evt_endpoint_closing_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x030B0000)
#define	dumo_evt_endpoint_closed_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x040B0000)
#define	dumo_evt_hardware_soft_timer_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x000C0000)
#define	dumo_evt_hardware_interrupt_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x010C0000)
#define	dumo_evt_flash_ps_key_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x000D0000)
#define	dumo_evt_test_dtm_completed_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x000E0000)
#define	dumo_evt_test_debug_id                                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x010E0000)
#define	dumo_evt_test_hcidump_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x020E0000)
#define	dumo_evt_test_error_id                                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x030E0000)
#define	dumo_evt_sm_passkey_display_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x000F0000)
#define	dumo_evt_sm_passkey_request_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x010F0000)
#define	dumo_evt_sm_confirm_passkey_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x020F0000)
#define	dumo_evt_sm_bonded_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x030F0000)
#define	dumo_evt_sm_bonding_failed_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x040F0000)
#define	dumo_evt_sm_list_bonding_entry_id                           	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x050F0000)
#define	dumo_evt_sm_list_all_bondings_complete_id                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x060F0000)
#define	dumo_evt_sm_bonding_request_id                              	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x070F0000)
#define	dumo_evt_sm_pin_code_request_id                             	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x080F0000)
#define	dumo_evt_iap_param_id                                       	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00100000)
#define	dumo_evt_iap_protocol_id                                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01100000)
#define	dumo_evt_iap_opened_id                                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02100000)
#define	dumo_evt_iap_auth_ok_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03100000)
#define	dumo_evt_iap_auth_fail_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x04100000)
#define	dumo_evt_iap_started_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x05100000)
#define	dumo_evt_iap_stopped_id                                     	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x06100000)
#define	dumo_evt_iap_resume_id                                      	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x07100000)
#define	dumo_evt_iap_link_params_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x08100000)
#define	dumo_evt_bt_l2cap_opened_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00120000)
#define	dumo_evt_bt_l2cap_failed_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01120000)
#define	dumo_evt_bt_l2cap_closed_id                                 	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02120000)
#define	dumo_evt_bt_l2cap_connectionless_data_id                    	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03120000)
#define	dumo_evt_bt_hid_opened_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00130000)
#define	dumo_evt_bt_hid_failed_id                                   	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x01130000)
#define	dumo_evt_bt_hid_output_report_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x02130000)
#define	dumo_evt_bt_hid_state_changed_id                            	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x03130000)
#define	dumo_evt_bt_hid_get_report_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x04130000)
#define	dumo_evt_bt_hid_set_report_id                               	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x05130000)
#define	dumo_evt_le_serial_opened_id                                	(((uint32)dumo_dev_type_dumo)|dumo_msg_type_evt|0x00140000)
#ifndef BG_ERRORCODES
#define BG_ERRORCODES
enum dumo_error_spaces
{
	dumo_errspc_hardware=1280,
	dumo_errspc_bg=256,
	dumo_errspc_sdp=1536,
	dumo_errspc_smp=768,
	dumo_errspc_bt=512,
	dumo_errspc_application=2560,
	dumo_errspc_att=1024,
	dumo_errspc_filesystem=2304,
};
typedef enum dumo_error
{
	dumo_err_hardware_ps_store_full                                                         =dumo_errspc_hardware+1,   //Flash reserved for PS store is full
	dumo_err_hardware_ps_key_not_found                                                      =dumo_errspc_hardware+2,   //PS key not found
	dumo_err_hardware_i2c_ack_missing                                                       =dumo_errspc_hardware+3,   //Acknowledge for i2c was not received.
	dumo_err_hardware_i2c_timeout                                                           =dumo_errspc_hardware+4,   //I2C read or write timed out.
	dumo_err_hardware_not_configured                                                        =dumo_errspc_hardware+5,   //Hardware is not configured for that function.
	dumo_err_hardware_ble_not_supported                                                     =dumo_errspc_hardware+6,   //Hardware does not support Bluetooth Low Energy.
	dumo_err_success                                                                        =0,                        //No error
	dumo_err_invalid_conn_handle                                                            =dumo_errspc_bg+1,         //Invalid GATT connection handle.
	dumo_err_waiting_response                                                               =dumo_errspc_bg+2,         //Waiting response from GATT server to previous procedure.
	dumo_err_gatt_connection_timeout                                                        =dumo_errspc_bg+3,         //GATT connection is closed due procedure timeout.
	dumo_err_invalid_param                                                                  =dumo_errspc_bg+128,       //Command contained invalid parameter
	dumo_err_wrong_state                                                                    =dumo_errspc_bg+129,       //Device is in wrong state to receive command
	dumo_err_out_of_memory                                                                  =dumo_errspc_bg+130,       //Device has run out of memory
	dumo_err_not_implemented                                                                =dumo_errspc_bg+131,       //Feature is not implemented
	dumo_err_invalid_command                                                                =dumo_errspc_bg+132,       //Command was not recognized
	dumo_err_timeout                                                                        =dumo_errspc_bg+133,       //Command or Procedure failed due to timeout
	dumo_err_not_connected                                                                  =dumo_errspc_bg+134,       //Connection handle passed is to command is not a valid handle
	dumo_err_flow                                                                           =dumo_errspc_bg+135,       //Command would cause either underflow or overflow error
	dumo_err_user_attribute                                                                 =dumo_errspc_bg+136,       //User attribute was accessed through API which is not supported
	dumo_err_invalid_license_key                                                            =dumo_errspc_bg+137,       //No valid license key found
	dumo_err_command_too_long                                                               =dumo_errspc_bg+138,       //Command maximum length exceeded
	dumo_err_out_of_bonds                                                                   =dumo_errspc_bg+139,       //Bonding procedure can't be started because device has no space left for bond.
	dumo_err_unspecified                                                                    =dumo_errspc_bg+140,       //Unspecified error
	dumo_err_hardware                                                                       =dumo_errspc_bg+141,       //Hardware failure
	dumo_err_buffers_full                                                                   =dumo_errspc_bg+142,       //Command not accepted, because internal buffers are full
	dumo_err_disconnected                                                                   =dumo_errspc_bg+143,       //Command or Procedure failed due to disconnection
	dumo_err_too_many_requests                                                              =dumo_errspc_bg+144,       //Too many Simultaneous Requests
	dumo_err_not_supported                                                                  =dumo_errspc_bg+145,       //Feature is not supported in this firmware build
	dumo_err_server_already_in_use                                                          =dumo_errspc_bg+146,       //A server for the specified type of connection (SPP, HID, or L2CAP with specific PSM) was already started. Only one SPP server per SDP entry is supported; only one HID server altogether is supported; only one L2CAP sever per PSM is supported.
	dumo_err_no_such_endpoint                                                               =dumo_errspc_bg+147,       //The specified endpoint does not exist.
	dumo_err_invalid_endpoint                                                               =dumo_errspc_bg+148,       //The endpoint exists, but it does not support the BGAPI command executed.
            
	dumo_err_invalid_endpoint_state                                                         =dumo_errspc_bg+149,       //The endpoint could not execute the command in its current state.
            
	dumo_err_sdp_record_not_found                                                           =dumo_errspc_sdp+1,        //Service Record not found
	dumo_err_sdp_record_already_exist                                                       =dumo_errspc_sdp+2,        //Service Record with this handle already exist.
	dumo_err_smp_passkey_entry_failed                                                       =dumo_errspc_smp+1,        //The user input of passkey failed, for example, the user cancelled the operation
	dumo_err_smp_oob_not_available                                                          =dumo_errspc_smp+2,        //Out of Band data is not available for authentication
	dumo_err_smp_authentication_requirements                                                =dumo_errspc_smp+3,        //The pairing procedure cannot be performed as authentication requirements cannot be met due to IO capabilities of one or both devices
	dumo_err_smp_confirm_value_failed                                                       =dumo_errspc_smp+4,        //The confirm value does not match the calculated compare value
	dumo_err_smp_pairing_not_supported                                                      =dumo_errspc_smp+5,        //Pairing is not supported by the device
	dumo_err_smp_encryption_key_size                                                        =dumo_errspc_smp+6,        //The resultant encryption key size is insufficient for the security requirements of this device
	dumo_err_smp_command_not_supported                                                      =dumo_errspc_smp+7,        //The SMP command received is not supported on this device
	dumo_err_smp_unspecified_reason                                                         =dumo_errspc_smp+8,        //Pairing failed due to an unspecified reason
	dumo_err_smp_repeated_attempts                                                          =dumo_errspc_smp+9,        //Pairing or authentication procedure is disallowed because too little time has elapsed since last pairing request or security request
	dumo_err_smp_invalid_parameters                                                         =dumo_errspc_smp+10,       //The Invalid Parameters error code indicates: the command length is invalid or a parameter is outside of the specified range.
	dumo_err_smp_no_bonding                                                                 =dumo_errspc_smp+11,       //The bonding does not exist.
	dumo_err_bt_error_success                                                               =dumo_errspc_bt+0,         //Command completed succesfully
	dumo_err_bt_unknown_connection_identifier                                               =dumo_errspc_bt+2,         //A command was sent from the Host that should identify a connection, but that connection does not exist. 
	dumo_err_bt_page_timeout                                                                =dumo_errspc_bt+4,         //The Page Timeout error code indicates that a page timed out because of the Page Timeout configuration parameter. 
	dumo_err_bt_authentication_failure                                                      =dumo_errspc_bt+5,         //Pairing or authentication failed due to incorrect results in the pairing or authentication procedure. This could be due to an incorrect PIN or Link Key
	dumo_err_bt_pin_or_key_missing                                                          =dumo_errspc_bt+6,         //Pairing failed because of missing PIN, or authentication failed because of missing Key
	dumo_err_bt_memory_capacity_exceeded                                                    =dumo_errspc_bt+7,         //Controller is out of memory.
	dumo_err_bt_connection_timeout                                                          =dumo_errspc_bt+8,         //Link supervision timeout has expired.
	dumo_err_bt_connection_limit_exceeded                                                   =dumo_errspc_bt+9,         //Controller is at limit of connections it can support.
	dumo_err_bt_synchronous_connectiontion_limit_exceeded                                   =dumo_errspc_bt+10,        //The Synchronous Connection Limit to a Device Exceeded error code indicates that the Controller has reached the limit to the number of synchronous connections that can be achieved to a device. 
	dumo_err_bt_acl_connection_already_exists                                               =dumo_errspc_bt+11,        //The ACL Connection Already Exists error code indicates that an attempt to create a new ACL Connection to a device when there is already a connection to this device.
	dumo_err_bt_command_disallowed                                                          =dumo_errspc_bt+12,        //Command requested cannot be executed because the Controller is in a state where it cannot process this command at this time.
	dumo_err_bt_connection_rejected_due_to_limited_resources                                =dumo_errspc_bt+13,        //The Connection Rejected Due To Limited Resources error code indicates that an incoming connection was rejected due to limited resources.
	dumo_err_bt_connection_rejected_due_to_security_reasons                                 =dumo_errspc_bt+14,        //The Connection Rejected Due To Security Reasons error code indicates that a connection was rejected due to security requirements not being fulfilled, like authentication or pairing.
	dumo_err_bt_connection_rejected_due_to_unacceptable_bd_addr                             =dumo_errspc_bt+15,        //The Connection was rejected because this device does not accept the BD_ADDR. This may be because the device will only accept connections from specific BD_ADDRs.
	dumo_err_bt_connection_accept_timeout_exceeded                                          =dumo_errspc_bt+16,        //The Connection Accept Timeout has been exceeded for this connection attempt.
	dumo_err_bt_unsupported_feature_or_parameter_value                                      =dumo_errspc_bt+17,        //A feature or parameter value in the HCI command is not supported.
	dumo_err_bt_invalid_command_parameters                                                  =dumo_errspc_bt+18,        //Command contained invalid parameters.
	dumo_err_bt_remote_user_terminated                                                      =dumo_errspc_bt+19,        //User on the remote device terminated the connection.
	dumo_err_bt_remote_device_terminated_connection_due_to_low_resources                    =dumo_errspc_bt+20,        //The remote device terminated the connection because of low resources
	dumo_err_bt_remote_powering_off                                                         =dumo_errspc_bt+21,        //Remote Device Terminated Connection due to Power Off
	dumo_err_bt_connection_terminated_by_local_host                                         =dumo_errspc_bt+22,        //Local device terminated the connection.
	dumo_err_bt_repeated_attempts                                                           =dumo_errspc_bt+23,        //The Controller is disallowing an authentication or pairing procedure because too little time has elapsed since the last authentication or pairing attempt failed.
	dumo_err_bt_pairing_not_allowed                                                         =dumo_errspc_bt+24,        //The device does not allow pairing. This can be for example, when a device only allows pairing during a certain time window after some user input allows pairing
	dumo_err_bt_unknown_lmp_pdu                                                             =dumo_errspc_bt+25,        //The Controller has received an unknown LMP OpCode.
	dumo_err_bt_unsupported_remote_feature                                                  =dumo_errspc_bt+26,        //The remote device does not support the feature associated with the issued command or LMP PDU.
	dumo_err_bt_sco_offset_rejected                                                         =dumo_errspc_bt+27,        //The offset requested in the LMP_SCO_link_req PDU has been rejected.
	dumo_err_bt_sco_interval_rejected                                                       =dumo_errspc_bt+28,        //The interval requested in the LMP_SCO_link_req PDU has been rejected.
	dumo_err_bt_sco_air_mode_rejected                                                       =dumo_errspc_bt+29,        //The air mode requested in the LMP_SCO_link_req PDU has been rejected.
	dumo_err_bt_invalid_lmp_parameters                                                      =dumo_errspc_bt+30,        //Some LMP PDU / LL Control PDU parameters were invalid. 
	dumo_err_bt_unspecified_error                                                           =dumo_errspc_bt+31,        //No other error code specified is appropriate to use.
	dumo_err_bt_unsupported_lmp_parameter_value                                             =dumo_errspc_bt+32,        //An LMP PDU or an LL Control PDU contains at least one parameter value that is not supported by the Controller at this time.
	dumo_err_bt_role_change_not_allowed                                                     =dumo_errspc_bt+33,        //Controller will not allow a role change at this time.
	dumo_err_bt_ll_response_timeout                                                         =dumo_errspc_bt+34,        //Connection terminated due to link-layer procedure timeout.
	dumo_err_bt_lmp_error_transaction_collision                                             =dumo_errspc_bt+35,        //LMP transaction has collided with the same transaction that is already in progress.
	dumo_err_bt_lmp_pdu_not_allowed                                                         =dumo_errspc_bt+36,        //Controller sent an LMP PDU with an OpCode that was not allowed.
	dumo_err_bt_encryption_mode_not_acceptable                                              =dumo_errspc_bt+37,        //The requested encryption mode is not acceptable at this time.
	dumo_err_bt_link_key_cannot_be_changed                                                  =dumo_errspc_bt+38,        //Link key cannot be changed because a fixed unit key is being used.
	dumo_err_bt_requested_qos_not_supported                                                 =dumo_errspc_bt+39,        //The requested Quality of Service is not supported.
	dumo_err_bt_instant_passed                                                              =dumo_errspc_bt+40,        //LMP PDU or LL PDU that includes an instant cannot be performed because the instant when this would have occurred has passed.
	dumo_err_bt_pairing_with_unit_key_not_supported                                         =dumo_errspc_bt+41,        //It was not possible to pair as a unit key was requested and it is not supported.
	dumo_err_bt_different_transaction_collision                                             =dumo_errspc_bt+42,        //LMP transaction was started that collides with an ongoing transaction.
	dumo_err_bt_qos_unacceptable_parameter                                                  =dumo_errspc_bt+44,        //The specified quality of service parameters could not be accepted at this time, but other parameters may be acceptable.
	dumo_err_bt_qos_rejected                                                                =dumo_errspc_bt+45,        //The specified quality of service parameters cannot be accepted and QoS negotiation should be terminated.
	dumo_err_bt_channel_assesment_not_supported                                             =dumo_errspc_bt+46,        //The Controller cannot perform channel assessment because it is not supported.
	dumo_err_bt_insufficient_security                                                       =dumo_errspc_bt+47,        //The HCI command or LMP PDU sent is only possible on an encrypted link.
	dumo_err_bt_parameter_out_of_mandatory_range                                            =dumo_errspc_bt+48,        //A parameter value requested is outside the mandatory range of parameters for the given HCI command or LMP PDU.
	dumo_err_bt_role_switch_pending                                                         =dumo_errspc_bt+50,        //Role Switch is pending. This can be used when an HCI command or LMP PDU cannot be accepted because of a pending role switch. This can also be used to notify a peer device about a pending role switch.
	dumo_err_bt_reserved_slot_violation                                                     =dumo_errspc_bt+52,        //The current Synchronous negotiation was terminated with the negotiation state set to Reserved Slot Violation.
	dumo_err_bt_role_switch_failed                                                          =dumo_errspc_bt+53,        //role switch was attempted but it failed and the original piconet structure is restored. The switch may have failed because the TDD switch or piconet switch failed.
	dumo_err_bt_extended_inquiry_response_too_large                                         =dumo_errspc_bt+54,        //The extended inquiry response, with the requested requirements for FEC, is too large to fit in any of the packet types supported by the Controller.
	dumo_err_bt_simple_pairing_not_supported_by_host                                        =dumo_errspc_bt+55,        //The IO capabilities request or response was rejected because the sending Host does not support Secure Simple Pairing even though the receiving Link Manager does.
	dumo_err_bt_host_busy_pairing                                                           =dumo_errspc_bt+56,        //The Host is busy with another pairing operation and unable to support the requested pairing. The receiving device should retry pairing again later.
	dumo_err_bt_connection_rejected_due_to_no_suitable_channel_found                        =dumo_errspc_bt+57,        //The Controller could not calculate an appropriate value for the Channel selection operation.
	dumo_err_bt_controller_busy                                                             =dumo_errspc_bt+58,        //Operation was rejected because the controller is busy and unable to process the request.
	dumo_err_bt_unacceptable_connection_interval                                            =dumo_errspc_bt+59,        //Remote evice terminated the connection because of an unacceptable connection interval.
	dumo_err_bt_directed_advertising_timeout                                                =dumo_errspc_bt+60,        //Directed advertising completed without a connection being created.
	dumo_err_bt_connection_terminated_due_to_mic_failure                                    =dumo_errspc_bt+61,        //Connection was terminated because the Message Integrity Check (MIC) failed on a received packet.
	dumo_err_bt_connection_failed_to_be_established                                         =dumo_errspc_bt+62,        //LL initiated a connection but the connection has failed to be established. Controller did not receive any packets from remote end.
	dumo_err_bt_mac_connection_failed                                                       =dumo_errspc_bt+63,        //The MAC of the 802.11 AMP was requested to connect to a peer, but the connection failed.
	dumo_err_bt_coarse_clock_adjustment_rejected_but_will_try_to_adjust_using_clock_dragging=dumo_errspc_bt+64,        //The Central, at this time, is unable to make a coarse adjustment to the piconet clock, using the supplied parameters. Instead the Central will attempt to move the clock using clock dragging.
	dumo_err_bt_refused_psm_not_supported                                                   =dumo_errspc_bt+65,        //The remote end did not have a service that would accept L2CAP connection requests to the specified PSM.
	dumo_err_bt_refused_security_block                                                      =dumo_errspc_bt+66,        //The L2CAP connection attempt was blocked by the remote end due to security reasons.
	dumo_err_bt_refused_no_resources                                                        =dumo_errspc_bt+67,        //The remote end rejected the L2CAP connection due to lack of resources.
	dumo_err_bt_acl_disconnected                                                            =dumo_errspc_bt+68,        //The ACL carrying the L2CAP link was disconnected.
	dumo_err_bt_psm_already_in_use                                                          =dumo_errspc_bt+69,        //An L2CAP connection with the specified PSM is already open on the same ACL. For each ACL, only one connection can use the same PSM.
	dumo_err_application_file_open_failed                                                   =dumo_errspc_application+1,//File open failed.
	dumo_err_application_xml_parse_failed                                                   =dumo_errspc_application+2,//XML parsing failed.
	dumo_err_application_device_connection_failed                                           =dumo_errspc_application+3,//Device connection failed.
	dumo_err_application_device_comunication_failed                                         =dumo_errspc_application+4,//Device communication failed.
	dumo_err_att_invalid_handle                                                             =dumo_errspc_att+1,        //The attribute handle given was not valid on this server
	dumo_err_att_read_not_permitted                                                         =dumo_errspc_att+2,        //The attribute cannot be read
	dumo_err_att_write_not_permitted                                                        =dumo_errspc_att+3,        //The attribute cannot be written
	dumo_err_att_invalid_pdu                                                                =dumo_errspc_att+4,        //The attribute PDU was invalid
	dumo_err_att_insufficient_authentication                                                =dumo_errspc_att+5,        //The attribute requires authentication before it can be read or written.
	dumo_err_att_request_not_supported                                                      =dumo_errspc_att+6,        //Attribute Server does not support the request received from the client.
	dumo_err_att_invalid_offset                                                             =dumo_errspc_att+7,        //Offset specified was past the end of the attribute
	dumo_err_att_insufficient_authorization                                                 =dumo_errspc_att+8,        //The attribute requires authorization before it can be read or written.
	dumo_err_att_prepare_queue_full                                                         =dumo_errspc_att+9,        //Too many prepare writes have been queueud
	dumo_err_att_att_not_found                                                              =dumo_errspc_att+10,       //No attribute found within the given attribute handle range.
	dumo_err_att_att_not_long                                                               =dumo_errspc_att+11,       //The attribute cannot be read or written using the Read Blob Request
	dumo_err_att_insufficient_enc_key_size                                                  =dumo_errspc_att+12,       //The Encryption Key Size used for encrypting this link is insufficient.
	dumo_err_att_invalid_att_length                                                         =dumo_errspc_att+13,       //The attribute value length is invalid for the operation
	dumo_err_att_unlikely_error                                                             =dumo_errspc_att+14,       //The attribute request that was requested has encountered an error that was unlikely, and therefore could not be completed as requested.
	dumo_err_att_insufficient_encryption                                                    =dumo_errspc_att+15,       //The attribute requires encryption before it can be read or written.
	dumo_err_att_unsupported_group_type                                                     =dumo_errspc_att+16,       //The attribute type is not a supported grouping attribute as defined by a higher layer specification.
	dumo_err_att_insufficient_resources                                                     =dumo_errspc_att+17,       //Insufficient Resources to complete the request
	dumo_err_att_application                                                                =dumo_errspc_att+128,      //Application error code defined by a higher layer specification.
	dumo_err_filesystem_file_not_found                                                      =dumo_errspc_filesystem+1, //File not found
	dumo_err_last
}errorcode_t;
#endif
PACKSTRUCT(struct dumo_msg_dfu_reset_cmd_t
{
	uint8	dfu;
});

PACKSTRUCT(struct dumo_msg_dfu_flash_set_address_cmd_t
{
	uint32	address;
});

PACKSTRUCT(struct dumo_msg_dfu_flash_set_address_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_dfu_flash_upload_cmd_t
{
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_dfu_flash_upload_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_dfu_flash_upload_finish_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_dfu_boot_evt_t
{
	uint32	version;
});

PACKSTRUCT(struct dumo_msg_system_hello_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_reset_cmd_t
{
	uint8	dfu;
});

PACKSTRUCT(struct dumo_msg_system_set_max_power_mode_cmd_t
{
	uint8	power_mode;
});

PACKSTRUCT(struct dumo_msg_system_set_max_power_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_get_bt_address_rsp_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_system_set_bt_address_cmd_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_system_set_bt_address_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_get_class_of_device_rsp_t
{
	uint32	cod;
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_set_class_of_device_cmd_t
{
	uint32	cod;
});

PACKSTRUCT(struct dumo_msg_system_set_class_of_device_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_reset_factory_settings_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_set_local_name_cmd_t
{
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_system_set_local_name_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_get_local_name_rsp_t
{
	uint16	result;
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_system_write_bt_config_cmd_t
{
	uint8	max_incoming_acl;
	uint8	rfcomm_credits_max;
	uint16	rfcomm_mtu;
});

PACKSTRUCT(struct dumo_msg_system_write_bt_config_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_system_boot_evt_t
{
	uint16	major;
	uint16	minor;
	uint16	patch;
	uint16	build;
	uint16	bootloader;
	uint16	hw;
});

PACKSTRUCT(struct dumo_msg_system_initialized_evt_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_system_recovery_evt_t
{
	uint32	id1;
	uint32	id2;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_system_script_stopped_evt_t
{
	uint8	event_class;
	uint8	event_id;
});

PACKSTRUCT(struct dumo_msg_bt_gap_discover_cmd_t
{
	uint8	timeout;
	int32	lap;
});

PACKSTRUCT(struct dumo_msg_bt_gap_discover_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_cancel_discovery_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_get_remote_name_cmd_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_bt_gap_get_remote_name_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_mode_cmd_t
{
	uint8	connectable;
	uint8	discoverable;
	uint8	limited;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_get_mode_rsp_t
{
	uint16	result;
	uint8	connectable;
	uint8	discoverable;
	uint8	limited;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_policy_cmd_t
{
	uint8	allow_role_change;
	uint8	allow_sniff;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_policy_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_parameters_cmd_t
{
	uint16	pagetimeout;
	uint16	scan_interval;
	uint16	scan_window;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_parameters_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_max_power_cmd_t
{
	int8	max_power;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_max_power_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_open_cmd_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_bt_gap_open_rsp_t
{
	uint16	result;
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_auto_sniff_cmd_t
{
	uint8	mode;
	uint16	idle;
	uint16	max;
	uint16	min;
	uint16	attempt;
	uint16	timeout;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_auto_sniff_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_discovery_mode_cmd_t
{
	uint8	mode;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_discovery_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_host_channel_classification_cmd_t
{
	uint8array	channel_map;
});

PACKSTRUCT(struct dumo_msg_bt_gap_set_host_channel_classification_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_gap_discovery_result_evt_t
{
	bd_addr	bd_addr;
	uint8	page_scan_repetition_mode;
	uint32	class_of_device;
	int8	rssi;
	uint8	bonding;
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_bt_gap_discovery_complete_evt_t
{
	uint16	status;
});

PACKSTRUCT(struct dumo_msg_bt_gap_remote_name_evt_t
{
	uint16	status;
	bd_addr	address;
	uint8array	remote_name;
});

PACKSTRUCT(struct dumo_msg_bt_gap_host_channel_classification_complete_evt_t
{
	uint16	status;
});

PACKSTRUCT(struct dumo_msg_le_gap_open_cmd_t
{
	bd_addr	address;
	uint8	address_type;
});

PACKSTRUCT(struct dumo_msg_le_gap_open_rsp_t
{
	uint16	result;
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_mode_cmd_t
{
	uint8	discover;
	uint8	connect;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_discover_cmd_t
{
	uint8	mode;
});

PACKSTRUCT(struct dumo_msg_le_gap_discover_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_end_procedure_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_adv_parameters_cmd_t
{
	uint16	interval_min;
	uint16	interval_max;
	uint8	channel_map;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_adv_parameters_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_conn_parameters_cmd_t
{
	uint16	min_interval;
	uint16	max_interval;
	uint16	latency;
	uint16	timeout;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_conn_parameters_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_scan_parameters_cmd_t
{
	uint16	scan_interval;
	uint16	scan_window;
	uint8	active;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_scan_parameters_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_adv_data_cmd_t
{
	uint8	scan_rsp;
	uint8array	adv_data;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_adv_data_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_max_power_cmd_t
{
	int8	max_power;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_max_power_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_host_channel_classification_cmd_t
{
	uint8array	channel_map;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_host_channel_classification_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_scan_result_filter_cmd_t
{
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_scan_result_filter_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_scan_filter_clear_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_random_static_address_cmd_t
{
	bd_addr	static_address;
	uint8	enable;
});

PACKSTRUCT(struct dumo_msg_le_gap_set_random_static_address_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_gap_scan_response_evt_t
{
	int8	rssi;
	uint8	packet_type;
	bd_addr	address;
	uint8	address_type;
	uint8	bonding;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_le_gap_host_channel_classification_complete_evt_t
{
	uint16	status;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_open_cmd_t
{
	bd_addr	address;
	uint8	streaming_destination;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_open_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_start_server_cmd_t
{
	uint8	sdp_id;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_start_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_open_port_cmd_t
{
	bd_addr	address;
	uint8	streaming_destination;
	uint8	port;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_open_port_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_stop_server_cmd_t
{
	uint8	sdp_id;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_stop_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_modem_status_cmd_t
{
	uint8	endpoint;
	uint8	modem;
	uint8	mask;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_modem_status_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_start_server_port_cmd_t
{
	uint8	port;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_start_server_port_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_stop_server_port_cmd_t
{
	uint8	port;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_stop_server_port_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_set_modem_status_default_cmd_t
{
	uint8	modem;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_set_modem_status_default_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_opened_evt_t
{
	uint8	endpoint;
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_modem_status_evt_t
{
	uint8	endpoint;
	uint8	modem;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_incoming_rejected_evt_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_bt_rfcomm_credit_starvation_evt_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_identity_remote_identity_cmd_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_identity_remote_identity_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_identity_local_identity_rsp_t
{
	uint16	result;
	uint16	source;
	uint16	vendor;
	uint16	product;
	uint16	version;
	uint8array	description;
});

PACKSTRUCT(struct dumo_msg_identity_modify_local_identity_cmd_t
{
	uint16	vendor;
	uint16	product;
	uint16	version;
});

PACKSTRUCT(struct dumo_msg_identity_modify_local_identity_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_identity_remote_identity_evt_t
{
	uint16	status;
	bd_addr	address;
	uint16	source;
	uint16	vendor;
	uint16	product;
	uint16	version;
	uint8array	description;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_search_rfcomm_port_cmd_t
{
	bd_addr	address;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_search_rfcomm_port_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_load_entry_cmd_t
{
	uint32	sdp_id;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_load_entry_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_add_entry_cmd_t
{
	uint32	sdp_id;
	uint8array	record;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_add_entry_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_delete_entry_cmd_t
{
	uint32	sdp_id;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_delete_entry_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_sdp_search_rfcomm_port_evt_t
{
	uint32	request_handle;
	uint16	status;
	uint8	port;
	uint8array	service_name;
});

PACKSTRUCT(struct dumo_msg_bt_connection_list_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_active_cmd_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_active_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_sniff_cmd_t
{
	uint8	endpoint;
	uint16	max;
	uint16	min;
	uint16	attempt;
	uint16	timeout;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_sniff_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_role_cmd_t
{
	uint8	endpoint;
	uint8	role;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_role_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_supervision_timeout_cmd_t
{
	uint8	endpoint;
	uint16	supervisiontimeout;
});

PACKSTRUCT(struct dumo_msg_bt_connection_set_supervision_timeout_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_get_rssi_cmd_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_bt_connection_get_rssi_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_read_clock_cmd_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_bt_connection_read_clock_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_connection_opened_evt_t
{
	bd_addr	address;
	uint8	central;
	uint8	connection;
	uint8	bonding;
});

PACKSTRUCT(struct dumo_msg_bt_connection_closed_evt_t
{
	uint16	reason;
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_bt_connection_parameters_evt_t
{
	uint8	endpoint;
	uint32	block_size;
	uint8	msc;
	bd_addr	address;
	uint8	direction;
	uint8	powermode;
	uint8	role;
	uint8	encryption;
	uint32	input_buffer;
	uint8	port;
});

PACKSTRUCT(struct dumo_msg_bt_connection_rssi_value_evt_t
{
	uint8	connection;
	int8	rssi;
});

PACKSTRUCT(struct dumo_msg_bt_connection_clock_value_evt_t
{
	uint8	connection;
	uint32	clock;
	uint16	accuracy;
});

PACKSTRUCT(struct dumo_msg_le_connection_set_parameters_cmd_t
{
	uint8	connection;
	uint16	min_interval;
	uint16	max_interval;
	uint16	latency;
	uint16	timeout;
});

PACKSTRUCT(struct dumo_msg_le_connection_set_parameters_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_connection_list_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_le_connection_opened_evt_t
{
	bd_addr	address;
	uint8	address_type;
	uint8	central;
	uint8	connection;
	uint8	bonding;
});

PACKSTRUCT(struct dumo_msg_le_connection_closed_evt_t
{
	uint16	reason;
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_le_connection_parameters_evt_t
{
	uint8	connection;
	uint16	interval;
	uint16	latency;
	uint16	timeout;
	uint8	security_mode;
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_gatt_set_max_mtu_cmd_t
{
	uint16	max_mtu;
});

PACKSTRUCT(struct dumo_msg_gatt_set_max_mtu_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_primary_services_cmd_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_primary_services_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_primary_services_by_uuid_cmd_t
{
	uint8	connection;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_primary_services_by_uuid_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_characteristics_cmd_t
{
	uint8	connection;
	uint32	service;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_characteristics_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_characteristics_by_uuid_cmd_t
{
	uint8	connection;
	uint32	service;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_characteristics_by_uuid_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_set_characteristic_notification_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	flags;
});

PACKSTRUCT(struct dumo_msg_gatt_set_characteristic_notification_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_descriptors_cmd_t
{
	uint8	connection;
	uint16	characteristic;
});

PACKSTRUCT(struct dumo_msg_gatt_discover_descriptors_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_read_characteristic_value_cmd_t
{
	uint8	connection;
	uint16	characteristic;
});

PACKSTRUCT(struct dumo_msg_gatt_read_characteristic_value_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_read_characteristic_value_by_uuid_cmd_t
{
	uint8	connection;
	uint32	service;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_read_characteristic_value_by_uuid_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_write_characteristic_value_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_write_characteristic_value_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_write_characteristic_value_without_response_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_write_characteristic_value_without_response_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_prepare_characteristic_value_write_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint16	offset;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_prepare_characteristic_value_write_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_execute_characteristic_value_write_cmd_t
{
	uint8	connection;
	uint8	flags;
});

PACKSTRUCT(struct dumo_msg_gatt_execute_characteristic_value_write_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_send_characteristic_confirmation_cmd_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_gatt_send_characteristic_confirmation_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_read_descriptor_value_cmd_t
{
	uint8	connection;
	uint16	descriptor;
});

PACKSTRUCT(struct dumo_msg_gatt_read_descriptor_value_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_write_descriptor_value_cmd_t
{
	uint8	connection;
	uint16	descriptor;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_write_descriptor_value_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_find_included_services_cmd_t
{
	uint8	connection;
	uint32	service;
});

PACKSTRUCT(struct dumo_msg_gatt_find_included_services_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_read_multiple_characteristic_values_cmd_t
{
	uint8	connection;
	uint8array	characteristic_list;
});

PACKSTRUCT(struct dumo_msg_gatt_read_multiple_characteristic_values_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_find_information_request_cmd_t
{
	uint8	connection;
	uint16	start_handle;
	uint16	end_handle;
});

PACKSTRUCT(struct dumo_msg_gatt_find_information_request_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_mtu_exchanged_evt_t
{
	uint8	connection;
	uint16	mtu;
});

PACKSTRUCT(struct dumo_msg_gatt_service_evt_t
{
	uint8	connection;
	uint32	service;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_characteristic_evt_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	properties;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_descriptor_evt_t
{
	uint8	connection;
	uint16	descriptor;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_characteristic_value_evt_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	att_opcode;
	uint16	offset;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_descriptor_value_evt_t
{
	uint8	connection;
	uint16	descriptor;
	uint16	offset;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_procedure_completed_evt_t
{
	uint8	connection;
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_find_information_found_evt_t
{
	uint8	connection;
	uint16	handle;
	uint8array	uuid;
});

PACKSTRUCT(struct dumo_msg_gatt_server_read_attribute_value_cmd_t
{
	uint16	attribute;
	uint16	offset;
});

PACKSTRUCT(struct dumo_msg_gatt_server_read_attribute_value_rsp_t
{
	uint16	result;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_server_read_attribute_type_cmd_t
{
	uint16	attribute;
});

PACKSTRUCT(struct dumo_msg_gatt_server_read_attribute_type_rsp_t
{
	uint16	result;
	uint8array	type;
});

PACKSTRUCT(struct dumo_msg_gatt_server_write_attribute_value_cmd_t
{
	uint16	attribute;
	uint16	offset;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_server_write_attribute_value_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_server_send_user_read_response_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	att_errorcode;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_server_send_user_read_response_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_server_send_user_write_response_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	att_errorcode;
});

PACKSTRUCT(struct dumo_msg_gatt_server_send_user_write_response_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_server_send_characteristic_notification_cmd_t
{
	uint8	connection;
	uint16	characteristic;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_server_send_characteristic_notification_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_server_set_service_status_cmd_t
{
	uint16	service;
	uint8	enable;
});

PACKSTRUCT(struct dumo_msg_gatt_server_set_service_status_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_gatt_server_attribute_value_evt_t
{
	uint8	connection;
	uint16	attribute;
	uint8	att_opcode;
	uint16	offset;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_server_user_read_request_evt_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	att_opcode;
	uint16	offset;
});

PACKSTRUCT(struct dumo_msg_gatt_server_user_write_request_evt_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	att_opcode;
	uint16	offset;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_gatt_server_characteristic_status_evt_t
{
	uint8	connection;
	uint16	characteristic;
	uint8	status_flags;
	uint16	client_config_flags;
});

PACKSTRUCT(struct dumo_msg_endpoint_send_cmd_t
{
	uint8	endpoint;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_endpoint_send_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_set_streaming_destination_cmd_t
{
	uint8	endpoint;
	uint8	destination_endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_set_streaming_destination_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_endpoint_close_cmd_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_close_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_set_flags_cmd_t
{
	uint8	endpoint;
	uint32	flags;
});

PACKSTRUCT(struct dumo_msg_endpoint_set_flags_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_clr_flags_cmd_t
{
	uint8	endpoint;
	uint32	flags;
});

PACKSTRUCT(struct dumo_msg_endpoint_clr_flags_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_read_counters_cmd_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_read_counters_rsp_t
{
	uint16	result;
	uint8	endpoint;
	uint32	tx;
	uint32	rx;
});

PACKSTRUCT(struct dumo_msg_endpoint_set_active_cmd_t
{
	uint8	endpoint;
	uint8	active;
});

PACKSTRUCT(struct dumo_msg_endpoint_set_active_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_endpoint_syntax_error_evt_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_data_evt_t
{
	uint8	endpoint;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_endpoint_status_evt_t
{
	uint8	endpoint;
	uint32	type;
	int8	destination_endpoint;
	uint8	flags;
});

PACKSTRUCT(struct dumo_msg_endpoint_closing_evt_t
{
	uint16	reason;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_endpoint_closed_evt_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_hardware_set_soft_timer_cmd_t
{
	uint32	time;
	uint8	timer_id;
	uint8	single_shot;
});

PACKSTRUCT(struct dumo_msg_hardware_set_soft_timer_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_configure_gpio_cmd_t
{
	uint8	port;
	uint16	gpio;
	uint8	mode;
	uint8	pullup;
});

PACKSTRUCT(struct dumo_msg_hardware_configure_gpio_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_write_gpio_cmd_t
{
	uint8	port;
	uint16	mask;
	uint16	data;
});

PACKSTRUCT(struct dumo_msg_hardware_write_gpio_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_read_gpio_cmd_t
{
	uint8	port;
	uint16	mask;
});

PACKSTRUCT(struct dumo_msg_hardware_read_gpio_rsp_t
{
	uint16	result;
	uint16	data;
});

PACKSTRUCT(struct dumo_msg_hardware_read_adc_cmd_t
{
	uint8	input;
});

PACKSTRUCT(struct dumo_msg_hardware_read_adc_rsp_t
{
	uint16	result;
	uint8	input;
	uint16	value;
});

PACKSTRUCT(struct dumo_msg_hardware_read_i2c_cmd_t
{
	uint8	channel;
	uint16	slave_address;
	uint8	length;
});

PACKSTRUCT(struct dumo_msg_hardware_read_i2c_rsp_t
{
	uint16	result;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_hardware_write_i2c_cmd_t
{
	uint8	channel;
	uint16	slave_address;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_hardware_write_i2c_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_stop_i2c_cmd_t
{
	uint8	channel;
});

PACKSTRUCT(struct dumo_msg_hardware_stop_i2c_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_set_uart_configuration_cmd_t
{
	uint8	endpoint;
	uint32	rate;
	uint8	data_bits;
	uint8	stop_bits;
	uint8	parity;
	uint8	flow_ctrl;
});

PACKSTRUCT(struct dumo_msg_hardware_set_uart_configuration_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_read_vdd_rsp_t
{
	uint16	result;
	uint16	value;
});

PACKSTRUCT(struct dumo_msg_hardware_read_junction_temperature_rsp_t
{
	uint16	result;
	uint16	value;
});

PACKSTRUCT(struct dumo_msg_hardware_read_write_spi_cmd_t
{
	uint8	channel;
	uint8	cs_port;
	uint8	cs_pin;
	uint8	cs_polarity;
	uint8	read_length;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_hardware_read_write_spi_rsp_t
{
	uint16	result;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_hardware_set_uart_bgapi_mode_cmd_t
{
	uint8	bgapi_mode;
});

PACKSTRUCT(struct dumo_msg_hardware_set_uart_bgapi_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_hardware_soft_timer_evt_t
{
	uint8	timer_id;
});

PACKSTRUCT(struct dumo_msg_hardware_interrupt_evt_t
{
	uint16	interrupts;
	uint32	timestamp;
});

PACKSTRUCT(struct dumo_msg_flash_ps_dump_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_flash_ps_erase_all_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_flash_ps_save_cmd_t
{
	uint16	key;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_flash_ps_save_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_flash_ps_load_cmd_t
{
	uint16	key;
});

PACKSTRUCT(struct dumo_msg_flash_ps_load_rsp_t
{
	uint16	result;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_flash_ps_erase_cmd_t
{
	uint16	key;
});

PACKSTRUCT(struct dumo_msg_flash_ps_erase_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_flash_ps_key_evt_t
{
	uint16	key;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_test_dtm_tx_cmd_t
{
	uint8	packet_type;
	uint8	length;
	uint8	channel;
	uint16	number_of_packets;
});

PACKSTRUCT(struct dumo_msg_test_dtm_tx_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_dtm_rx_cmd_t
{
	uint8	channel;
});

PACKSTRUCT(struct dumo_msg_test_dtm_rx_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_dtm_end_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_tx_test_cmd_t
{
	uint8	modulation;
	uint8	channel;
	uint8	power;
});

PACKSTRUCT(struct dumo_msg_test_tx_test_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_debug_enable_cmd_t
{
	uint8	id;
	uint8	enable;
});

PACKSTRUCT(struct dumo_msg_test_ssp_debug_cmd_t
{
	uint8	enable;
});

PACKSTRUCT(struct dumo_msg_test_debug_command_cmd_t
{
	uint8	id;
	uint8array	debugdata;
});

PACKSTRUCT(struct dumo_msg_test_debug_command_rsp_t
{
	uint16	result;
	uint8	id;
	uint8array	debugdata;
});

PACKSTRUCT(struct dumo_msg_test_device_under_test_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_rx_test_cmd_t
{
	uint8	channel;
});

PACKSTRUCT(struct dumo_msg_test_rx_test_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_packet_test_cmd_t
{
	uint8	mode;
	uint8	tx_freq;
	uint8	rx_freq;
	uint8	acl_type;
	uint16	acl_len;
	uint8	power;
	uint8	disable_whitening;
});

PACKSTRUCT(struct dumo_msg_test_packet_test_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_test_debug_counter_cmd_t
{
	uint32	id;
});

PACKSTRUCT(struct dumo_msg_test_debug_counter_rsp_t
{
	uint16	result;
	uint32	value;
});

PACKSTRUCT(struct dumo_msg_test_dtm_completed_evt_t
{
	uint16	result;
	uint16	number_of_packets;
});

PACKSTRUCT(struct dumo_msg_test_debug_evt_t
{
	uint16	id;
	uint16	u32;
	uint8array	stuff;
});

PACKSTRUCT(struct dumo_msg_test_hcidump_evt_t
{
	uint32	time;
	uint8	direction;
	uint8	packet_type;
	uint8	partial;
	uint16array	data;
});

PACKSTRUCT(struct dumo_msg_test_error_evt_t
{
	uint16	param1;
	uint8array	param2;
});

PACKSTRUCT(struct dumo_msg_sm_set_bondable_mode_cmd_t
{
	uint8	bondable;
});

PACKSTRUCT(struct dumo_msg_sm_set_bondable_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_configure_cmd_t
{
	uint8	mitm_required;
	uint8	io_capabilities;
});

PACKSTRUCT(struct dumo_msg_sm_configure_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_store_bonding_configuration_cmd_t
{
	uint8	max_bonding_count;
	uint8	policy_flags;
});

PACKSTRUCT(struct dumo_msg_sm_store_bonding_configuration_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_read_bonding_configuration_rsp_t
{
	uint8	max_bonding_count;
	uint8	policy_flags;
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_increase_security_cmd_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_sm_increase_security_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_read_bonding_cmd_t
{
	uint8	bonding;
});

PACKSTRUCT(struct dumo_msg_sm_read_bonding_rsp_t
{
	uint16	result;
	bd_addr	address;
	uint8	address_type;
	uint8array	bonding_key;
});

PACKSTRUCT(struct dumo_msg_sm_delete_bonding_cmd_t
{
	uint8	bonding;
});

PACKSTRUCT(struct dumo_msg_sm_delete_bonding_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_delete_bondings_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_enter_passkey_cmd_t
{
	uint8	connection;
	uint32	passkey;
});

PACKSTRUCT(struct dumo_msg_sm_enter_passkey_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_passkey_confirm_cmd_t
{
	uint8	connection;
	uint8	confirm;
});

PACKSTRUCT(struct dumo_msg_sm_passkey_confirm_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_set_oob_data_cmd_t
{
	uint8array	oob_data;
});

PACKSTRUCT(struct dumo_msg_sm_set_oob_data_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_list_all_bondings_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_confirm_bonding_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_enter_pin_code_cmd_t
{
	bd_addr	address;
	uint8array	pin_code;
});

PACKSTRUCT(struct dumo_msg_sm_enter_pin_code_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_set_le_bondable_mode_cmd_t
{
	uint8	bondable;
});

PACKSTRUCT(struct dumo_msg_sm_set_le_bondable_mode_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_set_le_key_enc_only_cmd_t
{
	uint8	enabled;
});

PACKSTRUCT(struct dumo_msg_sm_set_le_key_enc_only_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_sm_set_minimum_key_size_cmd_t
{
	uint8	minimum_key_size;
});

PACKSTRUCT(struct dumo_msg_sm_set_minimum_key_size_rsp_t
{
	uint16	result;
	uint8	minimum_key_size;
});

PACKSTRUCT(struct dumo_msg_sm_passkey_display_evt_t
{
	uint8	connection;
	uint32	passkey;
});

PACKSTRUCT(struct dumo_msg_sm_passkey_request_evt_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_sm_confirm_passkey_evt_t
{
	uint8	connection;
	uint32	passkey;
});

PACKSTRUCT(struct dumo_msg_sm_bonded_evt_t
{
	uint8	connection;
	uint8	bonding;
});

PACKSTRUCT(struct dumo_msg_sm_bonding_failed_evt_t
{
	uint8	connection;
	uint16	reason;
});

PACKSTRUCT(struct dumo_msg_sm_list_bonding_entry_evt_t
{
	uint8	bonding;
	bd_addr	address;
	uint8	address_type;
});

PACKSTRUCT(struct dumo_msg_sm_bonding_request_evt_t
{
	uint8	connection;
});

PACKSTRUCT(struct dumo_msg_sm_pin_code_request_evt_t
{
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_iap_open_cmd_t
{
	bd_addr	address;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_iap_open_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_iap_start_server_cmd_t
{
	uint8	sdp_id;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_iap_start_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_set_param_cmd_t
{
	uint8	index;
	uint32	flags;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_iap_set_param_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_set_protocol_cmd_t
{
	uint8	index;
	uint8	match_action;
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_iap_set_protocol_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_get_param_cmd_t
{
	uint8	index;
});

PACKSTRUCT(struct dumo_msg_iap_get_param_rsp_t
{
	uint16	result;
	uint32	flags;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_iap_get_protocol_cmd_t
{
	uint8	index;
});

PACKSTRUCT(struct dumo_msg_iap_get_protocol_rsp_t
{
	uint16	result;
	uint8	match_action;
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_iap_dump_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_validate_rsp_t
{
	uint16	result;
	uint32	reason;
});

PACKSTRUCT(struct dumo_msg_iap_erase_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_launch_app_cmd_t
{
	uint8	endpoint;
	uint8	launch_alert;
	uint8array	bundle_id;
});

PACKSTRUCT(struct dumo_msg_iap_launch_app_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_stop_server_cmd_t
{
	uint8	sdp_id;
});

PACKSTRUCT(struct dumo_msg_iap_stop_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_iap_param_evt_t
{
	uint8	index;
	uint32	flags;
	uint8array	value;
});

PACKSTRUCT(struct dumo_msg_iap_protocol_evt_t
{
	uint8	index;
	uint8	match_action;
	uint8array	name;
});

PACKSTRUCT(struct dumo_msg_iap_opened_evt_t
{
	uint8	endpoint;
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_iap_auth_ok_evt_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_iap_auth_fail_evt_t
{
	uint8	endpoint;
	uint32	reason;
});

PACKSTRUCT(struct dumo_msg_iap_started_evt_t
{
	uint8	endpoint;
	uint8	protocol;
});

PACKSTRUCT(struct dumo_msg_iap_stopped_evt_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_iap_resume_evt_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_iap_link_params_evt_t
{
	uint8	endpoint;
	uint16	retx_time;
	uint8	max_retx;
});

PACKSTRUCT(struct dumo_msg_util_atoi_cmd_t
{
	uint8array	string;
});

PACKSTRUCT(struct dumo_msg_util_atoi_rsp_t
{
	int32	value;
});

PACKSTRUCT(struct dumo_msg_util_itoa_cmd_t
{
	int32	value;
});

PACKSTRUCT(struct dumo_msg_util_itoa_rsp_t
{
	uint8array	string;
});

PACKSTRUCT(struct dumo_msg_util_debug_print_cmd_t
{
	uint8array	string;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_open_cmd_t
{
	bd_addr	address;
	uint8	streaming_destination;
	uint16	psm;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_open_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_start_server_cmd_t
{
	uint16	psm;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_start_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_opened_evt_t
{
	uint8	endpoint;
	bd_addr	address;
	uint16	psm;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_failed_evt_t
{
	uint8	endpoint;
	bd_addr	address;
	uint16	reason;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_closed_evt_t
{
	uint8	endpoint;
	bd_addr	address;
	uint16	reason;
});

PACKSTRUCT(struct dumo_msg_bt_l2cap_connectionless_data_evt_t
{
	bd_addr	address;
	uint8array	data;
});

PACKSTRUCT(struct dumo_msg_bt_hid_open_cmd_t
{
	bd_addr	address;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_bt_hid_open_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_hid_start_server_cmd_t
{
	uint8	sdp_id;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_bt_hid_start_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_stop_server_cmd_t
{
	uint8	sdp_id;
});

PACKSTRUCT(struct dumo_msg_bt_hid_stop_server_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_send_input_report_cmd_t
{
	uint8	endpoint;
	uint8	report_id;
	uint8array	report_data;
});

PACKSTRUCT(struct dumo_msg_bt_hid_send_input_report_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_virtual_cable_unplug_cmd_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_hid_virtual_cable_unplug_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_get_report_response_cmd_t
{
	uint8	endpoint;
	uint8	parameters_used;
	uint8	report_type;
	uint8	report_id;
	uint8array	report_data;
});

PACKSTRUCT(struct dumo_msg_bt_hid_get_report_response_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_set_report_response_cmd_t
{
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_bt_hid_set_report_response_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_get_set_report_reject_cmd_t
{
	uint8	endpoint;
	uint8	reason;
});

PACKSTRUCT(struct dumo_msg_bt_hid_get_set_report_reject_rsp_t
{
	uint16	result;
});

PACKSTRUCT(struct dumo_msg_bt_hid_opened_evt_t
{
	uint8	endpoint;
	bd_addr	address;
});

PACKSTRUCT(struct dumo_msg_bt_hid_failed_evt_t
{
	uint8	endpoint;
	bd_addr	address;
	uint16	reason;
});

PACKSTRUCT(struct dumo_msg_bt_hid_output_report_evt_t
{
	uint8	endpoint;
	uint8	report_id;
	uint8array	report_data;
});

PACKSTRUCT(struct dumo_msg_bt_hid_state_changed_evt_t
{
	uint8	endpoint;
	uint8	state;
});

PACKSTRUCT(struct dumo_msg_bt_hid_get_report_evt_t
{
	uint8	endpoint;
	uint8	parameters_used;
	uint8	report_type;
	uint8	report_id;
	uint16	max_bytes;
});

PACKSTRUCT(struct dumo_msg_bt_hid_set_report_evt_t
{
	uint8	endpoint;
	uint8	report_type;
	uint8	report_id;
	uint8array	report_data;
});

PACKSTRUCT(struct dumo_msg_le_serial_open_cmd_t
{
	bd_addr	address;
	uint8	address_type;
	uint8	streaming_destination;
	uint8array	service;
});

PACKSTRUCT(struct dumo_msg_le_serial_open_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_le_serial_listen_cmd_t
{
	uint16	characteristic;
	uint8	streaming_destination;
});

PACKSTRUCT(struct dumo_msg_le_serial_listen_rsp_t
{
	uint16	result;
	uint8	endpoint;
});

PACKSTRUCT(struct dumo_msg_le_serial_opened_evt_t
{
	uint8	endpoint;
});

PACKSTRUCT(
struct dumo_cmd_packet
{
	uint32   header;

union{
	uint8 handle;

	uint8 payload;

	struct dumo_msg_dfu_reset_cmd_t                                        cmd_dfu_reset;
	struct dumo_msg_dfu_flash_set_address_cmd_t                            cmd_dfu_flash_set_address;
	struct dumo_msg_dfu_flash_upload_cmd_t                                 cmd_dfu_flash_upload;
	struct dumo_msg_dfu_boot_evt_t                                         evt_dfu_boot;
	struct dumo_msg_dfu_flash_set_address_rsp_t                            rsp_dfu_flash_set_address;
	struct dumo_msg_dfu_flash_upload_rsp_t                                 rsp_dfu_flash_upload;
	struct dumo_msg_dfu_flash_upload_finish_rsp_t                          rsp_dfu_flash_upload_finish;
	struct dumo_msg_system_reset_cmd_t                                     cmd_system_reset;
	struct dumo_msg_system_set_max_power_mode_cmd_t                        cmd_system_set_max_power_mode;
	struct dumo_msg_system_set_bt_address_cmd_t                            cmd_system_set_bt_address;
	struct dumo_msg_system_set_class_of_device_cmd_t                       cmd_system_set_class_of_device;
	struct dumo_msg_system_set_local_name_cmd_t                            cmd_system_set_local_name;
	struct dumo_msg_system_write_bt_config_cmd_t                           cmd_system_write_bt_config;
	struct dumo_msg_system_boot_evt_t                                      evt_system_boot;
	struct dumo_msg_system_initialized_evt_t                               evt_system_initialized;
	struct dumo_msg_system_recovery_evt_t                                  evt_system_recovery;
	struct dumo_msg_system_script_stopped_evt_t                            evt_system_script_stopped;
	struct dumo_msg_system_hello_rsp_t                                     rsp_system_hello;
	struct dumo_msg_system_set_max_power_mode_rsp_t                        rsp_system_set_max_power_mode;
	struct dumo_msg_system_get_bt_address_rsp_t                            rsp_system_get_bt_address;
	struct dumo_msg_system_set_bt_address_rsp_t                            rsp_system_set_bt_address;
	struct dumo_msg_system_get_class_of_device_rsp_t                       rsp_system_get_class_of_device;
	struct dumo_msg_system_set_class_of_device_rsp_t                       rsp_system_set_class_of_device;
	struct dumo_msg_system_reset_factory_settings_rsp_t                    rsp_system_reset_factory_settings;
	struct dumo_msg_system_set_local_name_rsp_t                            rsp_system_set_local_name;
	struct dumo_msg_system_get_local_name_rsp_t                            rsp_system_get_local_name;
	struct dumo_msg_system_write_bt_config_rsp_t                           rsp_system_write_bt_config;
	struct dumo_msg_bt_gap_discover_cmd_t                                  cmd_bt_gap_discover;
	struct dumo_msg_bt_gap_get_remote_name_cmd_t                           cmd_bt_gap_get_remote_name;
	struct dumo_msg_bt_gap_set_mode_cmd_t                                  cmd_bt_gap_set_mode;
	struct dumo_msg_bt_gap_set_policy_cmd_t                                cmd_bt_gap_set_policy;
	struct dumo_msg_bt_gap_set_parameters_cmd_t                            cmd_bt_gap_set_parameters;
	struct dumo_msg_bt_gap_set_max_power_cmd_t                             cmd_bt_gap_set_max_power;
	struct dumo_msg_bt_gap_open_cmd_t                                      cmd_bt_gap_open;
	struct dumo_msg_bt_gap_set_auto_sniff_cmd_t                            cmd_bt_gap_set_auto_sniff;
	struct dumo_msg_bt_gap_set_discovery_mode_cmd_t                        cmd_bt_gap_set_discovery_mode;
	struct dumo_msg_bt_gap_set_host_channel_classification_cmd_t           cmd_bt_gap_set_host_channel_classification;
	struct dumo_msg_bt_gap_discovery_result_evt_t                          evt_bt_gap_discovery_result;
	struct dumo_msg_bt_gap_discovery_complete_evt_t                        evt_bt_gap_discovery_complete;
	struct dumo_msg_bt_gap_remote_name_evt_t                               evt_bt_gap_remote_name;
	struct dumo_msg_bt_gap_host_channel_classification_complete_evt_t      evt_bt_gap_host_channel_classification_complete;
	struct dumo_msg_bt_gap_discover_rsp_t                                  rsp_bt_gap_discover;
	struct dumo_msg_bt_gap_cancel_discovery_rsp_t                          rsp_bt_gap_cancel_discovery;
	struct dumo_msg_bt_gap_get_remote_name_rsp_t                           rsp_bt_gap_get_remote_name;
	struct dumo_msg_bt_gap_set_mode_rsp_t                                  rsp_bt_gap_set_mode;
	struct dumo_msg_bt_gap_get_mode_rsp_t                                  rsp_bt_gap_get_mode;
	struct dumo_msg_bt_gap_set_policy_rsp_t                                rsp_bt_gap_set_policy;
	struct dumo_msg_bt_gap_set_parameters_rsp_t                            rsp_bt_gap_set_parameters;
	struct dumo_msg_bt_gap_set_max_power_rsp_t                             rsp_bt_gap_set_max_power;
	struct dumo_msg_bt_gap_open_rsp_t                                      rsp_bt_gap_open;
	struct dumo_msg_bt_gap_set_auto_sniff_rsp_t                            rsp_bt_gap_set_auto_sniff;
	struct dumo_msg_bt_gap_set_discovery_mode_rsp_t                        rsp_bt_gap_set_discovery_mode;
	struct dumo_msg_bt_gap_set_host_channel_classification_rsp_t           rsp_bt_gap_set_host_channel_classification;
	struct dumo_msg_le_gap_open_cmd_t                                      cmd_le_gap_open;
	struct dumo_msg_le_gap_set_mode_cmd_t                                  cmd_le_gap_set_mode;
	struct dumo_msg_le_gap_discover_cmd_t                                  cmd_le_gap_discover;
	struct dumo_msg_le_gap_set_adv_parameters_cmd_t                        cmd_le_gap_set_adv_parameters;
	struct dumo_msg_le_gap_set_conn_parameters_cmd_t                       cmd_le_gap_set_conn_parameters;
	struct dumo_msg_le_gap_set_scan_parameters_cmd_t                       cmd_le_gap_set_scan_parameters;
	struct dumo_msg_le_gap_set_adv_data_cmd_t                              cmd_le_gap_set_adv_data;
	struct dumo_msg_le_gap_set_max_power_cmd_t                             cmd_le_gap_set_max_power;
	struct dumo_msg_le_gap_set_host_channel_classification_cmd_t           cmd_le_gap_set_host_channel_classification;
	struct dumo_msg_le_gap_set_scan_result_filter_cmd_t                    cmd_le_gap_set_scan_result_filter;
	struct dumo_msg_le_gap_set_random_static_address_cmd_t                 cmd_le_gap_set_random_static_address;
	struct dumo_msg_le_gap_scan_response_evt_t                             evt_le_gap_scan_response;
	struct dumo_msg_le_gap_host_channel_classification_complete_evt_t      evt_le_gap_host_channel_classification_complete;
	struct dumo_msg_le_gap_open_rsp_t                                      rsp_le_gap_open;
	struct dumo_msg_le_gap_set_mode_rsp_t                                  rsp_le_gap_set_mode;
	struct dumo_msg_le_gap_discover_rsp_t                                  rsp_le_gap_discover;
	struct dumo_msg_le_gap_end_procedure_rsp_t                             rsp_le_gap_end_procedure;
	struct dumo_msg_le_gap_set_adv_parameters_rsp_t                        rsp_le_gap_set_adv_parameters;
	struct dumo_msg_le_gap_set_conn_parameters_rsp_t                       rsp_le_gap_set_conn_parameters;
	struct dumo_msg_le_gap_set_scan_parameters_rsp_t                       rsp_le_gap_set_scan_parameters;
	struct dumo_msg_le_gap_set_adv_data_rsp_t                              rsp_le_gap_set_adv_data;
	struct dumo_msg_le_gap_set_max_power_rsp_t                             rsp_le_gap_set_max_power;
	struct dumo_msg_le_gap_set_host_channel_classification_rsp_t           rsp_le_gap_set_host_channel_classification;
	struct dumo_msg_le_gap_set_scan_result_filter_rsp_t                    rsp_le_gap_set_scan_result_filter;
	struct dumo_msg_le_gap_scan_filter_clear_rsp_t                         rsp_le_gap_scan_filter_clear;
	struct dumo_msg_le_gap_set_random_static_address_rsp_t                 rsp_le_gap_set_random_static_address;
	struct dumo_msg_bt_rfcomm_open_cmd_t                                   cmd_bt_rfcomm_open;
	struct dumo_msg_bt_rfcomm_start_server_cmd_t                           cmd_bt_rfcomm_start_server;
	struct dumo_msg_bt_rfcomm_open_port_cmd_t                              cmd_bt_rfcomm_open_port;
	struct dumo_msg_bt_rfcomm_stop_server_cmd_t                            cmd_bt_rfcomm_stop_server;
	struct dumo_msg_bt_rfcomm_modem_status_cmd_t                           cmd_bt_rfcomm_modem_status;
	struct dumo_msg_bt_rfcomm_start_server_port_cmd_t                      cmd_bt_rfcomm_start_server_port;
	struct dumo_msg_bt_rfcomm_stop_server_port_cmd_t                       cmd_bt_rfcomm_stop_server_port;
	struct dumo_msg_bt_rfcomm_set_modem_status_default_cmd_t               cmd_bt_rfcomm_set_modem_status_default;
	struct dumo_msg_bt_rfcomm_opened_evt_t                                 evt_bt_rfcomm_opened;
	struct dumo_msg_bt_rfcomm_modem_status_evt_t                           evt_bt_rfcomm_modem_status;
	struct dumo_msg_bt_rfcomm_incoming_rejected_evt_t                      evt_bt_rfcomm_incoming_rejected;
	struct dumo_msg_bt_rfcomm_credit_starvation_evt_t                      evt_bt_rfcomm_credit_starvation;
	struct dumo_msg_bt_rfcomm_open_rsp_t                                   rsp_bt_rfcomm_open;
	struct dumo_msg_bt_rfcomm_start_server_rsp_t                           rsp_bt_rfcomm_start_server;
	struct dumo_msg_bt_rfcomm_open_port_rsp_t                              rsp_bt_rfcomm_open_port;
	struct dumo_msg_bt_rfcomm_stop_server_rsp_t                            rsp_bt_rfcomm_stop_server;
	struct dumo_msg_bt_rfcomm_modem_status_rsp_t                           rsp_bt_rfcomm_modem_status;
	struct dumo_msg_bt_rfcomm_start_server_port_rsp_t                      rsp_bt_rfcomm_start_server_port;
	struct dumo_msg_bt_rfcomm_stop_server_port_rsp_t                       rsp_bt_rfcomm_stop_server_port;
	struct dumo_msg_bt_rfcomm_set_modem_status_default_rsp_t               rsp_bt_rfcomm_set_modem_status_default;
	struct dumo_msg_identity_remote_identity_cmd_t                         cmd_identity_remote_identity;
	struct dumo_msg_identity_modify_local_identity_cmd_t                   cmd_identity_modify_local_identity;
	struct dumo_msg_identity_remote_identity_evt_t                         evt_identity_remote_identity;
	struct dumo_msg_identity_remote_identity_rsp_t                         rsp_identity_remote_identity;
	struct dumo_msg_identity_local_identity_rsp_t                          rsp_identity_local_identity;
	struct dumo_msg_identity_modify_local_identity_rsp_t                   rsp_identity_modify_local_identity;
	struct dumo_msg_bt_sdp_search_rfcomm_port_cmd_t                        cmd_bt_sdp_search_rfcomm_port;
	struct dumo_msg_bt_sdp_load_entry_cmd_t                                cmd_bt_sdp_load_entry;
	struct dumo_msg_bt_sdp_add_entry_cmd_t                                 cmd_bt_sdp_add_entry;
	struct dumo_msg_bt_sdp_delete_entry_cmd_t                              cmd_bt_sdp_delete_entry;
	struct dumo_msg_bt_sdp_search_rfcomm_port_evt_t                        evt_bt_sdp_search_rfcomm_port;
	struct dumo_msg_bt_sdp_search_rfcomm_port_rsp_t                        rsp_bt_sdp_search_rfcomm_port;
	struct dumo_msg_bt_sdp_load_entry_rsp_t                                rsp_bt_sdp_load_entry;
	struct dumo_msg_bt_sdp_add_entry_rsp_t                                 rsp_bt_sdp_add_entry;
	struct dumo_msg_bt_sdp_delete_entry_rsp_t                              rsp_bt_sdp_delete_entry;
	struct dumo_msg_bt_connection_set_active_cmd_t                         cmd_bt_connection_set_active;
	struct dumo_msg_bt_connection_set_sniff_cmd_t                          cmd_bt_connection_set_sniff;
	struct dumo_msg_bt_connection_set_role_cmd_t                           cmd_bt_connection_set_role;
	struct dumo_msg_bt_connection_set_supervision_timeout_cmd_t            cmd_bt_connection_set_supervision_timeout;
	struct dumo_msg_bt_connection_get_rssi_cmd_t                           cmd_bt_connection_get_rssi;
	struct dumo_msg_bt_connection_read_clock_cmd_t                         cmd_bt_connection_read_clock;
	struct dumo_msg_bt_connection_opened_evt_t                             evt_bt_connection_opened;
	struct dumo_msg_bt_connection_closed_evt_t                             evt_bt_connection_closed;
	struct dumo_msg_bt_connection_parameters_evt_t                         evt_bt_connection_parameters;
	struct dumo_msg_bt_connection_rssi_value_evt_t                         evt_bt_connection_rssi_value;
	struct dumo_msg_bt_connection_clock_value_evt_t                        evt_bt_connection_clock_value;
	struct dumo_msg_bt_connection_list_rsp_t                               rsp_bt_connection_list;
	struct dumo_msg_bt_connection_set_active_rsp_t                         rsp_bt_connection_set_active;
	struct dumo_msg_bt_connection_set_sniff_rsp_t                          rsp_bt_connection_set_sniff;
	struct dumo_msg_bt_connection_set_role_rsp_t                           rsp_bt_connection_set_role;
	struct dumo_msg_bt_connection_set_supervision_timeout_rsp_t            rsp_bt_connection_set_supervision_timeout;
	struct dumo_msg_bt_connection_get_rssi_rsp_t                           rsp_bt_connection_get_rssi;
	struct dumo_msg_bt_connection_read_clock_rsp_t                         rsp_bt_connection_read_clock;
	struct dumo_msg_le_connection_set_parameters_cmd_t                     cmd_le_connection_set_parameters;
	struct dumo_msg_le_connection_opened_evt_t                             evt_le_connection_opened;
	struct dumo_msg_le_connection_closed_evt_t                             evt_le_connection_closed;
	struct dumo_msg_le_connection_parameters_evt_t                         evt_le_connection_parameters;
	struct dumo_msg_le_connection_set_parameters_rsp_t                     rsp_le_connection_set_parameters;
	struct dumo_msg_le_connection_list_rsp_t                               rsp_le_connection_list;
	struct dumo_msg_gatt_set_max_mtu_cmd_t                                 cmd_gatt_set_max_mtu;
	struct dumo_msg_gatt_discover_primary_services_cmd_t                   cmd_gatt_discover_primary_services;
	struct dumo_msg_gatt_discover_primary_services_by_uuid_cmd_t           cmd_gatt_discover_primary_services_by_uuid;
	struct dumo_msg_gatt_discover_characteristics_cmd_t                    cmd_gatt_discover_characteristics;
	struct dumo_msg_gatt_discover_characteristics_by_uuid_cmd_t            cmd_gatt_discover_characteristics_by_uuid;
	struct dumo_msg_gatt_set_characteristic_notification_cmd_t             cmd_gatt_set_characteristic_notification;
	struct dumo_msg_gatt_discover_descriptors_cmd_t                        cmd_gatt_discover_descriptors;
	struct dumo_msg_gatt_read_characteristic_value_cmd_t                   cmd_gatt_read_characteristic_value;
	struct dumo_msg_gatt_read_characteristic_value_by_uuid_cmd_t           cmd_gatt_read_characteristic_value_by_uuid;
	struct dumo_msg_gatt_write_characteristic_value_cmd_t                  cmd_gatt_write_characteristic_value;
	struct dumo_msg_gatt_write_characteristic_value_without_response_cmd_t cmd_gatt_write_characteristic_value_without_response;
	struct dumo_msg_gatt_prepare_characteristic_value_write_cmd_t          cmd_gatt_prepare_characteristic_value_write;
	struct dumo_msg_gatt_execute_characteristic_value_write_cmd_t          cmd_gatt_execute_characteristic_value_write;
	struct dumo_msg_gatt_send_characteristic_confirmation_cmd_t            cmd_gatt_send_characteristic_confirmation;
	struct dumo_msg_gatt_read_descriptor_value_cmd_t                       cmd_gatt_read_descriptor_value;
	struct dumo_msg_gatt_write_descriptor_value_cmd_t                      cmd_gatt_write_descriptor_value;
	struct dumo_msg_gatt_find_included_services_cmd_t                      cmd_gatt_find_included_services;
	struct dumo_msg_gatt_read_multiple_characteristic_values_cmd_t         cmd_gatt_read_multiple_characteristic_values;
	struct dumo_msg_gatt_find_information_request_cmd_t                    cmd_gatt_find_information_request;
	struct dumo_msg_gatt_mtu_exchanged_evt_t                               evt_gatt_mtu_exchanged;
	struct dumo_msg_gatt_service_evt_t                                     evt_gatt_service;
	struct dumo_msg_gatt_characteristic_evt_t                              evt_gatt_characteristic;
	struct dumo_msg_gatt_descriptor_evt_t                                  evt_gatt_descriptor;
	struct dumo_msg_gatt_characteristic_value_evt_t                        evt_gatt_characteristic_value;
	struct dumo_msg_gatt_descriptor_value_evt_t                            evt_gatt_descriptor_value;
	struct dumo_msg_gatt_procedure_completed_evt_t                         evt_gatt_procedure_completed;
	struct dumo_msg_gatt_find_information_found_evt_t                      evt_gatt_find_information_found;
	struct dumo_msg_gatt_set_max_mtu_rsp_t                                 rsp_gatt_set_max_mtu;
	struct dumo_msg_gatt_discover_primary_services_rsp_t                   rsp_gatt_discover_primary_services;
	struct dumo_msg_gatt_discover_primary_services_by_uuid_rsp_t           rsp_gatt_discover_primary_services_by_uuid;
	struct dumo_msg_gatt_discover_characteristics_rsp_t                    rsp_gatt_discover_characteristics;
	struct dumo_msg_gatt_discover_characteristics_by_uuid_rsp_t            rsp_gatt_discover_characteristics_by_uuid;
	struct dumo_msg_gatt_set_characteristic_notification_rsp_t             rsp_gatt_set_characteristic_notification;
	struct dumo_msg_gatt_discover_descriptors_rsp_t                        rsp_gatt_discover_descriptors;
	struct dumo_msg_gatt_read_characteristic_value_rsp_t                   rsp_gatt_read_characteristic_value;
	struct dumo_msg_gatt_read_characteristic_value_by_uuid_rsp_t           rsp_gatt_read_characteristic_value_by_uuid;
	struct dumo_msg_gatt_write_characteristic_value_rsp_t                  rsp_gatt_write_characteristic_value;
	struct dumo_msg_gatt_write_characteristic_value_without_response_rsp_t rsp_gatt_write_characteristic_value_without_response;
	struct dumo_msg_gatt_prepare_characteristic_value_write_rsp_t          rsp_gatt_prepare_characteristic_value_write;
	struct dumo_msg_gatt_execute_characteristic_value_write_rsp_t          rsp_gatt_execute_characteristic_value_write;
	struct dumo_msg_gatt_send_characteristic_confirmation_rsp_t            rsp_gatt_send_characteristic_confirmation;
	struct dumo_msg_gatt_read_descriptor_value_rsp_t                       rsp_gatt_read_descriptor_value;
	struct dumo_msg_gatt_write_descriptor_value_rsp_t                      rsp_gatt_write_descriptor_value;
	struct dumo_msg_gatt_find_included_services_rsp_t                      rsp_gatt_find_included_services;
	struct dumo_msg_gatt_read_multiple_characteristic_values_rsp_t         rsp_gatt_read_multiple_characteristic_values;
	struct dumo_msg_gatt_find_information_request_rsp_t                    rsp_gatt_find_information_request;
	struct dumo_msg_gatt_server_read_attribute_value_cmd_t                 cmd_gatt_server_read_attribute_value;
	struct dumo_msg_gatt_server_read_attribute_type_cmd_t                  cmd_gatt_server_read_attribute_type;
	struct dumo_msg_gatt_server_write_attribute_value_cmd_t                cmd_gatt_server_write_attribute_value;
	struct dumo_msg_gatt_server_send_user_read_response_cmd_t              cmd_gatt_server_send_user_read_response;
	struct dumo_msg_gatt_server_send_user_write_response_cmd_t             cmd_gatt_server_send_user_write_response;
	struct dumo_msg_gatt_server_send_characteristic_notification_cmd_t     cmd_gatt_server_send_characteristic_notification;
	struct dumo_msg_gatt_server_set_service_status_cmd_t                   cmd_gatt_server_set_service_status;
	struct dumo_msg_gatt_server_attribute_value_evt_t                      evt_gatt_server_attribute_value;
	struct dumo_msg_gatt_server_user_read_request_evt_t                    evt_gatt_server_user_read_request;
	struct dumo_msg_gatt_server_user_write_request_evt_t                   evt_gatt_server_user_write_request;
	struct dumo_msg_gatt_server_characteristic_status_evt_t                evt_gatt_server_characteristic_status;
	struct dumo_msg_gatt_server_read_attribute_value_rsp_t                 rsp_gatt_server_read_attribute_value;
	struct dumo_msg_gatt_server_read_attribute_type_rsp_t                  rsp_gatt_server_read_attribute_type;
	struct dumo_msg_gatt_server_write_attribute_value_rsp_t                rsp_gatt_server_write_attribute_value;
	struct dumo_msg_gatt_server_send_user_read_response_rsp_t              rsp_gatt_server_send_user_read_response;
	struct dumo_msg_gatt_server_send_user_write_response_rsp_t             rsp_gatt_server_send_user_write_response;
	struct dumo_msg_gatt_server_send_characteristic_notification_rsp_t     rsp_gatt_server_send_characteristic_notification;
	struct dumo_msg_gatt_server_set_service_status_rsp_t                   rsp_gatt_server_set_service_status;
	struct dumo_msg_endpoint_send_cmd_t                                    cmd_endpoint_send;
	struct dumo_msg_endpoint_set_streaming_destination_cmd_t               cmd_endpoint_set_streaming_destination;
	struct dumo_msg_endpoint_close_cmd_t                                   cmd_endpoint_close;
	struct dumo_msg_endpoint_set_flags_cmd_t                               cmd_endpoint_set_flags;
	struct dumo_msg_endpoint_clr_flags_cmd_t                               cmd_endpoint_clr_flags;
	struct dumo_msg_endpoint_read_counters_cmd_t                           cmd_endpoint_read_counters;
	struct dumo_msg_endpoint_set_active_cmd_t                              cmd_endpoint_set_active;
	struct dumo_msg_endpoint_syntax_error_evt_t                            evt_endpoint_syntax_error;
	struct dumo_msg_endpoint_data_evt_t                                    evt_endpoint_data;
	struct dumo_msg_endpoint_status_evt_t                                  evt_endpoint_status;
	struct dumo_msg_endpoint_closing_evt_t                                 evt_endpoint_closing;
	struct dumo_msg_endpoint_closed_evt_t                                  evt_endpoint_closed;
	struct dumo_msg_endpoint_send_rsp_t                                    rsp_endpoint_send;
	struct dumo_msg_endpoint_set_streaming_destination_rsp_t               rsp_endpoint_set_streaming_destination;
	struct dumo_msg_endpoint_close_rsp_t                                   rsp_endpoint_close;
	struct dumo_msg_endpoint_set_flags_rsp_t                               rsp_endpoint_set_flags;
	struct dumo_msg_endpoint_clr_flags_rsp_t                               rsp_endpoint_clr_flags;
	struct dumo_msg_endpoint_read_counters_rsp_t                           rsp_endpoint_read_counters;
	struct dumo_msg_endpoint_set_active_rsp_t                              rsp_endpoint_set_active;
	struct dumo_msg_hardware_set_soft_timer_cmd_t                          cmd_hardware_set_soft_timer;
	struct dumo_msg_hardware_configure_gpio_cmd_t                          cmd_hardware_configure_gpio;
	struct dumo_msg_hardware_write_gpio_cmd_t                              cmd_hardware_write_gpio;
	struct dumo_msg_hardware_read_gpio_cmd_t                               cmd_hardware_read_gpio;
	struct dumo_msg_hardware_read_adc_cmd_t                                cmd_hardware_read_adc;
	struct dumo_msg_hardware_read_i2c_cmd_t                                cmd_hardware_read_i2c;
	struct dumo_msg_hardware_write_i2c_cmd_t                               cmd_hardware_write_i2c;
	struct dumo_msg_hardware_stop_i2c_cmd_t                                cmd_hardware_stop_i2c;
	struct dumo_msg_hardware_set_uart_configuration_cmd_t                  cmd_hardware_set_uart_configuration;
	struct dumo_msg_hardware_read_write_spi_cmd_t                          cmd_hardware_read_write_spi;
	struct dumo_msg_hardware_set_uart_bgapi_mode_cmd_t                     cmd_hardware_set_uart_bgapi_mode;
	struct dumo_msg_hardware_soft_timer_evt_t                              evt_hardware_soft_timer;
	struct dumo_msg_hardware_interrupt_evt_t                               evt_hardware_interrupt;
	struct dumo_msg_hardware_set_soft_timer_rsp_t                          rsp_hardware_set_soft_timer;
	struct dumo_msg_hardware_configure_gpio_rsp_t                          rsp_hardware_configure_gpio;
	struct dumo_msg_hardware_write_gpio_rsp_t                              rsp_hardware_write_gpio;
	struct dumo_msg_hardware_read_gpio_rsp_t                               rsp_hardware_read_gpio;
	struct dumo_msg_hardware_read_adc_rsp_t                                rsp_hardware_read_adc;
	struct dumo_msg_hardware_read_i2c_rsp_t                                rsp_hardware_read_i2c;
	struct dumo_msg_hardware_write_i2c_rsp_t                               rsp_hardware_write_i2c;
	struct dumo_msg_hardware_stop_i2c_rsp_t                                rsp_hardware_stop_i2c;
	struct dumo_msg_hardware_set_uart_configuration_rsp_t                  rsp_hardware_set_uart_configuration;
	struct dumo_msg_hardware_read_vdd_rsp_t                                rsp_hardware_read_vdd;
	struct dumo_msg_hardware_read_junction_temperature_rsp_t               rsp_hardware_read_junction_temperature;
	struct dumo_msg_hardware_read_write_spi_rsp_t                          rsp_hardware_read_write_spi;
	struct dumo_msg_hardware_set_uart_bgapi_mode_rsp_t                     rsp_hardware_set_uart_bgapi_mode;
	struct dumo_msg_flash_ps_save_cmd_t                                    cmd_flash_ps_save;
	struct dumo_msg_flash_ps_load_cmd_t                                    cmd_flash_ps_load;
	struct dumo_msg_flash_ps_erase_cmd_t                                   cmd_flash_ps_erase;
	struct dumo_msg_flash_ps_key_evt_t                                     evt_flash_ps_key;
	struct dumo_msg_flash_ps_dump_rsp_t                                    rsp_flash_ps_dump;
	struct dumo_msg_flash_ps_erase_all_rsp_t                               rsp_flash_ps_erase_all;
	struct dumo_msg_flash_ps_save_rsp_t                                    rsp_flash_ps_save;
	struct dumo_msg_flash_ps_load_rsp_t                                    rsp_flash_ps_load;
	struct dumo_msg_flash_ps_erase_rsp_t                                   rsp_flash_ps_erase;
	struct dumo_msg_test_dtm_tx_cmd_t                                      cmd_test_dtm_tx;
	struct dumo_msg_test_dtm_rx_cmd_t                                      cmd_test_dtm_rx;
	struct dumo_msg_test_tx_test_cmd_t                                     cmd_test_tx_test;
	struct dumo_msg_test_debug_enable_cmd_t                                cmd_test_debug_enable;
	struct dumo_msg_test_ssp_debug_cmd_t                                   cmd_test_ssp_debug;
	struct dumo_msg_test_debug_command_cmd_t                               cmd_test_debug_command;
	struct dumo_msg_test_rx_test_cmd_t                                     cmd_test_rx_test;
	struct dumo_msg_test_packet_test_cmd_t                                 cmd_test_packet_test;
	struct dumo_msg_test_debug_counter_cmd_t                               cmd_test_debug_counter;
	struct dumo_msg_test_dtm_completed_evt_t                               evt_test_dtm_completed;
	struct dumo_msg_test_debug_evt_t                                       evt_test_debug;
	struct dumo_msg_test_hcidump_evt_t                                     evt_test_hcidump;
	struct dumo_msg_test_error_evt_t                                       evt_test_error;
	struct dumo_msg_test_dtm_tx_rsp_t                                      rsp_test_dtm_tx;
	struct dumo_msg_test_dtm_rx_rsp_t                                      rsp_test_dtm_rx;
	struct dumo_msg_test_dtm_end_rsp_t                                     rsp_test_dtm_end;
	struct dumo_msg_test_tx_test_rsp_t                                     rsp_test_tx_test;
	struct dumo_msg_test_debug_command_rsp_t                               rsp_test_debug_command;
	struct dumo_msg_test_device_under_test_mode_rsp_t                      rsp_test_device_under_test_mode;
	struct dumo_msg_test_rx_test_rsp_t                                     rsp_test_rx_test;
	struct dumo_msg_test_packet_test_rsp_t                                 rsp_test_packet_test;
	struct dumo_msg_test_debug_counter_rsp_t                               rsp_test_debug_counter;
	struct dumo_msg_sm_set_bondable_mode_cmd_t                             cmd_sm_set_bondable_mode;
	struct dumo_msg_sm_configure_cmd_t                                     cmd_sm_configure;
	struct dumo_msg_sm_store_bonding_configuration_cmd_t                   cmd_sm_store_bonding_configuration;
	struct dumo_msg_sm_increase_security_cmd_t                             cmd_sm_increase_security;
	struct dumo_msg_sm_read_bonding_cmd_t                                  cmd_sm_read_bonding;
	struct dumo_msg_sm_delete_bonding_cmd_t                                cmd_sm_delete_bonding;
	struct dumo_msg_sm_enter_passkey_cmd_t                                 cmd_sm_enter_passkey;
	struct dumo_msg_sm_passkey_confirm_cmd_t                               cmd_sm_passkey_confirm;
	struct dumo_msg_sm_set_oob_data_cmd_t                                  cmd_sm_set_oob_data;
	struct dumo_msg_sm_enter_pin_code_cmd_t                                cmd_sm_enter_pin_code;
	struct dumo_msg_sm_set_le_bondable_mode_cmd_t                          cmd_sm_set_le_bondable_mode;
	struct dumo_msg_sm_set_le_key_enc_only_cmd_t                           cmd_sm_set_le_key_enc_only;
	struct dumo_msg_sm_set_minimum_key_size_cmd_t                          cmd_sm_set_minimum_key_size;
	struct dumo_msg_sm_passkey_display_evt_t                               evt_sm_passkey_display;
	struct dumo_msg_sm_passkey_request_evt_t                               evt_sm_passkey_request;
	struct dumo_msg_sm_confirm_passkey_evt_t                               evt_sm_confirm_passkey;
	struct dumo_msg_sm_bonded_evt_t                                        evt_sm_bonded;
	struct dumo_msg_sm_bonding_failed_evt_t                                evt_sm_bonding_failed;
	struct dumo_msg_sm_list_bonding_entry_evt_t                            evt_sm_list_bonding_entry;
	struct dumo_msg_sm_bonding_request_evt_t                               evt_sm_bonding_request;
	struct dumo_msg_sm_pin_code_request_evt_t                              evt_sm_pin_code_request;
	struct dumo_msg_sm_set_bondable_mode_rsp_t                             rsp_sm_set_bondable_mode;
	struct dumo_msg_sm_configure_rsp_t                                     rsp_sm_configure;
	struct dumo_msg_sm_store_bonding_configuration_rsp_t                   rsp_sm_store_bonding_configuration;
	struct dumo_msg_sm_read_bonding_configuration_rsp_t                    rsp_sm_read_bonding_configuration;
	struct dumo_msg_sm_increase_security_rsp_t                             rsp_sm_increase_security;
	struct dumo_msg_sm_read_bonding_rsp_t                                  rsp_sm_read_bonding;
	struct dumo_msg_sm_delete_bonding_rsp_t                                rsp_sm_delete_bonding;
	struct dumo_msg_sm_delete_bondings_rsp_t                               rsp_sm_delete_bondings;
	struct dumo_msg_sm_enter_passkey_rsp_t                                 rsp_sm_enter_passkey;
	struct dumo_msg_sm_passkey_confirm_rsp_t                               rsp_sm_passkey_confirm;
	struct dumo_msg_sm_set_oob_data_rsp_t                                  rsp_sm_set_oob_data;
	struct dumo_msg_sm_list_all_bondings_rsp_t                             rsp_sm_list_all_bondings;
	struct dumo_msg_sm_confirm_bonding_rsp_t                               rsp_sm_confirm_bonding;
	struct dumo_msg_sm_enter_pin_code_rsp_t                                rsp_sm_enter_pin_code;
	struct dumo_msg_sm_set_le_bondable_mode_rsp_t                          rsp_sm_set_le_bondable_mode;
	struct dumo_msg_sm_set_le_key_enc_only_rsp_t                           rsp_sm_set_le_key_enc_only;
	struct dumo_msg_sm_set_minimum_key_size_rsp_t                          rsp_sm_set_minimum_key_size;
	struct dumo_msg_iap_open_cmd_t                                         cmd_iap_open;
	struct dumo_msg_iap_start_server_cmd_t                                 cmd_iap_start_server;
	struct dumo_msg_iap_set_param_cmd_t                                    cmd_iap_set_param;
	struct dumo_msg_iap_set_protocol_cmd_t                                 cmd_iap_set_protocol;
	struct dumo_msg_iap_get_param_cmd_t                                    cmd_iap_get_param;
	struct dumo_msg_iap_get_protocol_cmd_t                                 cmd_iap_get_protocol;
	struct dumo_msg_iap_launch_app_cmd_t                                   cmd_iap_launch_app;
	struct dumo_msg_iap_stop_server_cmd_t                                  cmd_iap_stop_server;
	struct dumo_msg_iap_param_evt_t                                        evt_iap_param;
	struct dumo_msg_iap_protocol_evt_t                                     evt_iap_protocol;
	struct dumo_msg_iap_opened_evt_t                                       evt_iap_opened;
	struct dumo_msg_iap_auth_ok_evt_t                                      evt_iap_auth_ok;
	struct dumo_msg_iap_auth_fail_evt_t                                    evt_iap_auth_fail;
	struct dumo_msg_iap_started_evt_t                                      evt_iap_started;
	struct dumo_msg_iap_stopped_evt_t                                      evt_iap_stopped;
	struct dumo_msg_iap_resume_evt_t                                       evt_iap_resume;
	struct dumo_msg_iap_link_params_evt_t                                  evt_iap_link_params;
	struct dumo_msg_iap_open_rsp_t                                         rsp_iap_open;
	struct dumo_msg_iap_start_server_rsp_t                                 rsp_iap_start_server;
	struct dumo_msg_iap_set_param_rsp_t                                    rsp_iap_set_param;
	struct dumo_msg_iap_set_protocol_rsp_t                                 rsp_iap_set_protocol;
	struct dumo_msg_iap_get_param_rsp_t                                    rsp_iap_get_param;
	struct dumo_msg_iap_get_protocol_rsp_t                                 rsp_iap_get_protocol;
	struct dumo_msg_iap_dump_rsp_t                                         rsp_iap_dump;
	struct dumo_msg_iap_validate_rsp_t                                     rsp_iap_validate;
	struct dumo_msg_iap_erase_rsp_t                                        rsp_iap_erase;
	struct dumo_msg_iap_launch_app_rsp_t                                   rsp_iap_launch_app;
	struct dumo_msg_iap_stop_server_rsp_t                                  rsp_iap_stop_server;
	struct dumo_msg_util_atoi_cmd_t                                        cmd_util_atoi;
	struct dumo_msg_util_itoa_cmd_t                                        cmd_util_itoa;
	struct dumo_msg_util_debug_print_cmd_t                                 cmd_util_debug_print;
	struct dumo_msg_util_atoi_rsp_t                                        rsp_util_atoi;
	struct dumo_msg_util_itoa_rsp_t                                        rsp_util_itoa;
	struct dumo_msg_bt_l2cap_open_cmd_t                                    cmd_bt_l2cap_open;
	struct dumo_msg_bt_l2cap_start_server_cmd_t                            cmd_bt_l2cap_start_server;
	struct dumo_msg_bt_l2cap_opened_evt_t                                  evt_bt_l2cap_opened;
	struct dumo_msg_bt_l2cap_failed_evt_t                                  evt_bt_l2cap_failed;
	struct dumo_msg_bt_l2cap_closed_evt_t                                  evt_bt_l2cap_closed;
	struct dumo_msg_bt_l2cap_connectionless_data_evt_t                     evt_bt_l2cap_connectionless_data;
	struct dumo_msg_bt_l2cap_open_rsp_t                                    rsp_bt_l2cap_open;
	struct dumo_msg_bt_l2cap_start_server_rsp_t                            rsp_bt_l2cap_start_server;
	struct dumo_msg_bt_hid_open_cmd_t                                      cmd_bt_hid_open;
	struct dumo_msg_bt_hid_start_server_cmd_t                              cmd_bt_hid_start_server;
	struct dumo_msg_bt_hid_stop_server_cmd_t                               cmd_bt_hid_stop_server;
	struct dumo_msg_bt_hid_send_input_report_cmd_t                         cmd_bt_hid_send_input_report;
	struct dumo_msg_bt_hid_virtual_cable_unplug_cmd_t                      cmd_bt_hid_virtual_cable_unplug;
	struct dumo_msg_bt_hid_get_report_response_cmd_t                       cmd_bt_hid_get_report_response;
	struct dumo_msg_bt_hid_set_report_response_cmd_t                       cmd_bt_hid_set_report_response;
	struct dumo_msg_bt_hid_get_set_report_reject_cmd_t                     cmd_bt_hid_get_set_report_reject;
	struct dumo_msg_bt_hid_opened_evt_t                                    evt_bt_hid_opened;
	struct dumo_msg_bt_hid_failed_evt_t                                    evt_bt_hid_failed;
	struct dumo_msg_bt_hid_output_report_evt_t                             evt_bt_hid_output_report;
	struct dumo_msg_bt_hid_state_changed_evt_t                             evt_bt_hid_state_changed;
	struct dumo_msg_bt_hid_get_report_evt_t                                evt_bt_hid_get_report;
	struct dumo_msg_bt_hid_set_report_evt_t                                evt_bt_hid_set_report;
	struct dumo_msg_bt_hid_open_rsp_t                                      rsp_bt_hid_open;
	struct dumo_msg_bt_hid_start_server_rsp_t                              rsp_bt_hid_start_server;
	struct dumo_msg_bt_hid_stop_server_rsp_t                               rsp_bt_hid_stop_server;
	struct dumo_msg_bt_hid_send_input_report_rsp_t                         rsp_bt_hid_send_input_report;
	struct dumo_msg_bt_hid_virtual_cable_unplug_rsp_t                      rsp_bt_hid_virtual_cable_unplug;
	struct dumo_msg_bt_hid_get_report_response_rsp_t                       rsp_bt_hid_get_report_response;
	struct dumo_msg_bt_hid_set_report_response_rsp_t                       rsp_bt_hid_set_report_response;
	struct dumo_msg_bt_hid_get_set_report_reject_rsp_t                     rsp_bt_hid_get_set_report_reject;
	struct dumo_msg_le_serial_open_cmd_t                                   cmd_le_serial_open;
	struct dumo_msg_le_serial_listen_cmd_t                                 cmd_le_serial_listen;
	struct dumo_msg_le_serial_opened_evt_t                                 evt_le_serial_opened;
	struct dumo_msg_le_serial_open_rsp_t                                   rsp_le_serial_open;
	struct dumo_msg_le_serial_listen_rsp_t                                 rsp_le_serial_listen;
};

}ALIGNED);

extern struct dumo_cmd_packet bglib_temp_msg;

/**This command can be used to reset the system. This command does not have a response, but it triggers one of the boot events (normal reset or boot to DFU mode) after re-boot. **/
#define dumo_cmd_dfu_reset(DFU) \
{\
bglib_temp_msg.cmd_dfu_reset.dfu=(DFU);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x0<<16)|((uint32)0x0<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**After re-booting the local device into DFU mode, this command can be used to define the starting address on the flash to where the new firmware will be written in.**/
#define dumo_cmd_dfu_flash_set_address(ADDRESS) \
{\
bglib_temp_msg.cmd_dfu_flash_set_address.address=(ADDRESS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x0<<16)|((uint32)0x1<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to upload the whole firmware image file in to the Bluetooth module. The recommended payload size of the command is 128 bytes, so multiple commands need to be issued one after the other until the whole .bin firmware image file is uploaded to the module. The next address of the flash sector in memory to write to is automatically updated by the bootloader after each individual command.**/
#define dumo_cmd_dfu_flash_upload(DATA_LEN,DATA_DATA) \
{\
bglib_temp_msg.cmd_dfu_flash_upload.data.len=(DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_dfu_flash_upload.data.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_dfu_flash_upload.data.len)&0xff)<<8)|((uint32)0x0<<16)|((uint32)0x2<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_dfu_flash_upload.data.len,(uint8*)DATA_DATA);\
}
/**This command can be used to tell to the device that the DFU file has been fully uploaded. To return the device back to normal mode the command {a href="#cmd_dfu_reset"}DFU Reset {/a} must be issued next.**/
#define dumo_cmd_dfu_flash_upload_finish() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x0<<16)|((uint32)0x3<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command does not trigger any event but the response to the command is used to verify that communication between the host and the module is working.**/
#define dumo_cmd_system_hello() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x0<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to reset the system. It does not have a response, but it triggers one of the boot events (normal reset or boot to DFU mode) depending on the selected boot mode.**/
#define dumo_cmd_system_reset(DFU) \
{\
bglib_temp_msg.cmd_system_reset.dfu=(DFU);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x1<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to set the most efficient power saving state allowed for the system.
            **/
#define dumo_cmd_system_set_max_power_mode(POWER_MODE) \
{\
bglib_temp_msg.cmd_system_set_max_power_mode.power_mode=(POWER_MODE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x2<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the Bluetooth address of the module.**/
#define dumo_cmd_system_get_bt_address() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x3<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command overrides the Bluetooth address which has been given at firmware update.**/
#define dumo_cmd_system_set_bt_address(ADDRESS) \
{\
memcpy(&bglib_temp_msg.cmd_system_set_bt_address.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x4<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the device's Bluetooth BR/EDR Class of Device (COD) information.**/
#define dumo_cmd_system_get_class_of_device() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x5<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command is used to set the Bluetooth BR/EDR Class of Device (COD) setting.**/
#define dumo_cmd_system_set_class_of_device(COD) \
{\
bglib_temp_msg.cmd_system_set_class_of_device.cod=(COD);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x6<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to reset the module and clear all settings including bonding information. It also resets the friendly name to its default value. The Apple iAP configuration and the persistent store are cleared. Note that the module's Bluetooth BR/EDR / LE public address is preserved.**/
#define dumo_cmd_system_reset_factory_settings() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x7<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the local device's Bluetooth BR/EDR friendly name. For Bluetooth LE the device name is stored in the GAP Service local 
            name parameter of the GATT database. If the device name attribute is not set as a constant in the project's GATT configuration XML file, it can be changed with 
            {a href="#cmd_gatt_server_write_attribute_value"}gatt_server_write_attribute_value{/a}. It is also possible to advertise with a different device name by using
            {a href="#cmd_le_gap_set_adv_data"}le_gap_set_adv_data{/a}, but in that case if a remote device connects and asks for the device name over GATT, the value will be 
            different from the advertised value.**/
#define dumo_cmd_system_set_local_name(NAME_LEN,NAME_DATA) \
{\
bglib_temp_msg.cmd_system_set_local_name.name.len=(NAME_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_system_set_local_name.name.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_system_set_local_name.name.len)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x8<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_system_set_local_name.name.len,(uint8*)NAME_DATA);\
}
/**This command can be used to read the Bluetooth BR/EDR friendly name of the local device. Note that for Bluetooth LE the device name is stored in the GAP Service of the GATT database.**/
#define dumo_cmd_system_get_local_name() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0x9<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**Write BT stack configuration parameters to flash**/
#define dumo_cmd_system_write_bt_config(MAX_INCOMING_ACL,RFCOMM_CREDITS_MAX,RFCOMM_MTU) \
{\
bglib_temp_msg.cmd_system_write_bt_config.max_incoming_acl=(MAX_INCOMING_ACL);\
bglib_temp_msg.cmd_system_write_bt_config.rfcomm_credits_max=(RFCOMM_CREDITS_MAX);\
bglib_temp_msg.cmd_system_write_bt_config.rfcomm_mtu=(RFCOMM_MTU);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x1<<16)|((uint32)0xa<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to discover other Bluetooth BR/EDR devices with Bluetooth BR/EDR inquiry.
            {br}Command {a href="#cmd_bt_gap_set_discovery_mode"}set_discovery_mode{/a} can be used to select how much information is delivered in discovery results
            **/
#define dumo_cmd_bt_gap_discover(TIMEOUT,LAP) \
{\
bglib_temp_msg.cmd_bt_gap_discover.timeout=(TIMEOUT);\
bglib_temp_msg.cmd_bt_gap_discover.lap=(LAP);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x0<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to cancel the ongoing Bluetooth BR/EDR device discovery (inquiry) procedure.
            **/
#define dumo_cmd_bt_gap_cancel_discovery() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x1<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the name of the remote Bluetooth BR/EDR device.**/
#define dumo_cmd_bt_gap_get_remote_name(ADDRESS) \
{\
memcpy(&bglib_temp_msg.cmd_bt_gap_get_remote_name.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x2<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set Bluetooth BR/EDR visibility and connectability.**/
#define dumo_cmd_bt_gap_set_mode(CONNECTABLE,DISCOVERABLE,LIMITED) \
{\
bglib_temp_msg.cmd_bt_gap_set_mode.connectable=(CONNECTABLE);\
bglib_temp_msg.cmd_bt_gap_set_mode.discoverable=(DISCOVERABLE);\
bglib_temp_msg.cmd_bt_gap_set_mode.limited=(LIMITED);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x3<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the device's Bluetooth BR/EDR visibility and connectability settings.**/
#define dumo_cmd_bt_gap_get_mode() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x4<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set default policies for connections. For more information on the use of policies please see BLUETOOTH SPECIFICATION Version 4.1 [Vol 2] Part E chapter 7.2.10.**/
#define dumo_cmd_bt_gap_set_policy(ALLOW_ROLE_CHANGE,ALLOW_SNIFF) \
{\
bglib_temp_msg.cmd_bt_gap_set_policy.allow_role_change=(ALLOW_ROLE_CHANGE);\
bglib_temp_msg.cmd_bt_gap_set_policy.allow_sniff=(ALLOW_SNIFF);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x5<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the default page timeout and scan mode.**/
#define dumo_cmd_bt_gap_set_parameters(PAGETIMEOUT,SCAN_INTERVAL,SCAN_WINDOW) \
{\
bglib_temp_msg.cmd_bt_gap_set_parameters.pagetimeout=(PAGETIMEOUT);\
bglib_temp_msg.cmd_bt_gap_set_parameters.scan_interval=(SCAN_INTERVAL);\
bglib_temp_msg.cmd_bt_gap_set_parameters.scan_window=(SCAN_WINDOW);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x6<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the maximum TX power for Bluetooth BR/EDR.**/
#define dumo_cmd_bt_gap_set_max_power(MAX_POWER) \
{\
bglib_temp_msg.cmd_bt_gap_set_max_power.max_power=(MAX_POWER);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x7<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
                This command can be used to open a Bluetooth BR/EDR connection to a remote device. After connection is established, closing another Bluetooth BR/EDR endpoints (like RFCOMM or HID) will not close this endpoint. It is needed to be closed manually with {a href="#cmd_endpoint_close"}dumo_cmd_endpoint_close{/a}. Command can be used for example when there is a need to bond the devices without opening an RFCOMM connection. After connection has been opened bonding can be done with the {a href="#cmd_sm_increase_security"} sm_increase_security{/a} command.
            **/
#define dumo_cmd_bt_gap_open(ADDRESS) \
{\
memcpy(&bglib_temp_msg.cmd_bt_gap_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x8<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**Set automatic sniff parameters for all connections. Please see the BLUETOOTH SPECIFICATION Version 4.1 [Vol 2] Part E chapter 7.2.2 for more information regarding the use of the related parameters.
            **/
#define dumo_cmd_bt_gap_set_auto_sniff(MODE,IDLE,MAX,MIN,ATTEMPT,TIMEOUT) \
{\
bglib_temp_msg.cmd_bt_gap_set_auto_sniff.mode=(MODE);\
bglib_temp_msg.cmd_bt_gap_set_auto_sniff.idle=(IDLE);\
bglib_temp_msg.cmd_bt_gap_set_auto_sniff.max=(MAX);\
bglib_temp_msg.cmd_bt_gap_set_auto_sniff.min=(MIN);\
bglib_temp_msg.cmd_bt_gap_set_auto_sniff.attempt=(ATTEMPT);\
bglib_temp_msg.cmd_bt_gap_set_auto_sniff.timeout=(TIMEOUT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)11+0)>>8)))|((((uint32)11+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0x9<<24);\
bglib_output(4+11,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to set the discovery mode. This determines the information given in {a href="#evt_bt_gap_discovery_result"}discovery_result{/a} events.
            **/
#define dumo_cmd_bt_gap_set_discovery_mode(MODE) \
{\
bglib_temp_msg.cmd_bt_gap_set_discovery_mode.mode=(MODE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0xa<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command configures Bluetooth BR/EDR channel classifications.. If successful, a {a href="#evt_bt_gap_host_channel_classification_complete"}bt_gap_host_channel_classification_complete{/a} event will follow when the controller has completed the configuration.**/
#define dumo_cmd_bt_gap_set_host_channel_classification(CHANNEL_MAP_LEN,CHANNEL_MAP_DATA) \
{\
bglib_temp_msg.cmd_bt_gap_set_host_channel_classification.channel_map.len=(CHANNEL_MAP_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_bt_gap_set_host_channel_classification.channel_map.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_bt_gap_set_host_channel_classification.channel_map.len)&0xff)<<8)|((uint32)0x2<<16)|((uint32)0xb<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_bt_gap_set_host_channel_classification.channel_map.len,(uint8*)CHANNEL_MAP_DATA);\
}
/**This command can be used to open a Bluetooth LE connection. Scanning parameters can be configured with the le_gap_set_scan_parameters command before issuing this command. To cancel on an ongoing connection process use the {a href="#cmd_endpoint_close"}endpoint_close{/a} command.**/
#define dumo_cmd_le_gap_open(ADDRESS,ADDRESS_TYPE) \
{\
memcpy(&bglib_temp_msg.cmd_le_gap_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_le_gap_open.address_type=(ADDRESS_TYPE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)7+0)>>8)))|((((uint32)7+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x0<<24);\
bglib_output(4+7,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to configure the current Bluetooth LE GAP Connectable and Discoverable modes. It can be used to enable advertisements and/or allow incoming connections. To exit from this mode (to stop advertising and/or disallow incoming connections), issue this command with the Not Discoverable / Not Connectable parameter values. After connection establishment with remote central module, advertising on the peripheral module is stopped and switched to non - connectable mode. Availability of the discoverable and connectable modes depend on device role (advertiser, scanner, central, peripheral). For more information see Bluetooth 4.0 specification.**/
#define dumo_cmd_le_gap_set_mode(DISCOVER,CONNECT) \
{\
bglib_temp_msg.cmd_le_gap_set_mode.discover=(DISCOVER);\
bglib_temp_msg.cmd_le_gap_set_mode.connect=(CONNECT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start the GAP discovery procedure to scan for advertising devices, that is to perform a device discovery. Scanning parameters 
            can be configured with the {a href="#cmd_le_gap_set_scan_parameters"}le_gap_set_scan_parameters{/a} command before issuing this command. To cancel an ongoing 
            discovery process use the {a href="#cmd_le_gap_end_procedure"}le_gap_end_procedure{/a} command.**/
#define dumo_cmd_le_gap_discover(MODE) \
{\
bglib_temp_msg.cmd_le_gap_discover.mode=(MODE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x2<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to end a current GAP procedure.**/
#define dumo_cmd_le_gap_end_procedure() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x3<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set Bluetooth LE advertisement parameters.**/
#define dumo_cmd_le_gap_set_adv_parameters(INTERVAL_MIN,INTERVAL_MAX,CHANNEL_MAP) \
{\
bglib_temp_msg.cmd_le_gap_set_adv_parameters.interval_min=(INTERVAL_MIN);\
bglib_temp_msg.cmd_le_gap_set_adv_parameters.interval_max=(INTERVAL_MAX);\
bglib_temp_msg.cmd_le_gap_set_adv_parameters.channel_map=(CHANNEL_MAP);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x4<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the default Bluetooth LE connection parameters. The configured values are valid for all subsequent connections that will 
            be established. For changing the parameters of an already established connection use the command {a href="#cmd_le_connection_set_parameters"}le_connection_set_parameters{/a}.**/
#define dumo_cmd_le_gap_set_conn_parameters(MIN_INTERVAL,MAX_INTERVAL,LATENCY,TIMEOUT) \
{\
bglib_temp_msg.cmd_le_gap_set_conn_parameters.min_interval=(MIN_INTERVAL);\
bglib_temp_msg.cmd_le_gap_set_conn_parameters.max_interval=(MAX_INTERVAL);\
bglib_temp_msg.cmd_le_gap_set_conn_parameters.latency=(LATENCY);\
bglib_temp_msg.cmd_le_gap_set_conn_parameters.timeout=(TIMEOUT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)8+0)>>8)))|((((uint32)8+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x5<<24);\
bglib_output(4+8,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set Bluetooth LE scan parameters.**/
#define dumo_cmd_le_gap_set_scan_parameters(SCAN_INTERVAL,SCAN_WINDOW,ACTIVE) \
{\
bglib_temp_msg.cmd_le_gap_set_scan_parameters.scan_interval=(SCAN_INTERVAL);\
bglib_temp_msg.cmd_le_gap_set_scan_parameters.scan_window=(SCAN_WINDOW);\
bglib_temp_msg.cmd_le_gap_set_scan_parameters.active=(ACTIVE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x6<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the data in advertisement packets or in the scan response packets. This data is used when advertising in user data mode. 
            It is recommended to set both the advertisement data and scan response data at the same time.**/
#define dumo_cmd_le_gap_set_adv_data(SCAN_RSP,ADV_DATA_LEN,ADV_DATA_DATA) \
{\
bglib_temp_msg.cmd_le_gap_set_adv_data.scan_rsp=(SCAN_RSP);\
bglib_temp_msg.cmd_le_gap_set_adv_data.adv_data.len=(ADV_DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+bglib_temp_msg.cmd_le_gap_set_adv_data.adv_data.len)>>8)))|((((uint32)2+bglib_temp_msg.cmd_le_gap_set_adv_data.adv_data.len)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x7<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_le_gap_set_adv_data.adv_data.len,(uint8*)ADV_DATA_DATA);\
}
/**This command can be used to set the maximum TX power for Bluetooth LE.**/
#define dumo_cmd_le_gap_set_max_power(MAX_POWER) \
{\
bglib_temp_msg.cmd_le_gap_set_max_power.max_power=(MAX_POWER);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x8<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command configures Bluetooth LE channel classifications. If successful, a {a href="#evt_le_gap_host_channel_classification_complete"}bt_gap_host_channel_classification_complete{/a} event will follow when the controller has completed the configuration.**/
#define dumo_cmd_le_gap_set_host_channel_classification(CHANNEL_MAP_LEN,CHANNEL_MAP_DATA) \
{\
bglib_temp_msg.cmd_le_gap_set_host_channel_classification.channel_map.len=(CHANNEL_MAP_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_le_gap_set_host_channel_classification.channel_map.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_le_gap_set_host_channel_classification.channel_map.len)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0x9<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_le_gap_set_host_channel_classification.channel_map.len,(uint8*)CHANNEL_MAP_DATA);\
}
/**
            This command can be used to filter scan responses and advertisements by device name.
            **/
#define dumo_cmd_le_gap_set_scan_result_filter(NAME_LEN,NAME_DATA) \
{\
bglib_temp_msg.cmd_le_gap_set_scan_result_filter.name.len=(NAME_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_le_gap_set_scan_result_filter.name.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_le_gap_set_scan_result_filter.name.len)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0xa<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_le_gap_set_scan_result_filter.name.len,(uint8*)NAME_DATA);\
}
/**
            This command can be used to remove all scan filters.
            **/
#define dumo_cmd_le_gap_scan_filter_clear() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0xb<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set random static address for LE**/
#define dumo_cmd_le_gap_set_random_static_address(STATIC_ADDRESS,ENABLE) \
{\
memcpy(&bglib_temp_msg.cmd_le_gap_set_random_static_address.static_address,(STATIC_ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_le_gap_set_random_static_address.enable=(ENABLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)7+0)>>8)))|((((uint32)7+0)&0xff)<<8)|((uint32)0x3<<16)|((uint32)0xc<<24);\
bglib_output(4+7,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**Open an RFCOMM channel to remote device. An existing connection, or a connection attempt before the page timeout expires, 
            can be closed with the {a href="#cmd_endpoint_close"}endpoint_close{/a} command.
            {br}{br}
            NOTE: Controller does not allow starting a BR/EDR connection while a BR/EDR discovery 
            process in in progress. Please terminate any BR/EDR discovery with 
            {a href="#cmd_bt_gap_cancel_discovery"}
            bt_gap_cancel_discovery{/a} 
            command prior to starting the RFCOMM connection.
            **/
#define dumo_cmd_bt_rfcomm_open(ADDRESS,STREAMING_DESTINATION,UUID_LEN,UUID_DATA) \
{\
memcpy(&bglib_temp_msg.cmd_bt_rfcomm_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_bt_rfcomm_open.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.cmd_bt_rfcomm_open.uuid.len=(UUID_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)8+bglib_temp_msg.cmd_bt_rfcomm_open.uuid.len)>>8)))|((((uint32)8+bglib_temp_msg.cmd_bt_rfcomm_open.uuid.len)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x0<<24);\
bglib_output(4+8,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_bt_rfcomm_open.uuid.len,(uint8*)UUID_DATA);\
}
/**
            This command can be used to start a RFCOMM server as defined in the referenced SDP-entry.
            **/
#define dumo_cmd_bt_rfcomm_start_server(SDP_ID,STREAMING_DESTINATION) \
{\
bglib_temp_msg.cmd_bt_rfcomm_start_server.sdp_id=(SDP_ID);\
bglib_temp_msg.cmd_bt_rfcomm_start_server.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to open an RFCOMM channel to a remote device using a fixed RFCOMM port number. This bypasses the Service Discovery Protocol procedure
            for retrieving the RFCOMM port from the remote device's Serial Port Profile SDP entry, thus speeding up connection establishment.**/
#define dumo_cmd_bt_rfcomm_open_port(ADDRESS,STREAMING_DESTINATION,PORT) \
{\
memcpy(&bglib_temp_msg.cmd_bt_rfcomm_open_port.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_bt_rfcomm_open_port.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.cmd_bt_rfcomm_open_port.port=(PORT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)8+0)>>8)))|((((uint32)8+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x2<<24);\
bglib_output(4+8,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to stop an RFCOMM server.
            **/
#define dumo_cmd_bt_rfcomm_stop_server(SDP_ID) \
{\
bglib_temp_msg.cmd_bt_rfcomm_stop_server.sdp_id=(SDP_ID);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x3<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command sets modem control status bits for RFCOMM connection. This is a legacy support feature and {b}SHOULD NOT BE USED FOR FLOW CONTROL{/b}.{br}
            For description and meaning of bits in this field see the ETSI TS 07.10 specification. 
            **/
#define dumo_cmd_bt_rfcomm_modem_status(ENDPOINT,MODEM,MASK) \
{\
bglib_temp_msg.cmd_bt_rfcomm_modem_status.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_rfcomm_modem_status.modem=(MODEM);\
bglib_temp_msg.cmd_bt_rfcomm_modem_status.mask=(MASK);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x4<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to start a RFCOMM server on specific port. In contrast to {a href="#cmd_bt_rfcomm_start_server"}cmd_bt_rfcomm_start_server{/a} command, components of the port descriptor are not loaded from SDP records.
            **/
#define dumo_cmd_bt_rfcomm_start_server_port(PORT,STREAMING_DESTINATION) \
{\
bglib_temp_msg.cmd_bt_rfcomm_start_server_port.port=(PORT);\
bglib_temp_msg.cmd_bt_rfcomm_start_server_port.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x5<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to stop an RFCOMM server that has been started ona port.
            **/
#define dumo_cmd_bt_rfcomm_stop_server_port(PORT) \
{\
bglib_temp_msg.cmd_bt_rfcomm_stop_server_port.port=(PORT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x6<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command sets starting value modem control status bits for new outogoing RFCOMM connections. 
            **/
#define dumo_cmd_bt_rfcomm_set_modem_status_default(MODEM) \
{\
bglib_temp_msg.cmd_bt_rfcomm_set_modem_status_default.modem=(MODEM);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x4<<16)|((uint32)0x7<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read Identity Profile information from a remote Bluetooth BR/EDR device.**/
#define dumo_cmd_identity_remote_identity(ADDRESS) \
{\
memcpy(&bglib_temp_msg.cmd_identity_remote_identity.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0x5<<16)|((uint32)0x0<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read Identity Profile information from the local device.**/
#define dumo_cmd_identity_local_identity() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x5<<16)|((uint32)0x1<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to modify some fields in the local identity profile.**/
#define dumo_cmd_identity_modify_local_identity(VENDOR,PRODUCT,VERSION) \
{\
bglib_temp_msg.cmd_identity_modify_local_identity.vendor=(VENDOR);\
bglib_temp_msg.cmd_identity_modify_local_identity.product=(PRODUCT);\
bglib_temp_msg.cmd_identity_modify_local_identity.version=(VERSION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0x5<<16)|((uint32)0x2<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to search for Serial Port Profile (SPP) services in a Bluetooth BR/EDR device. The reply is an event specifying the RFCOMM port found.
            **/
#define dumo_cmd_bt_sdp_search_rfcomm_port(ADDRESS,UUID_LEN,UUID_DATA) \
{\
memcpy(&bglib_temp_msg.cmd_bt_sdp_search_rfcomm_port.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_bt_sdp_search_rfcomm_port.uuid.len=(UUID_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)7+bglib_temp_msg.cmd_bt_sdp_search_rfcomm_port.uuid.len)>>8)))|((((uint32)7+bglib_temp_msg.cmd_bt_sdp_search_rfcomm_port.uuid.len)&0xff)<<8)|((uint32)0x6<<16)|((uint32)0x0<<24);\
bglib_output(4+7,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_bt_sdp_search_rfcomm_port.uuid.len,(uint8*)UUID_DATA);\
}
/**
            This command can be used to load an SDP entry from the internal filesystem to the SDP server.
            {br}NOTE: command only loads SDP record. It does not do anything else. Like start RFCOMM server
            **/
#define dumo_cmd_bt_sdp_load_entry(SDP_ID) \
{\
bglib_temp_msg.cmd_bt_sdp_load_entry.sdp_id=(SDP_ID);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x6<<16)|((uint32)0x1<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This function load custom defined SDP record into the SDP server.{br}
            Note that this funtion does not verify the correctness of the record at all.
            **/
#define dumo_cmd_bt_sdp_add_entry(SDP_ID,RECORD_LEN,RECORD_DATA) \
{\
bglib_temp_msg.cmd_bt_sdp_add_entry.sdp_id=(SDP_ID);\
bglib_temp_msg.cmd_bt_sdp_add_entry.record.len=(RECORD_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+bglib_temp_msg.cmd_bt_sdp_add_entry.record.len)>>8)))|((((uint32)5+bglib_temp_msg.cmd_bt_sdp_add_entry.record.len)&0xff)<<8)|((uint32)0x6<<16)|((uint32)0x2<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_bt_sdp_add_entry.record.len,(uint8*)RECORD_DATA);\
}
/**
            This function removes custom defined SDP record from the SDP server.
            **/
#define dumo_cmd_bt_sdp_delete_entry(SDP_ID) \
{\
bglib_temp_msg.cmd_bt_sdp_delete_entry.sdp_id=(SDP_ID);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x6<<16)|((uint32)0x3<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to list all RFCOMM and HID connections and 
            check parameters of each connection.**/
#define dumo_cmd_bt_connection_list() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x0<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the connection state to active.**/
#define dumo_cmd_bt_connection_set_active(ENDPOINT) \
{\
bglib_temp_msg.cmd_bt_connection_set_active.endpoint=(ENDPOINT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x1<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the sniff parameters for a connection. Please see BLUETOOTH SPECIFICATION Version 4.1 [Vol 2] Part E chapter 7.2.2 for more information regarding the use of the related parameters.**/
#define dumo_cmd_bt_connection_set_sniff(ENDPOINT,MAX,MIN,ATTEMPT,TIMEOUT) \
{\
bglib_temp_msg.cmd_bt_connection_set_sniff.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_connection_set_sniff.max=(MAX);\
bglib_temp_msg.cmd_bt_connection_set_sniff.min=(MIN);\
bglib_temp_msg.cmd_bt_connection_set_sniff.attempt=(ATTEMPT);\
bglib_temp_msg.cmd_bt_connection_set_sniff.timeout=(TIMEOUT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)9+0)>>8)))|((((uint32)9+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x2<<24);\
bglib_output(4+9,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the Bluetooth connection local role. Note that this may not be possible if the remote device is the Central and does not 
            allow role changes.**/
#define dumo_cmd_bt_connection_set_role(ENDPOINT,ROLE) \
{\
bglib_temp_msg.cmd_bt_connection_set_role.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_connection_set_role.role=(ROLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x3<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the connection supervision timeout.**/
#define dumo_cmd_bt_connection_set_supervision_timeout(ENDPOINT,SUPERVISIONTIMEOUT) \
{\
bglib_temp_msg.cmd_bt_connection_set_supervision_timeout.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_connection_set_supervision_timeout.supervisiontimeout=(SUPERVISIONTIMEOUT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x4<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**Get RSSI value of a connection.
            **/
#define dumo_cmd_bt_connection_get_rssi(CONNECTION) \
{\
bglib_temp_msg.cmd_bt_connection_get_rssi.connection=(CONNECTION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x5<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
			Read Bluetooth Clock of a connection/piconet.
			{br}For detailed information, see Bluetooth Specification 4.1, vol. 2, part E, section 7.5.6 (Read Clock Command).
			**/
#define dumo_cmd_bt_connection_read_clock(CONNECTION) \
{\
bglib_temp_msg.cmd_bt_connection_read_clock.connection=(CONNECTION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x7<<16)|((uint32)0x6<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to request a change in the connection parameters of a Bluetooth LE connection.**/
#define dumo_cmd_le_connection_set_parameters(CONNECTION,MIN_INTERVAL,MAX_INTERVAL,LATENCY,TIMEOUT) \
{\
bglib_temp_msg.cmd_le_connection_set_parameters.connection=(CONNECTION);\
bglib_temp_msg.cmd_le_connection_set_parameters.min_interval=(MIN_INTERVAL);\
bglib_temp_msg.cmd_le_connection_set_parameters.max_interval=(MAX_INTERVAL);\
bglib_temp_msg.cmd_le_connection_set_parameters.latency=(LATENCY);\
bglib_temp_msg.cmd_le_connection_set_parameters.timeout=(TIMEOUT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)9+0)>>8)))|((((uint32)9+0)&0xff)<<8)|((uint32)0x8<<16)|((uint32)0x0<<24);\
bglib_output(4+9,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to list all LE connections and check their parameters.**/
#define dumo_cmd_le_connection_list() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x8<<16)|((uint32)0x1<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the maximum number of GATT Message Transfer Units (MTU).
             If max_mtu is non-default, MTU is exchanged automatically after Bluetooth LE connection has been established. 
             **/
#define dumo_cmd_gatt_set_max_mtu(MAX_MTU) \
{\
bglib_temp_msg.cmd_gatt_set_max_mtu.max_mtu=(MAX_MTU);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x0<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to discover all the primary services of a remote GATT database. This command generates a unique gatt_service event for 
            every discovered primary service. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that this GATT procedure 
            has successfully completed or failed with error.**/
#define dumo_cmd_gatt_discover_primary_services(CONNECTION) \
{\
bglib_temp_msg.cmd_gatt_discover_primary_services.connection=(CONNECTION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x1<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to discover primary services with the specified UUID in a remote GATT database. This command generates unique gatt_service event for every discovered primary service. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that this GATT procedure has succesfully completed or failed with error.**/
#define dumo_cmd_gatt_discover_primary_services_by_uuid(CONNECTION,UUID_LEN,UUID_DATA) \
{\
bglib_temp_msg.cmd_gatt_discover_primary_services_by_uuid.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_discover_primary_services_by_uuid.uuid.len=(UUID_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+bglib_temp_msg.cmd_gatt_discover_primary_services_by_uuid.uuid.len)>>8)))|((((uint32)2+bglib_temp_msg.cmd_gatt_discover_primary_services_by_uuid.uuid.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x2<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_discover_primary_services_by_uuid.uuid.len,(uint8*)UUID_DATA);\
}
/**This command can be used to discover all characteristics of the defined GATT service from a remote GATT database. This command generates a unique gatt_characteristic event for every discovered characteristic. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that this GATT procedure has succesfully completed or failed with error.**/
#define dumo_cmd_gatt_discover_characteristics(CONNECTION,SERVICE) \
{\
bglib_temp_msg.cmd_gatt_discover_characteristics.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_discover_characteristics.service=(SERVICE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x3<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to discover all the characteristics of the specified GATT service in a remote GATT database having the specified UUID. This command
generates a unique gatt_characteristic event for every discovered
characteristic having the specified UUID. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that
this GATT procedure has successfully completed or failed with error.
            **/
#define dumo_cmd_gatt_discover_characteristics_by_uuid(CONNECTION,SERVICE,UUID_LEN,UUID_DATA) \
{\
bglib_temp_msg.cmd_gatt_discover_characteristics_by_uuid.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_discover_characteristics_by_uuid.service=(SERVICE);\
bglib_temp_msg.cmd_gatt_discover_characteristics_by_uuid.uuid.len=(UUID_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+bglib_temp_msg.cmd_gatt_discover_characteristics_by_uuid.uuid.len)>>8)))|((((uint32)6+bglib_temp_msg.cmd_gatt_discover_characteristics_by_uuid.uuid.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x4<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_discover_characteristics_by_uuid.uuid.len,(uint8*)UUID_DATA);\
}
/**This command can be used to enable or disable the notifications and indications being sent from a remote GATT server. This procedure discovers a characteristic client configuration descriptor and writes the related configuration flags to a remote GATT database. A received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that this GATT procedure has successfully completed or that is has failed with an error. {br}{br}
            The alternative way to subscribe attribute's notification or subscribe attribute's indication is to discover Client Characteristic Configuration Descriptor (with {a href="#cmd_gatt_find_information_request"}cmd_gatt_find_information_request{/a}), and then to write {a href="#enum_gatt_client_config_flag"}expected value{/a} (with {a href="#cmd_gatt_write_descriptor_value"}cmd_gatt_write_descriptor_value{/a}) to its handler.
            **/
#define dumo_cmd_gatt_set_characteristic_notification(CONNECTION,CHARACTERISTIC,FLAGS) \
{\
bglib_temp_msg.cmd_gatt_set_characteristic_notification.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_set_characteristic_notification.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_set_characteristic_notification.flags=(FLAGS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x5<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to discover all the descriptors of the specified remote GATT characteristics in a remote GATT database. This command generates a unique gatt_descriptor event for every discovered descriptor. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that this GATT procedure has succesfully completed or failed with error.**/
#define dumo_cmd_gatt_discover_descriptors(CONNECTION,CHARACTERISTIC) \
{\
bglib_temp_msg.cmd_gatt_discover_descriptors.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_discover_descriptors.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x6<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the value of a characteristic from a remote GATT database. A single {a href="#evt_gatt_characteristic_value"}
            gatt_characteristic_value{/a} event is generated 
            if the length of the characteristic value returned by the remote GATT server is less than or equal to the size of the GATT MTU. If the length of the value 
            exceeds the size of the GATT MTU more than one {a href="#evt_gatt_characteristic_value"}
            gatt_characteristic_value{/a} event is generated because the firmware will automatically use the "read long" 
            GATT procedure. A received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that all data has been read successfully or that an error response has been received.**/
#define dumo_cmd_gatt_read_characteristic_value(CONNECTION,CHARACTERISTIC) \
{\
bglib_temp_msg.cmd_gatt_read_characteristic_value.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_read_characteristic_value.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x7<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the characteristic value of a service from a remote GATT database by giving the UUID of the characteristic and the 
            handle of the service containing this characteristic. A single {a href="#evt_gatt_characteristic_value"}
            gatt_characteristic_value{/a} event is generated if the length of the characteristic value returned by the 
            remote GATT server is less than or equal to the size of the GATT MTU. If the length of the value exceeds the size of the GATT MTU more than one 
            {a href="#evt_gatt_characteristic_value"}
            gatt_characteristic_value{/a} event is generated because the firmware will automatically use the "read long" GATT procedure. 
            A received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that all data has been read successfully or that an error response has been received.**/
#define dumo_cmd_gatt_read_characteristic_value_by_uuid(CONNECTION,SERVICE,UUID_LEN,UUID_DATA) \
{\
bglib_temp_msg.cmd_gatt_read_characteristic_value_by_uuid.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_read_characteristic_value_by_uuid.service=(SERVICE);\
bglib_temp_msg.cmd_gatt_read_characteristic_value_by_uuid.uuid.len=(UUID_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+bglib_temp_msg.cmd_gatt_read_characteristic_value_by_uuid.uuid.len)>>8)))|((((uint32)6+bglib_temp_msg.cmd_gatt_read_characteristic_value_by_uuid.uuid.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x8<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_read_characteristic_value_by_uuid.uuid.len,(uint8*)UUID_DATA);\
}
/**This command can be used to write the value of a characteristic in a remote GATT database. If the length of the given value is greater than 
            the exchanged GATT MTU (Message Transfer Unit), "write long" GATT procedure is used automatically. Received 
            {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that all data has been written successfully or that an error response 
            has been received.**/
#define dumo_cmd_gatt_write_characteristic_value(CONNECTION,CHARACTERISTIC,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_write_characteristic_value.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_write_characteristic_value.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_write_characteristic_value.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+bglib_temp_msg.cmd_gatt_write_characteristic_value.value.len)>>8)))|((((uint32)4+bglib_temp_msg.cmd_gatt_write_characteristic_value.value.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x9<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_write_characteristic_value.value.len,(uint8*)VALUE_DATA);\
}
/**This command can be used to write the value of a characteristic in a
            remote GATT database. This command does not generate any event. All failures on the server are ignored silently. 
            For example, if an error is generated in the remote GATT server and the given value is not written into database no error message will be reported to the local 
            GATT client. Note that this command cannot be used to write long values.**/
#define dumo_cmd_gatt_write_characteristic_value_without_response(CONNECTION,CHARACTERISTIC,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_write_characteristic_value_without_response.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_write_characteristic_value_without_response.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_write_characteristic_value_without_response.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+bglib_temp_msg.cmd_gatt_write_characteristic_value_without_response.value.len)>>8)))|((((uint32)4+bglib_temp_msg.cmd_gatt_write_characteristic_value_without_response.value.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0xa<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_write_characteristic_value_without_response.value.len,(uint8*)VALUE_DATA);\
}
/**This command can be used to add a characteristic value to the write queue of a remote GATT server. 
            This command can be used in cases where very long attributes need to be written, or a set of values needs to be written atomically.
            In all cases where the amount of data to transfer fits into the BGAPI payload the command 
            {a href="#cmd_gatt_write_characteristic_value"}gatt_write_characteristic_value{/a} is recommended for writing long values since it transparently 
            performs the prepare_write and execute_write commands. A received {a href="#evt_gatt_characteristic_value"}evt_gatt_characteristic_value{/a} event can
            be used to verify that the data has been transmitted. Writes are executed or cancelled with the {a href="#cmd_gatt_execute_characteristic_value_write"}execute_characteristic_value_write{/a} command.
            Whether the writes succeeded or not are indicated in the response of the {a href="#cmd_gatt_execute_characteristic_value_write"}execute_characteristic_value_write{/a} command.
            **/
#define dumo_cmd_gatt_prepare_characteristic_value_write(CONNECTION,CHARACTERISTIC,OFFSET,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.offset=(OFFSET);\
bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.value.len)>>8)))|((((uint32)6+bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.value.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0xb<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_prepare_characteristic_value_write.value.len,(uint8*)VALUE_DATA);\
}
/**This command can be used to commit or cancel previously queued writes to a long characteristic of a remote GATT server.
            Writes are sent to queue with {a href="#cmd_gatt_prepare_characteristic_value_write"}prepare_characteristic_value_write{/a} command. 
            Content, offset and length of queued values are validated by this procedure. A received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a}
            event indicates that all data has been written successfully or that an error response has been received.
            **/
#define dumo_cmd_gatt_execute_characteristic_value_write(CONNECTION,FLAGS) \
{\
bglib_temp_msg.cmd_gatt_execute_characteristic_value_write.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_execute_characteristic_value_write.flags=(FLAGS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0xc<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command must be used to send a characteristic confirmation to a remote GATT server after receiving an indication. 
            The {a href="#evt_gatt_characteristic_value"}gatt_characteristic_value_event{/a} carries the att_opcode containing handle_value_indication (0x1e) which reveals 
            that an indication has been received and this must be confirmed with this command. Confirmation needs to be sent within 30 seconds, otherwise the GATT transactions 
            between the client and the server are discontinued.**/
#define dumo_cmd_gatt_send_characteristic_confirmation(CONNECTION) \
{\
bglib_temp_msg.cmd_gatt_send_characteristic_confirmation.connection=(CONNECTION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0xd<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the descriptor value of a characteristic in a remote GATT database. A single {a href="#evt_gatt_descriptor_value"}
            gatt_descriptor_value{/a} event is  generated if the length of the descriptor value returned by the remote GATT server is less than or equal to the size of the 
            GATT MTU. If the length of the value exceeds the size of the GATT MTU more than one {a href="#evt_gatt_descriptor_value"}gatt_descriptor_value{/a} event is generated because the firmware 
            will automatically use the "read long" GATT procedure. A received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that all 
            data has been read successfully or that an error response has been received.**/
#define dumo_cmd_gatt_read_descriptor_value(CONNECTION,DESCRIPTOR) \
{\
bglib_temp_msg.cmd_gatt_read_descriptor_value.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_read_descriptor_value.descriptor=(DESCRIPTOR);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0xe<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to write the value of a characteristic descriptor in a remote GATT database. If the length of the given value is greater 
            than the exchanged GATT MTU size, "write long" GATT procedure is used automatically. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} 
            event indicates that all data has been written succesfully or that an error response has been received.
            **/
#define dumo_cmd_gatt_write_descriptor_value(CONNECTION,DESCRIPTOR,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_write_descriptor_value.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_write_descriptor_value.descriptor=(DESCRIPTOR);\
bglib_temp_msg.cmd_gatt_write_descriptor_value.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+bglib_temp_msg.cmd_gatt_write_descriptor_value.value.len)>>8)))|((((uint32)4+bglib_temp_msg.cmd_gatt_write_descriptor_value.value.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0xf<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_write_descriptor_value.value.len,(uint8*)VALUE_DATA);\
}
/**This command can be used to find out if a service of a remote GATT database includes one or more other services. This command generates a unique gatt_service_completed event for each included service. This command generates a unique gatt_service event for every discovered service. Received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that this GATT procedure has successfully completed or failed with error.**/
#define dumo_cmd_gatt_find_included_services(CONNECTION,SERVICE) \
{\
bglib_temp_msg.cmd_gatt_find_included_services.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_find_included_services.service=(SERVICE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x10<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the values of multiple characteristics from a remote GATT database at once. 
            {a href="#evt_gatt_characteristic_value"}gatt_characteristic_value{/a} events are generated as the values are returned by the remote GATT server.
            A received {a href="#evt_gatt_procedure_completed"}gatt_procedure_completed{/a} event indicates that either all data has been read successfully or that an error response has been received.**/
#define dumo_cmd_gatt_read_multiple_characteristic_values(CONNECTION,CHARACTERISTIC_LIST_LEN,CHARACTERISTIC_LIST_DATA) \
{\
bglib_temp_msg.cmd_gatt_read_multiple_characteristic_values.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_read_multiple_characteristic_values.characteristic_list.len=(CHARACTERISTIC_LIST_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+bglib_temp_msg.cmd_gatt_read_multiple_characteristic_values.characteristic_list.len)>>8)))|((((uint32)2+bglib_temp_msg.cmd_gatt_read_multiple_characteristic_values.characteristic_list.len)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x11<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_read_multiple_characteristic_values.characteristic_list.len,(uint8*)CHARACTERISTIC_LIST_DATA);\
}
/**This command is used to obtain the mapping of attribute handles with their associated types (UUIDs) in a given handle range.**/
#define dumo_cmd_gatt_find_information_request(CONNECTION,START_HANDLE,END_HANDLE) \
{\
bglib_temp_msg.cmd_gatt_find_information_request.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_find_information_request.start_handle=(START_HANDLE);\
bglib_temp_msg.cmd_gatt_find_information_request.end_handle=(END_HANDLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0x9<<16)|((uint32)0x12<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the value of an attribute from a local GATT database.**/
#define dumo_cmd_gatt_server_read_attribute_value(ATTRIBUTE,OFFSET) \
{\
bglib_temp_msg.cmd_gatt_server_read_attribute_value.attribute=(ATTRIBUTE);\
bglib_temp_msg.cmd_gatt_server_read_attribute_value.offset=(OFFSET);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x0<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the type of an attribute from a local GATT database. The type is a UUID, usually 16 or 128 bits long.**/
#define dumo_cmd_gatt_server_read_attribute_type(ATTRIBUTE) \
{\
bglib_temp_msg.cmd_gatt_server_read_attribute_type.attribute=(ATTRIBUTE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to write the value of an attribute in the local GATT database. Writing the value of a characteristic of the local GATT database 
            will not trigger notifications or indications to the remote GATT client in case such characteristic has property of indicate or notify and the client has enabled 
            notification or indication. Notifications and indications are sent to the remote GATT client using {a href="#cmd_gatt_server_send_characteristic_notification"}
            gatt_server_send_characteristic_notification{/a} command.**/
#define dumo_cmd_gatt_server_write_attribute_value(ATTRIBUTE,OFFSET,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_server_write_attribute_value.attribute=(ATTRIBUTE);\
bglib_temp_msg.cmd_gatt_server_write_attribute_value.offset=(OFFSET);\
bglib_temp_msg.cmd_gatt_server_write_attribute_value.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+bglib_temp_msg.cmd_gatt_server_write_attribute_value.value.len)>>8)))|((((uint32)5+bglib_temp_msg.cmd_gatt_server_write_attribute_value.value.len)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x2<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_server_write_attribute_value.value.len,(uint8*)VALUE_DATA);\
}
/**This command must be used to send a response to a {a href="#evt_gatt_server_user_read_request"}user_read_request{/a} event. The response needs to be sent within 30 second, otherwise no more GATT 
            transactions are allowed by the remote side. If attr_errorcode is set to 0 the characteristic value is sent to the remote GATT client in the normal way. Other attr_errorcode 
            values will cause the local GATT server to send an attribute protocol error response instead of the actual data.**/
#define dumo_cmd_gatt_server_send_user_read_response(CONNECTION,CHARACTERISTIC,ATT_ERRORCODE,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_server_send_user_read_response.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_server_send_user_read_response.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_server_send_user_read_response.att_errorcode=(ATT_ERRORCODE);\
bglib_temp_msg.cmd_gatt_server_send_user_read_response.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+bglib_temp_msg.cmd_gatt_server_send_user_read_response.value.len)>>8)))|((((uint32)5+bglib_temp_msg.cmd_gatt_server_send_user_read_response.value.len)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x3<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_server_send_user_read_response.value.len,(uint8*)VALUE_DATA);\
}
/**This command must be used to send a response to a {a href="#evt_gatt_server_user_write_request"}gatt_server_user_write_request{/a} event. The response needs to be sent within 30 seconds, otherwise no more GATT 
            transactions are allowed by the remote side. If attr_errorcode is set to 0 the ATT protocol's write response is sent to indicate to the remote GATT client that 
            the write operation was processed successfully. Other values will cause the local GATT server to send an ATT protocol error response.**/
#define dumo_cmd_gatt_server_send_user_write_response(CONNECTION,CHARACTERISTIC,ATT_ERRORCODE) \
{\
bglib_temp_msg.cmd_gatt_server_send_user_write_response.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_server_send_user_write_response.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_server_send_user_write_response.att_errorcode=(ATT_ERRORCODE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x4<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to send notifications or indications to a remote GATT client. Notification or indication is sent only if the client has enabled 
            them by setting the corresponding flag to the Client Characteristic Configuration descriptor. A new indication cannot be sent before a confirmation 
            from the GATT client is first received. The confirmation is indicated by {a href="#evt_gatt_server_characteristic_status"}gatt_server_characteristic_status event{/a}.**/
#define dumo_cmd_gatt_server_send_characteristic_notification(CONNECTION,CHARACTERISTIC,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_gatt_server_send_characteristic_notification.connection=(CONNECTION);\
bglib_temp_msg.cmd_gatt_server_send_characteristic_notification.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_gatt_server_send_characteristic_notification.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+bglib_temp_msg.cmd_gatt_server_send_characteristic_notification.value.len)>>8)))|((((uint32)4+bglib_temp_msg.cmd_gatt_server_send_characteristic_notification.value.len)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x5<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_gatt_server_send_characteristic_notification.value.len,(uint8*)VALUE_DATA);\
}
/**This command can be used to enable or disable service visibility when other device perform Service Discovery procedure.
            Refer to 'constants' file produced by build process to find handles for services. Only services with 'id' attribute in gatt description file
            are stored in 'constants' file.
            **/
#define dumo_cmd_gatt_server_set_service_status(SERVICE,ENABLE) \
{\
bglib_temp_msg.cmd_gatt_server_set_service_status.service=(SERVICE);\
bglib_temp_msg.cmd_gatt_server_set_service_status.enable=(ENABLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0xa<<16)|((uint32)0x6<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to send data to the defined endpoint.
            {br}{br}When this command is issued data to be sent is placed to send queue for sending, except for SPI endpoints. 
            In case of SPI endpoints data is immediately sent over the interface and transfer is complete once 
            the response to this command has been received.
            **/
#define dumo_cmd_endpoint_send(ENDPOINT,DATA_LEN,DATA_DATA) \
{\
bglib_temp_msg.cmd_endpoint_send.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_endpoint_send.data.len=(DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+bglib_temp_msg.cmd_endpoint_send.data.len)>>8)))|((((uint32)2+bglib_temp_msg.cmd_endpoint_send.data.len)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x0<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_endpoint_send.data.len,(uint8*)DATA_DATA);\
}
/**This command can be used to set the destination into which data from an endpoint will be routed to.**/
#define dumo_cmd_endpoint_set_streaming_destination(ENDPOINT,DESTINATION_ENDPOINT) \
{\
bglib_temp_msg.cmd_endpoint_set_streaming_destination.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_endpoint_set_streaming_destination.destination_endpoint=(DESTINATION_ENDPOINT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to close an RFCOMM endpoint
            , LE cable replacement endpoint, or a BLE connection. 
            This command must always be used to close an endpoint with WAIT_CLOSE status
             when the remote side has closed the RFCOMM or LE cable replacement connection.
              This is to free the memory allocated by the endpoint for future reuse in the case 
              when the remote side closes the connection.{br}{br}
              Note : this command may not be issued inside {a href="#evt_endpoint_data"}
            endpoint_data{/a} event handler in BgScript.
              **/
#define dumo_cmd_endpoint_close(ENDPOINT) \
{\
bglib_temp_msg.cmd_endpoint_close.endpoint=(ENDPOINT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x2<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set endpoint flags to control and/or indicate in which mode the endpoint connection is operating.**/
#define dumo_cmd_endpoint_set_flags(ENDPOINT,FLAGS) \
{\
bglib_temp_msg.cmd_endpoint_set_flags.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_endpoint_set_flags.flags=(FLAGS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x3<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to clear endpoint flags.**/
#define dumo_cmd_endpoint_clr_flags(ENDPOINT,FLAGS) \
{\
bglib_temp_msg.cmd_endpoint_clr_flags.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_endpoint_clr_flags.flags=(FLAGS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x4<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the data performance counters (data sent counter and data received counter) of an endpoint.**/
#define dumo_cmd_endpoint_read_counters(ENDPOINT) \
{\
bglib_temp_msg.cmd_endpoint_read_counters.endpoint=(ENDPOINT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x5<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the endpoint active or inactive.
			When the endpoint is inactive, any incoming data will be paused, and no data may be sent out.
			Not all endpoint types can set changed inactive. Currently only rfcomm endpoints can be set inactive.**/
#define dumo_cmd_endpoint_set_active(ENDPOINT,ACTIVE) \
{\
bglib_temp_msg.cmd_endpoint_set_active.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_endpoint_set_active.active=(ACTIVE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xb<<16)|((uint32)0x6<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start a software timer. Multiple concurrent timers can be running simultaneously. There are 256 unique timer ID's available,
            but in practice the amount of concurrent timers is limited by the amount of free memory.**/
#define dumo_cmd_hardware_set_soft_timer(TIME,TIMER_ID,SINGLE_SHOT) \
{\
bglib_temp_msg.cmd_hardware_set_soft_timer.time=(TIME);\
bglib_temp_msg.cmd_hardware_set_soft_timer.timer_id=(TIMER_ID);\
bglib_temp_msg.cmd_hardware_set_soft_timer.single_shot=(SINGLE_SHOT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+0)>>8)))|((((uint32)6+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x0<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to configure the mode of an I/O port. After a boot, the module uses the default settings defined in hardware.xml, and this command can used to override the default settings. Note that GPIO configurations set with this command do not persist over a reset. The hardware_gpio_output mode configuration sets the output pins in the push-pull mode.**/
#define dumo_cmd_hardware_configure_gpio(PORT,GPIO,MODE,PULLUP) \
{\
bglib_temp_msg.cmd_hardware_configure_gpio.port=(PORT);\
bglib_temp_msg.cmd_hardware_configure_gpio.gpio=(GPIO);\
bglib_temp_msg.cmd_hardware_configure_gpio.mode=(MODE);\
bglib_temp_msg.cmd_hardware_configure_gpio.pullup=(PULLUP);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x1<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set the logic states of pins of the specified I/O-port using a bitmask. Correct usage of this command requires output pins configuration with the {a href="#cmd_hardware_configure_gpio"}cmd_hardware_configure_gpio{/a} (or port configuration in hardware configuration file) at first.**/
#define dumo_cmd_hardware_write_gpio(PORT,MASK,DATA) \
{\
bglib_temp_msg.cmd_hardware_write_gpio.port=(PORT);\
bglib_temp_msg.cmd_hardware_write_gpio.mask=(MASK);\
bglib_temp_msg.cmd_hardware_write_gpio.data=(DATA);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x2<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the pins of the specified I/O-port of the module.**/
#define dumo_cmd_hardware_read_gpio(PORT,MASK) \
{\
bglib_temp_msg.cmd_hardware_read_gpio.port=(PORT);\
bglib_temp_msg.cmd_hardware_read_gpio.mask=(MASK);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x3<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the specified channel of the A/D converter in the module. Note that the ADC channels must be configured
            in the hardware.xml file, as described in the module Configuration Guide. Note that only channels 4 through 7 are connected, Data read from channels 0 to 3 will be undefined.{br}
            ADC value of 4095 corresponds to the input being at Vdd, value of which can be obtained with 
            {a href="#cmd_hardware_read_vdd"}read vdd command{/a}
            **/
#define dumo_cmd_hardware_read_adc(INPUT) \
{\
bglib_temp_msg.cmd_hardware_read_adc.input=(INPUT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x4<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read from the specified I2C interface. The I2C interfaces must be configured in the hardware.xml file, as described in the
            module Configuration Guide.**/
#define dumo_cmd_hardware_read_i2c(CHANNEL,SLAVE_ADDRESS,LENGTH) \
{\
bglib_temp_msg.cmd_hardware_read_i2c.channel=(CHANNEL);\
bglib_temp_msg.cmd_hardware_read_i2c.slave_address=(SLAVE_ADDRESS);\
bglib_temp_msg.cmd_hardware_read_i2c.length=(LENGTH);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x5<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to write data into I2C interface. The I2C interfaces must be configured in the hardware.xml file, as described in the
            module Configuration Guide.**/
#define dumo_cmd_hardware_write_i2c(CHANNEL,SLAVE_ADDRESS,DATA_LEN,DATA_DATA) \
{\
bglib_temp_msg.cmd_hardware_write_i2c.channel=(CHANNEL);\
bglib_temp_msg.cmd_hardware_write_i2c.slave_address=(SLAVE_ADDRESS);\
bglib_temp_msg.cmd_hardware_write_i2c.data.len=(DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+bglib_temp_msg.cmd_hardware_write_i2c.data.len)>>8)))|((((uint32)4+bglib_temp_msg.cmd_hardware_write_i2c.data.len)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x6<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_hardware_write_i2c.data.len,(uint8*)DATA_DATA);\
}
/**This command can be used to stop I2C transmission.**/
#define dumo_cmd_hardware_stop_i2c(CHANNEL) \
{\
bglib_temp_msg.cmd_hardware_stop_i2c.channel=(CHANNEL);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x7<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to reconfigure the UART interface. The new configuration will become effective after a 100 ms delay from issuing the command. The response to the command will be sent before the 100 ms delay has elapsed using the original UART parameters. Any new configuration parameter is lost after reset and the firmware default parameters will be used again.**/
#define dumo_cmd_hardware_set_uart_configuration(ENDPOINT,RATE,DATA_BITS,STOP_BITS,PARITY,FLOW_CTRL) \
{\
bglib_temp_msg.cmd_hardware_set_uart_configuration.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_hardware_set_uart_configuration.rate=(RATE);\
bglib_temp_msg.cmd_hardware_set_uart_configuration.data_bits=(DATA_BITS);\
bglib_temp_msg.cmd_hardware_set_uart_configuration.stop_bits=(STOP_BITS);\
bglib_temp_msg.cmd_hardware_set_uart_configuration.parity=(PARITY);\
bglib_temp_msg.cmd_hardware_set_uart_configuration.flow_ctrl=(FLOW_CTRL);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)9+0)>>8)))|((((uint32)9+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x8<<24);\
bglib_output(4+9,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the voltage level of the Vdd pin. This command is active only if the ADC is active. For more information see the Hardware Configuration Guide.**/
#define dumo_cmd_hardware_read_vdd() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0x9<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the junction temperature of the module's MCU. 
            Command is active only if the ADC is active. For more information, see the Hardware
             Configuration Guide.
            **/
#define dumo_cmd_hardware_read_junction_temperature() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0xa<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to read from and write to the specified SPI interface. 
            The SPI interfaces must be configured in the hardware.xml file, as described in the 
            module Configuration Guide. {br}
			SPI slave is working in the blocking mode, waiting for CS signal and clock from master.**/
#define dumo_cmd_hardware_read_write_spi(CHANNEL,CS_PORT,CS_PIN,CS_POLARITY,READ_LENGTH,DATA_LEN,DATA_DATA) \
{\
bglib_temp_msg.cmd_hardware_read_write_spi.channel=(CHANNEL);\
bglib_temp_msg.cmd_hardware_read_write_spi.cs_port=(CS_PORT);\
bglib_temp_msg.cmd_hardware_read_write_spi.cs_pin=(CS_PIN);\
bglib_temp_msg.cmd_hardware_read_write_spi.cs_polarity=(CS_POLARITY);\
bglib_temp_msg.cmd_hardware_read_write_spi.read_length=(READ_LENGTH);\
bglib_temp_msg.cmd_hardware_read_write_spi.data.len=(DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+bglib_temp_msg.cmd_hardware_read_write_spi.data.len)>>8)))|((((uint32)6+bglib_temp_msg.cmd_hardware_read_write_spi.data.len)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0xb<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_hardware_read_write_spi.data.len,(uint8*)DATA_DATA);\
}
/**This command can be used to switch the UART interface between BGAPI and DATA mode. In BGAPI mode all data frames defined in BGAPI protocol specification are processed (and also generated) by the firmware. In DATA mode all UART's data are routed "from" and "to" selected endpoint as RAW data.**/
#define dumo_cmd_hardware_set_uart_bgapi_mode(BGAPI_MODE) \
{\
bglib_temp_msg.cmd_hardware_set_uart_bgapi_mode.bgapi_mode=(BGAPI_MODE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xc<<16)|((uint32)0xc<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to retrieve all PS keys and their current values. For each existing PS key a flash_pskey event will be generated which includes the corresponding PS key value.**/
#define dumo_cmd_flash_ps_dump() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xd<<16)|((uint32)0x0<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to erase all PS keys and their corresponding value.**/
#define dumo_cmd_flash_ps_erase_all() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xd<<16)|((uint32)0x1<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to store a value into the specified PS key.
            The allowed PS key range is from 0x4000 to 0x407F.
            Maximum length of value  is 250 bytes. 
            Maximum storage space is 4 KB and this is shared with system internal keys.
            Available storage space depends on how much storage space the application will use. In addition 20 bytes of metadata is stored for each PS item.
            **/
#define dumo_cmd_flash_ps_save(KEY,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_flash_ps_save.key=(KEY);\
bglib_temp_msg.cmd_flash_ps_save.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+bglib_temp_msg.cmd_flash_ps_save.value.len)>>8)))|((((uint32)3+bglib_temp_msg.cmd_flash_ps_save.value.len)&0xff)<<8)|((uint32)0xd<<16)|((uint32)0x2<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_flash_ps_save.value.len,(uint8*)VALUE_DATA);\
}
/**This command can be used for retrieving the value of the specified PS key.**/
#define dumo_cmd_flash_ps_load(KEY) \
{\
bglib_temp_msg.cmd_flash_ps_load.key=(KEY);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xd<<16)|((uint32)0x3<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to erase a single PS key and its value from the Persistent Store.**/
#define dumo_cmd_flash_ps_erase(KEY) \
{\
bglib_temp_msg.cmd_flash_ps_erase.key=(KEY);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xd<<16)|((uint32)0x4<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start the Bluetooth LE Direct Test Mode TX test. The module will send continuous BLE packets at the highest output power for BLE. 
            It is possible to change the RF TX power in DTM transmissions using {a href="#cmd_le_gap_set_max_power"}dumo_cmd_le_gap_set_max_power{/a} command before starting the test mode.**/
#define dumo_cmd_test_dtm_tx(PACKET_TYPE,LENGTH,CHANNEL,NUMBER_OF_PACKETS) \
{\
bglib_temp_msg.cmd_test_dtm_tx.packet_type=(PACKET_TYPE);\
bglib_temp_msg.cmd_test_dtm_tx.length=(LENGTH);\
bglib_temp_msg.cmd_test_dtm_tx.channel=(CHANNEL);\
bglib_temp_msg.cmd_test_dtm_tx.number_of_packets=(NUMBER_OF_PACKETS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x0<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start the Bluetooth LE Direct Test Mode (DTM) RX test. After RX test termination with {a href="#cmd_test_dtm_end"}dumo_cmd_test_dtm_end{/a}, event {a href="#evt_test_dtm_completed"}dumo_evt_test_dtm_completed{/a} (with number of packets received) will be generated.**/
#define dumo_cmd_test_dtm_rx(CHANNEL) \
{\
bglib_temp_msg.cmd_test_dtm_rx.channel=(CHANNEL);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x1<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to end the Bluetooth LE Direct Test Mode (DTM) mode.**/
#define dumo_cmd_test_dtm_end() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x2<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start a continuous transmission TX test. For Continuous Wave modulation, the data is all zeros, for all other modulations
            the data is PN15 pseudo - random noise. No Bluetooth - formatted packets are sent in this mode.**/
#define dumo_cmd_test_tx_test(MODULATION,CHANNEL,POWER) \
{\
bglib_temp_msg.cmd_test_tx_test.modulation=(MODULATION);\
bglib_temp_msg.cmd_test_tx_test.channel=(CHANNEL);\
bglib_temp_msg.cmd_test_tx_test.power=(POWER);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x3<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**Enable debug messages**/
#define dumo_cmd_test_debug_enable(ID,ENABLE) \
{\
bglib_temp_msg.cmd_test_debug_enable.id=(ID);\
bglib_temp_msg.cmd_test_debug_enable.enable=(ENABLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x4<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/****/
#define dumo_cmd_test_enable_uart_loopback_mode() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x5<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to enable or disable using a pre-defined Diffie-Hellman key pair for generating Bluetooth BR/EDR Secure Simple Pairing
            link keys. When the debug mode is enabled, a Bluetooth sniffer can decrypt the communication between two Bluetooth devices using Secure Simple Pairing. It is 
            sufficient that one party uses the debug mode - it is not necessary for both parties to use it.**/
#define dumo_cmd_test_ssp_debug(ENABLE) \
{\
bglib_temp_msg.cmd_test_ssp_debug.enable=(ENABLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x6<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**debug_command**/
#define dumo_cmd_test_debug_command(ID,DEBUGDATA_LEN,DEBUGDATA_DATA) \
{\
bglib_temp_msg.cmd_test_debug_command.id=(ID);\
bglib_temp_msg.cmd_test_debug_command.debugdata.len=(DEBUGDATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+bglib_temp_msg.cmd_test_debug_command.debugdata.len)>>8)))|((((uint32)2+bglib_temp_msg.cmd_test_debug_command.debugdata.len)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x7<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_test_debug_command.debugdata.len,(uint8*)DEBUGDATA_DATA);\
}
/** This command can used to set BT121 in "BT RF SIG mode" which is meant for connecting with a Bluetooth tester where the BT121 is controlled over the LMP (Link Management Protocol). Command makes the BT121 module visible, connectable, acceptable all connections and put them in test mode. Then the Bluetooth tester can take control of the BT121 module through the RF link. For more details see Bluetooth Core Specification 4.1 (Volume 2 Part E, chapter 7.6.3).
           **/
#define dumo_cmd_test_device_under_test_mode() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x8<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start the RX test mode with continuous reception. It can be useful for current consumption measurements 
            or marking the level of unwanted emissions. **/
#define dumo_cmd_test_rx_test(CHANNEL) \
{\
bglib_temp_msg.cmd_test_rx_test.channel=(CHANNEL);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0x9<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to start the packet testing mode for Bluetooth BR/EDR.**/
#define dumo_cmd_test_packet_test(MODE,TX_FREQ,RX_FREQ,ACL_TYPE,ACL_LEN,POWER,DISABLE_WHITENING) \
{\
bglib_temp_msg.cmd_test_packet_test.mode=(MODE);\
bglib_temp_msg.cmd_test_packet_test.tx_freq=(TX_FREQ);\
bglib_temp_msg.cmd_test_packet_test.rx_freq=(RX_FREQ);\
bglib_temp_msg.cmd_test_packet_test.acl_type=(ACL_TYPE);\
bglib_temp_msg.cmd_test_packet_test.acl_len=(ACL_LEN);\
bglib_temp_msg.cmd_test_packet_test.power=(POWER);\
bglib_temp_msg.cmd_test_packet_test.disable_whitening=(DISABLE_WHITENING);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)8+0)>>8)))|((((uint32)8+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0xa<<24);\
bglib_output(4+8,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**debug_counter**/
#define dumo_cmd_test_debug_counter(ID) \
{\
bglib_temp_msg.cmd_test_debug_counter.id=(ID);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0xe<<16)|((uint32)0xb<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set whether the device accepts new bondings or not.**/
#define dumo_cmd_sm_set_bondable_mode(BONDABLE) \
{\
bglib_temp_msg.cmd_sm_set_bondable_mode.bondable=(BONDABLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x0<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to configure authentication methods and I/O capabilities of the system.**/
#define dumo_cmd_sm_configure(MITM_REQUIRED,IO_CAPABILITIES) \
{\
bglib_temp_msg.cmd_sm_configure.mitm_required=(MITM_REQUIRED);\
bglib_temp_msg.cmd_sm_configure.io_capabilities=(IO_CAPABILITIES);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to set maximum allowed bonding count and bonding policy.**/
#define dumo_cmd_sm_store_bonding_configuration(MAX_BONDING_COUNT,POLICY_FLAGS) \
{\
bglib_temp_msg.cmd_sm_store_bonding_configuration.max_bonding_count=(MAX_BONDING_COUNT);\
bglib_temp_msg.cmd_sm_store_bonding_configuration.policy_flags=(POLICY_FLAGS);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x2<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the maximum number of allowed bonding entries and to reveal the currently set bonding policy.**/
#define dumo_cmd_sm_read_bonding_configuration() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x3<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to enhance the security of a Bluetooth BR/EDR or Bluetooth LE connection to current security requirements.
            By default, connections opened by the module are not encrypted, unless the remote device requests it (most devices do). For Bluetooth LE connections,
            this command initiates encryption, as there are only two levels of security: unencrypted and encrypted. If the devices are bonded, the existing bonding
            will be used. If the devices are not bonded, a new key will be created; if bonding is enabled the key will be stored for future use, if not the key will
            be discarded after the connection is closed. For Bluetooth BR/EDR connections, in some cases it's possible to first increase the security to encryption 
            with a Just Works key, then increase it again by creating a MITM protected key.
            **/
#define dumo_cmd_sm_increase_security(CONNECTION) \
{\
bglib_temp_msg.cmd_sm_increase_security.connection=(CONNECTION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x4<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to read the encryption key for a specific bonding. Used in debugging for reading encryption keys which can be used 
            e.g. for protocol sniffing.**/
#define dumo_cmd_sm_read_bonding(BONDING) \
{\
bglib_temp_msg.cmd_sm_read_bonding.bonding=(BONDING);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x5<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to delete specified bonding information from Persistent Store.**/
#define dumo_cmd_sm_delete_bonding(BONDING) \
{\
bglib_temp_msg.cmd_sm_delete_bonding.bonding=(BONDING);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x6<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to delete all bonding information from Persistent Store.**/
#define dumo_cmd_sm_delete_bondings() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x7<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to enter a passkey after receiving a passkey request event.**/
#define dumo_cmd_sm_enter_passkey(CONNECTION,PASSKEY) \
{\
bglib_temp_msg.cmd_sm_enter_passkey.connection=(CONNECTION);\
bglib_temp_msg.cmd_sm_enter_passkey.passkey=(PASSKEY);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+0)>>8)))|((((uint32)5+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x8<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used for accepting or rejecting reported confirm value.**/
#define dumo_cmd_sm_passkey_confirm(CONNECTION,CONFIRM) \
{\
bglib_temp_msg.cmd_sm_passkey_confirm.connection=(CONNECTION);\
bglib_temp_msg.cmd_sm_passkey_confirm.confirm=(CONFIRM);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x9<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
                This command can be used to set the OOB data (out-of-band encryption data) for a device. The OOB data may be, for example, a PIN code exchanged over 
                an alternate path like NFC. The device will not allow any other kind of bonding if OOB data is set.
                {br}{br}
                This bonding method is available only for LE bonding.
            **/
#define dumo_cmd_sm_set_oob_data(OOB_DATA_LEN,OOB_DATA_DATA) \
{\
bglib_temp_msg.cmd_sm_set_oob_data.oob_data.len=(OOB_DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_sm_set_oob_data.oob_data.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_sm_set_oob_data.oob_data.len)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0xa<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_sm_set_oob_data.oob_data.len,(uint8*)OOB_DATA_DATA);\
}
/**This command can be used to list all bondings stored in the bonding database. Bondings are reported by using the {a href="#evt_sm_list_bonding_entry"}
            sm_list_bonding_entry{/a} event for each bonding and the report is ended with {a href="#evt_sm_list_all_bondings_complete"}sm_list_all_bondings_complete{/a} event. 
            Recommended to be used only for debugging purposes, because reading from the Persistent Store is relatively slow.**/
#define dumo_cmd_sm_list_all_bondings() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0xb<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to confirm bonding when sm_bonding_request event is received.**/
#define dumo_cmd_sm_confirm_bonding() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0xc<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command can be used to enter a PIN code after receiving a PIN code request event.**/
#define dumo_cmd_sm_enter_pin_code(ADDRESS,PIN_CODE_LEN,PIN_CODE_DATA) \
{\
memcpy(&bglib_temp_msg.cmd_sm_enter_pin_code.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_sm_enter_pin_code.pin_code.len=(PIN_CODE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)7+bglib_temp_msg.cmd_sm_enter_pin_code.pin_code.len)>>8)))|((((uint32)7+bglib_temp_msg.cmd_sm_enter_pin_code.pin_code.len)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0xd<<24);\
bglib_output(4+7,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_sm_enter_pin_code.pin_code.len,(uint8*)PIN_CODE_DATA);\
}
/**This command can be used to prevent LE bondings event though bondings are allowed.**/
#define dumo_cmd_sm_set_le_bondable_mode(BONDABLE) \
{\
bglib_temp_msg.cmd_sm_set_le_bondable_mode.bondable=(BONDABLE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0xe<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command will configure the LE to send only encryption key.**/
#define dumo_cmd_sm_set_le_key_enc_only(ENABLED) \
{\
bglib_temp_msg.cmd_sm_set_le_key_enc_only.enabled=(ENABLED);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0xf<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/** This command can be used to set the minimum allowed encryption key size value used for bonding. The default value is 7 bytes. If negotiated value of the key size is less than expected minimum after connection establishment, the credential bonding process is considered invalid and {a href="#evt_sm_bonding_failed"}dumo_evt_sm_bonding_failed{/a} event with {a href="#error_codes"}dumo_err_sm_encryption_key_size (0x0306){/a} code is generated. **/
#define dumo_cmd_sm_set_minimum_key_size(MINIMUM_KEY_SIZE) \
{\
bglib_temp_msg.cmd_sm_set_minimum_key_size.minimum_key_size=(MINIMUM_KEY_SIZE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0xf<<16)|((uint32)0x10<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command is used to open an iAP connection to an iOS device. Connection can be closed by endpoint_close command.**/
#define dumo_cmd_iap_open(ADDRESS,STREAMING_DESTINATION) \
{\
memcpy(&bglib_temp_msg.cmd_iap_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_iap_open.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)7+0)>>8)))|((((uint32)7+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x0<<24);\
bglib_output(4+7,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command is used to start the iAP server.
            **/
#define dumo_cmd_iap_start_server(SDP_ID,STREAMING_DESTINATION) \
{\
bglib_temp_msg.cmd_iap_start_server.sdp_id=(SDP_ID);\
bglib_temp_msg.cmd_iap_start_server.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command  is used to set iAP configuration parameters.
            **/
#define dumo_cmd_iap_set_param(INDEX,FLAGS,VALUE_LEN,VALUE_DATA) \
{\
bglib_temp_msg.cmd_iap_set_param.index=(INDEX);\
bglib_temp_msg.cmd_iap_set_param.flags=(FLAGS);\
bglib_temp_msg.cmd_iap_set_param.value.len=(VALUE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)6+bglib_temp_msg.cmd_iap_set_param.value.len)>>8)))|((((uint32)6+bglib_temp_msg.cmd_iap_set_param.value.len)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x2<<24);\
bglib_output(4+6,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_iap_set_param.value.len,(uint8*)VALUE_DATA);\
}
/**
            This command  is used to set the desired protocol for iAP communication.
            **/
#define dumo_cmd_iap_set_protocol(INDEX,MATCH_ACTION,NAME_LEN,NAME_DATA) \
{\
bglib_temp_msg.cmd_iap_set_protocol.index=(INDEX);\
bglib_temp_msg.cmd_iap_set_protocol.match_action=(MATCH_ACTION);\
bglib_temp_msg.cmd_iap_set_protocol.name.len=(NAME_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+bglib_temp_msg.cmd_iap_set_protocol.name.len)>>8)))|((((uint32)3+bglib_temp_msg.cmd_iap_set_protocol.name.len)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x3<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_iap_set_protocol.name.len,(uint8*)NAME_DATA);\
}
/**
            This command  is used for reading the value of the specified parameter.
            **/
#define dumo_cmd_iap_get_param(INDEX) \
{\
bglib_temp_msg.cmd_iap_get_param.index=(INDEX);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x4<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command  is used to read the protocol configuration.
            **/
#define dumo_cmd_iap_get_protocol(INDEX) \
{\
bglib_temp_msg.cmd_iap_get_protocol.index=(INDEX);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x5<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
           This command can be used to dump all iAP parameters.
            **/
#define dumo_cmd_iap_dump() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x6<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command is used for checking the validity of the current iAP configuration.
            **/
#define dumo_cmd_iap_validate() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x7<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command  is used to clear all iAP parameter and protocol settings.
            **/
#define dumo_cmd_iap_erase() \
{\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)0+0)>>8)))|((((uint32)0+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x8<<24);\
bglib_output(4+0,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
                This command is used to launch an app on an iOS device.
            **/
#define dumo_cmd_iap_launch_app(ENDPOINT,LAUNCH_ALERT,BUNDLE_ID_LEN,BUNDLE_ID_DATA) \
{\
bglib_temp_msg.cmd_iap_launch_app.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_iap_launch_app.launch_alert=(LAUNCH_ALERT);\
bglib_temp_msg.cmd_iap_launch_app.bundle_id.len=(BUNDLE_ID_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+bglib_temp_msg.cmd_iap_launch_app.bundle_id.len)>>8)))|((((uint32)3+bglib_temp_msg.cmd_iap_launch_app.bundle_id.len)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0x9<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_iap_launch_app.bundle_id.len,(uint8*)BUNDLE_ID_DATA);\
}
/**
            This command can be used to stop the iAP server
            **/
#define dumo_cmd_iap_stop_server(SDP_ID) \
{\
bglib_temp_msg.cmd_iap_stop_server.sdp_id=(SDP_ID);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x10<<16)|((uint32)0xa<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            Converts decimal value in ASCII string to 32-bit signed integer.
            **/
#define dumo_cmd_util_atoi(STRING_LEN,STRING_DATA) \
{\
bglib_temp_msg.cmd_util_atoi.string.len=(STRING_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_util_atoi.string.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_util_atoi.string.len)&0xff)<<8)|((uint32)0x11<<16)|((uint32)0x0<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_util_atoi.string.len,(uint8*)STRING_DATA);\
}
/**
            This command can be used to convert a 32-bit signed integer value into a decimal value represented as a string.
            **/
#define dumo_cmd_util_itoa(VALUE) \
{\
bglib_temp_msg.cmd_util_itoa.value=(VALUE);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)4+0)>>8)))|((((uint32)4+0)&0xff)<<8)|((uint32)0x11<<16)|((uint32)0x1<<24);\
bglib_output(4+4,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            **/
#define dumo_cmd_util_debug_print(STRING_LEN,STRING_DATA) \
{\
bglib_temp_msg.cmd_util_debug_print.string.len=(STRING_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+bglib_temp_msg.cmd_util_debug_print.string.len)>>8)))|((((uint32)1+bglib_temp_msg.cmd_util_debug_print.string.len)&0xff)<<8)|((uint32)0x11<<16)|((uint32)0x2<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_util_debug_print.string.len,(uint8*)STRING_DATA);\
}
/**
            Open L2CAP channel to remote device
            **/
#define dumo_cmd_bt_l2cap_open(ADDRESS,STREAMING_DESTINATION,PSM) \
{\
memcpy(&bglib_temp_msg.cmd_bt_l2cap_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_bt_l2cap_open.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.cmd_bt_l2cap_open.psm=(PSM);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)9+0)>>8)))|((((uint32)9+0)&0xff)<<8)|((uint32)0x12<<16)|((uint32)0x0<<24);\
bglib_output(4+9,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to receive incoming connections to the specified PSM. 
            **/
#define dumo_cmd_bt_l2cap_start_server(PSM,STREAMING_DESTINATION) \
{\
bglib_temp_msg.cmd_bt_l2cap_start_server.psm=(PSM);\
bglib_temp_msg.cmd_bt_l2cap_start_server.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x12<<16)|((uint32)0x1<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            Open HID connection to remote HID Host. Note that the HID Host must have initiated a previous connection and stored our HID SDP entry, and our SDP entry must indicate support for HID Device -initiated connections; otherwise the HID Host will reject our connection attempt.
            **/
#define dumo_cmd_bt_hid_open(ADDRESS,STREAMING_DESTINATION) \
{\
memcpy(&bglib_temp_msg.cmd_bt_hid_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_bt_hid_open.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)7+0)>>8)))|((((uint32)7+0)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x0<<24);\
bglib_output(4+7,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to start accepting HID connections.
            **/
#define dumo_cmd_bt_hid_start_server(SDP_ID,STREAMING_DESTINATION) \
{\
bglib_temp_msg.cmd_bt_hid_start_server.sdp_id=(SDP_ID);\
bglib_temp_msg.cmd_bt_hid_start_server.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x1<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
            This command can be used to stop an HID server.
            **/
#define dumo_cmd_bt_hid_stop_server(SDP_ID) \
{\
bglib_temp_msg.cmd_bt_hid_stop_server.sdp_id=(SDP_ID);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x2<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**
                 This command can be used to send HID Input Reports to the HID Host.
             **/
#define dumo_cmd_bt_hid_send_input_report(ENDPOINT,REPORT_ID,REPORT_DATA_LEN,REPORT_DATA_DATA) \
{\
bglib_temp_msg.cmd_bt_hid_send_input_report.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_hid_send_input_report.report_id=(REPORT_ID);\
bglib_temp_msg.cmd_bt_hid_send_input_report.report_data.len=(REPORT_DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+bglib_temp_msg.cmd_bt_hid_send_input_report.report_data.len)>>8)))|((((uint32)3+bglib_temp_msg.cmd_bt_hid_send_input_report.report_data.len)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x3<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_bt_hid_send_input_report.report_data.len,(uint8*)REPORT_DATA_DATA);\
}
/**
                 This command can be used to disassociate the device from the HID Host. Bonding information will be deleted and 
                 the connection disconnected automatically.
             **/
#define dumo_cmd_bt_hid_virtual_cable_unplug(ENDPOINT) \
{\
bglib_temp_msg.cmd_bt_hid_virtual_cable_unplug.endpoint=(ENDPOINT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x4<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command is used to respond to evt_bt_hid_get_report requests. The parameters endpoint, parameters_used, report_type, and report_id should match those received in the request. The report_data should contain the current instantaneous state of the fields specified in the request.**/
#define dumo_cmd_bt_hid_get_report_response(ENDPOINT,PARAMETERS_USED,REPORT_TYPE,REPORT_ID,REPORT_DATA_LEN,REPORT_DATA_DATA) \
{\
bglib_temp_msg.cmd_bt_hid_get_report_response.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_hid_get_report_response.parameters_used=(PARAMETERS_USED);\
bglib_temp_msg.cmd_bt_hid_get_report_response.report_type=(REPORT_TYPE);\
bglib_temp_msg.cmd_bt_hid_get_report_response.report_id=(REPORT_ID);\
bglib_temp_msg.cmd_bt_hid_get_report_response.report_data.len=(REPORT_DATA_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)5+bglib_temp_msg.cmd_bt_hid_get_report_response.report_data.len)>>8)))|((((uint32)5+bglib_temp_msg.cmd_bt_hid_get_report_response.report_data.len)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x5<<24);\
bglib_output(4+5,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_bt_hid_get_report_response.report_data.len,(uint8*)REPORT_DATA_DATA);\
}
/**This command is used to acknowledge an evt_bt_hid_set_report request. If the parameters are invalid, use cmd_bt_hid_get_set_report_reject to reject it.**/
#define dumo_cmd_bt_hid_set_report_response(ENDPOINT) \
{\
bglib_temp_msg.cmd_bt_hid_set_report_response.endpoint=(ENDPOINT);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)1+0)>>8)))|((((uint32)1+0)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x6<<24);\
bglib_output(4+1,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**This command is used to reject an evt_bt_hid_get_report request for an invalid Report.**/
#define dumo_cmd_bt_hid_get_set_report_reject(ENDPOINT,REASON) \
{\
bglib_temp_msg.cmd_bt_hid_get_set_report_reject.endpoint=(ENDPOINT);\
bglib_temp_msg.cmd_bt_hid_get_set_report_reject.reason=(REASON);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)2+0)>>8)))|((((uint32)2+0)&0xff)<<8)|((uint32)0x13<<16)|((uint32)0x7<<24);\
bglib_output(4+2,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
/**Open an LE cable replacement connection endpoint to a remote device. The endpoint can be closed with the {a href="#cmd_endpoint_close"}endpoint_close{/a} command when done.
         
         If no ACL connection to the remote device exists, one is formed before the cable replacement connection handshake. Otherwise the existing ACL connection will be used.
         
         Note that if the remote device requires encryption or authentication for communication, ACL connection security must be set up before opening the LE cable replacement connection; see {a href="#cmd_sm_increase_security"}sm_increase_security{/a} command**/
#define dumo_cmd_le_serial_open(ADDRESS,ADDRESS_TYPE,STREAMING_DESTINATION,SERVICE_LEN,SERVICE_DATA) \
{\
memcpy(&bglib_temp_msg.cmd_le_serial_open.address,(ADDRESS),sizeof(hw_addr));\
bglib_temp_msg.cmd_le_serial_open.address_type=(ADDRESS_TYPE);\
bglib_temp_msg.cmd_le_serial_open.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.cmd_le_serial_open.service.len=(SERVICE_LEN);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)9+bglib_temp_msg.cmd_le_serial_open.service.len)>>8)))|((((uint32)9+bglib_temp_msg.cmd_le_serial_open.service.len)&0xff)<<8)|((uint32)0x14<<16)|((uint32)0x0<<24);\
bglib_output(4+9,(uint8*)&bglib_temp_msg,bglib_temp_msg.cmd_le_serial_open.service.len,(uint8*)SERVICE_DATA);\
}
/**
              Listen for an incoming LE cable replacement connection. Creates an endpoint which will be set as active when an incoming connection completes. Note that once the cable replacement connection is closed by the remote side, the endpoint is not usable anymore; it must be cleaned up using {a href="#cmd_endpoint_close"}endpoint_close{/a} command and listen must be called again to handle the next incoming connection.
              Also note that a listening endpoint may be closed only when not yet connected or already disconnected by the remote. Only the side initiating the connection may close the endpoint while it is connected.
            **/
#define dumo_cmd_le_serial_listen(CHARACTERISTIC,STREAMING_DESTINATION) \
{\
bglib_temp_msg.cmd_le_serial_listen.characteristic=(CHARACTERISTIC);\
bglib_temp_msg.cmd_le_serial_listen.streaming_destination=(STREAMING_DESTINATION);\
bglib_temp_msg.header=(((uint32)dumo_msg_type_cmd|dumo_dev_type_dumo|(((uint32)3+0)>>8)))|((((uint32)3+0)&0xff)<<8)|((uint32)0x14<<16)|((uint32)0x1<<24);\
bglib_output(4+3,(uint8*)&bglib_temp_msg,0,(uint8*)NULL);\
}
 
#ifdef __cplusplus
}
#endif
/*lint -restore*/
#endif
