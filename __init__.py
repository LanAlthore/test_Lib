#
# -*- coding: utf-8 -*-
############################################################################
#
# @file    prodvklib.py
# @brief   EM9304 Production DVK Test Tool Library
#
# This is a python-based tool to perform test operations on the
# EM9304 Production Test DVK Board via a USB HID interface.
# The Production Test Board must be running the "prodvk" FW application.
#
#
#
# Copyright (c) 2017-2018 EM Microelectronic-US Inc. All rights reserved.
# Developed by Glacier River Design, LLC.
#
############################################################################
# EM Microelectronic-US Inc. License Agreement
#
# < leave blank for now >
#
############################################################################

# Imported modules
import usb.core
import usb.util
import struct
import logging
import sys
from datetime import datetime
import pdb
global printheader
printheader=""
def setprintheader(ph):
    global printheader
    printheader = ph

# Standard VID/PID for the Production Test Board
# Must be aligned with FW settings in app_usbd_cfg.h
DVK_USB_VID = 0x1fc9    # USB_VENDOR_ID
DVK_USB_PID = 0x0081    # USB_PRODUCT_ID

# SN for a device (as an example of the format)
DVK_SERIAL = "120D004190108CEA740927655891005F"

# USB HID parameters
DVK_USB_READ_EP  = 0x81
DVK_USB_WRITE_EP = 0x01
DVK_USB_EP_SIZE  = 64
DVK_USB_TIMEOUT  = 6000
# Indices to useful descriptor strings
# Must be kept in sync with the descriptors in the FW
DVK_USB_PRODUCT_NAME_STRING_DESCRIPTOR_INDEX = 2
DVK_USB_SERIAL_NUM_STRING_DESCRIPTOR_INDEX = 3

# ==========================================================================
# ==========================================================================
# The following constants are derived from the HID interface defined in
# the file command_if.h.  Keep these values in sync.

COMMAND_TX     = 0x00
COMMAND_RX     = 0x01
COMMAND_RESET  = 0x02
COMMAND_PTM    = 0x03        # Program Test Mode
COMMAND_TESTOP = 0x20        # Test Operation

# The Sub-commands of the TESTOP command
# First, the ProDVK board-specific commands
TESTOP_SUBCMD_READ_PRODVK_FW_VER    = (1)
TESTOP_SUBCMD_READ_PRODVK_SN        = (2)
TESTOP_SUBCMD_READ_DUT_VER          = (3)
TESTOP_SUBCMD_READ_REF_VER          = (4)
TESTOP_SUBCMD_READ_CURRENT          = (5)
TESTOP_SUBCMD_READ_ADC              = (6)
TESTOP_SUBCMD_UNUSED                = (7)
TESTOP_SUBCMD_MODESWITCH_TO_TESTOP  = (8)
TESTOP_SUBCMD_MODESWITCH_TO_BRIDGE  = (9)
TESTOP_SUBCMD_READ_STATUS           =(10)
TESTOP_SUBCMD_SET_CURRENT_RANGE_1   =(11)
TESTOP_SUBCMD_SET_CURRENT_RANGE_2   =(12)
TESTOP_SUBCMD_SET_CURRENT_RANGE_3   =(13)
TESTOP_SUBCMD_RESET_9304            =(14)
TESTOP_SUBCMD_SET_REF_CLOCK         =(15)
TESTOP_SUBCMD_EXEC_XTALVALIDATION   =(16)
TESTOP_SUBCMD_SET_MUX_STATE         =(17)
TESTOP_SUBCMD_EXEC_CALIBRATION      =(18)

# Standard HCI and LE HCI commands
# For clarity, include "HCI_LE_" in all the LE-specific commands
TESTOP_SUBCMD_HCI_READ_9304_VER             =  (0x13)    # HCI_Read_Local_Version_Information
TESTOP_SUBCMD_HCI_LE_RECEIVER_TEST          =  (0x14)    # HCI_LE_Receiver_Test
TESTOP_SUBCMD_HCI_LE_TRANSMITTER_TEST       =  (0x15)    # HCI_LE_Transmitter_Test
TESTOP_SUBCMD_HCI_LE_TEST_END               =  (0x16)    # HCI_LE_Test_End
TESTOP_SUBCMD_HCI_RESET                     =  (0x17)    # HCI_Reset
TESTOP_SUBCMD_HCI_READ_BD_ADDR              =  (0x18)
TESTOP_SUBCMD_HCI_SET_ADVERTISING_DATA      =  (0x19)
TESTOP_SUBCMD_HCI_SET_ADVERTISING_PARAMETERS = (0x1A)
TESTOP_SUBCMD_HCI_LE_SET_ADVERTISE_ENABLE   =  (0x1B)
TESTOP_SUBCMD_HCI_LE_CLEAR_WHITE_LIST       =  (0x1C)
TESTOP_SUBCMD_HCI_LE_ADD_DEVICE_TO_WHITE_LIST = (0x1D)
TESTOP_SUBCMD_HCI_LE_SET_SCAN_PARAMETERS    =  (0x1E)
TESTOP_SUBCMD_HCI_LE_SET_SCAN_ENABLE        =  (0x1F)
TESTOP_SUBCMD_HCI_LE_GET_ADVERTISING_REPORT =  (0x20)

# High level functional test commands
TESTOP_SUBCMD_FUNCTEST_CURRENT_SLEEP		=	(0x21)
TESTOP_SUBCMD_FUNCTEST_CURRENT_ACTIVE		=	(0x22)
TESTOP_SUBCMD_FUNCTEST_CURRENT_RX			=	(0x23)
TESTOP_SUBCMD_FUNCTEST_CURRENT_TX			=	(0x24)
TESTOP_SUBCMD_FUNCTEST_PER_TX				=	(0x25)
TESTOP_SUBCMD_FUNCTEST_PER_RX				=	(0x26)
TESTOP_SUBCMD_FUNCTEST_ADVERTISE			=	(0x27)
TESTOP_SUBCMD_FUNCTEST_RSSI					=	(0x28)
TESTOP_SUBCMD_FUNCTEST_XTAL					=	(0x29)
TESTOP_SUBCMD_FUNCTEST_PWR_MODE				=	(0x2A)
TESTOP_SUBCMD_FUNCTEST_SVLD 				=	(0x2B)
TESTOP_SUBCMD_FUNCTEST_READ_RESULTS			=	(0x2C)

# GPIO control commands
TESTOP_SUBCMD_GPIO_CONFIGURE_IO     		=   (0x30)
TESTOP_SUBCMD_GPIO_SET_IO           		=   (0x31)
TESTOP_SUBCMD_GPIO_READ_DIGITAL_IO     	    =	(0x32)
TESTOP_SUBCMD_GPIO_READ_ANALOG_IO      		=	(0x33)
TESTOP_SUBCMD_GPIO_DISABLE_IO_SET      		=	(0x34)

TESTOP_SUBCMD_MEASURE_TRIGGERED_CURRENT		=	(0x35)
TESTOP_SUBCMD_WRITE_DAC_LTC2633     		=	(0x36)
TESTOP_SUBCMD_READ_ADC_MAX11614EEE          =   (0x37)
TESTOP_SUBCMD_RESET_ADC_MAX11614EEE         =   (0x38)
TESTOP_SUBCMD_UPLOAD_TO_9304                =   (0x39)
TESTOP_SUBCMD_RESET_PRODVK                  =   (0x3A)
TESTOP_SUBCMD_READ_CRC                      =   (0x3B)

# For the EM vendor-specific HCI commands, OR the opcode with 0x40 to
# convert to a TestOp subcommand
# All EM command names should have "HCI_EM_" in them.
HCIEM_CMD_BASE                           = 0x40
TESTOP_SUBCMD_HCI_EM_SET_PUBLIC_ADDRESS     = (HCIEM_CMD_BASE + 0x02)
TESTOP_SUBCMD_HCI_EM_SET_UART_BAUD_RATE     = (HCIEM_CMD_BASE + 0x07)
TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST       = (HCIEM_CMD_BASE + 0x11)
TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST_END   = (HCIEM_CMD_BASE + 0x12)
TESTOP_SUBCMD_HCI_EM_READ_AT_ADDRESS        = (HCIEM_CMD_BASE + 0x20)
TESTOP_SUBCMD_HCI_EM_READ_CONTINUE          = (HCIEM_CMD_BASE + 0x21)
TESTOP_SUBCMD_HCI_EM_WRITE_AT_ADDRESS       = (HCIEM_CMD_BASE + 0x22)
TESTOP_SUBCMD_HCI_EM_WRITE_CONTINUE         = (HCIEM_CMD_BASE + 0x23)
TESTOP_SUBCMD_HCI_EM_SET_POWER_MODE_EX      = (HCIEM_CMD_BASE + 0x24)
TESTOP_SUBCMD_HCI_EM_SET_RF_ACTIVITY_SIGNAL_EX=(HCIEM_CMD_BASE+ 0x25)
TESTOP_SUBCMD_HCI_EM_SET_RF_POWER_LEVEL_EX  = (HCIEM_CMD_BASE + 0x26)
TESTOP_SUBCMD_HCI_EM_WRITE_PATCH_START      = (HCIEM_CMD_BASE + 0x27)
TESTOP_SUBCMD_HCI_EM_WRITE_PATCH_CONTINUE   = (HCIEM_CMD_BASE + 0x28)
TESTOP_SUBCMD_HCI_EM_WRITE_PATCH_ABORT      = (HCIEM_CMD_BASE + 0x29)
TESTOP_SUBCMD_HCI_EM_SET_CLOCK_SOURCE       = (HCIEM_CMD_BASE + 0x2A)
TESTOP_SUBCMD_HCI_EM_SET_MEMORY_MODE        = (HCIEM_CMD_BASE + 0x2B)
TESTOP_SUBCMD_HCI_EM_GET_MEMORY_USAGE       = (HCIEM_CMD_BASE + 0x2C)
TESTOP_SUBCMD_HCI_EM_SET_SLEEP_OPTIONS      = (HCIEM_CMD_BASE + 0x2D)
TESTOP_SUBCMD_HCI_EM_SVLD_MEASUREMENT       = (HCIEM_CMD_BASE + 0x2E)
TESTOP_SUBCMD_HCI_EM_SET_EVENT_MASK         = (HCIEM_CMD_BASE + 0x2F)
TESTOP_SUBCMD_HCI_EM_CPU_RESET              = (HCIEM_CMD_BASE + 0x32)
TESTOP_SUBCMD_HCI_EM_CALCULATE_CRC32_EX     = (HCIEM_CMD_BASE + 0x33)
TESTOP_SUBCMD_HCI_EM_PATCH_QUERY            = (HCIEM_CMD_BASE + 0x34)

# Table lookup for subcmd names, starting at index 0x40 (the EM vendor-specific commands)
# TODO: cover the full set of subcmd names.
subcmd_names      = [
    'unused',
    'unused',
    'TESTOP_SUBCMD_HCI_EM_SET_PUBLIC_ADDRESS',  # 0x02
    'unused',
    'unused',
    'unused',
    'unused',
    'TESTOP_SUBCMD_HCI_EM_SET_UART_BAUD_RATE',  # 0x07
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST',      # 0x11
    'TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST_END',  # 0x12
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'TESTOP_SUBCMD_HCI_EM_READ_AT_ADDRESS',
    'TESTOP_SUBCMD_HCI_EM_READ_CONTINUE',
    'TESTOP_SUBCMD_HCI_EM_WRITE_AT_ADDRESS',
    'TESTOP_SUBCMD_HCI_EM_WRITE_CONTINUE',
    'TESTOP_SUBCMD_HCI_EM_SET_POWER_MODE_EX',
    'TESTOP_SUBCMD_HCI_EM_SET_RF_ACTIVITY_SIGNAL_EX',
    'TESTOP_SUBCMD_HCI_EM_SET_RF_POWER_LEVEL_EX',
    'TESTOP_SUBCMD_HCI_EM_WRITE_PATCH_START' ,
    'TESTOP_SUBCMD_HCI_EM_WRITE_PATCH_CONTINUE',
    'TESTOP_SUBCMD_HCI_EM_WRITE_PATCH_ABORT',
    'TESTOP_SUBCMD_HCI_EM_SET_CLOCK_SOURCE',
    'TESTOP_SUBCMD_HCI_EM_SET_MEMORY_MODE'  ,
    'TESTOP_SUBCMD_HCI_EM_GET_MEMORY_USAGE',
    'TESTOP_SUBCMD_HCI_EM_SET_SLEEP_OPTIONS',
    'TESTOP_SUBCMD_HCI_EM_SVLD_MEASUREMENT',
    'TESTOP_SUBCMD_HCI_EM_SET_EVENT_MASK',
    'unused',
    'unused',
    'TESTOP_SUBCMD_HCI_EM_CPU_RESET',
    'TESTOP_SUBCMD_HCI_EM_CALCULATE_CRC32_EX',
    'TESTOP_SUBCMD_HCI_EM_PATCH_QUERY',         # 0x34
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    'unused',
    # The special test commands  (starting at 0x80)
    'TESTOP_SUBCMD_HCI_PROTEST_SLEEP',
    'TESTOP_SUBCMD_HCI_PROTEST_ACTIVE',
    'TESTOP_SUBCMD_HCI_PROTEST_TXSTART',
    'TESTOP_SUBCMD_HCI_PROTEST_TXSTOP',
    'TESTOP_SUBCMD_HCI_PROTEST_RXSTART',
    'TESTOP_SUBCMD_HCI_PROTEST_RXSTOP',
    'TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_ENABLE',
    'TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_DISABLE',
    'TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_ENABLE',
    'TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_DISABLE',
    'TESTOP_SUBCMD_HCI_PROTEST_GET_SVLD',
    'TESTOP_SUBCMD_HCI_PROTEST_SET_GPIO',
    'unused',
    'unused' ]


# For the EM vendor-specific HCI commands that are implemented as
# PROTEST_SUB_ commands, use the same code but ORd with 0x80
HCITEST_CMD_BASE                         = 0x80
TESTOP_SUBCMD_HCI_PROTEST_SLEEP                  = (HCITEST_CMD_BASE + 0)  # HCI2SUBCMD(PROTEST_SUB_SLEEP)
TESTOP_SUBCMD_HCI_PROTEST_ACTIVE                 = (HCITEST_CMD_BASE + 1)  # HCI2SUBCMD(PROTEST_SUB_ACTIVE)
TESTOP_SUBCMD_HCI_PROTEST_TXSTART                = (HCITEST_CMD_BASE + 2)  # HCI2SUBCMD(PROTEST_SUB_TX_TEST_START)
TESTOP_SUBCMD_HCI_PROTEST_TXSTOP                 = (HCITEST_CMD_BASE + 3)  # HCI2SUBCMD(PROTEST_SUB_TX_TEST_STOP)
TESTOP_SUBCMD_HCI_PROTEST_RXSTART                = (HCITEST_CMD_BASE + 4)  # HCI2SUBCMD(PROTEST_SUB_RX_TEST_START)
TESTOP_SUBCMD_HCI_PROTEST_RXSTOP                 = (HCITEST_CMD_BASE + 5)  # HCI2SUBCMD(PROTEST_SUB_RX_TEST_STOP)
TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_ENABLE         = (HCITEST_CMD_BASE + 6)  # HCI2SUBCMD(PROTEST_SUB_HF_XTAL_ENABLE)
TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_DISABLE        = (HCITEST_CMD_BASE + 7)  # HCI2SUBCMD(PROTEST_SUB_HF_XTAL_DISABLE)
TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_ENABLE         = (HCITEST_CMD_BASE + 8)  # HCI2SUBCMD(PROTEST_SUB_LF_XTAL_ENABLE)
TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_DISABLE        = (HCITEST_CMD_BASE + 9)  # HCI2SUBCMD(PROTEST_SUB_LF_XTAL_DISABLE)
TESTOP_SUBCMD_HCI_PROTEST_GET_SVLD               = (HCITEST_CMD_BASE +10)  # HCI2SUBCMD(PROTEST_SUB_GET_SVLD )
TESTOP_SUBCMD_HCI_PROTEST_SET_GPIO               = (HCITEST_CMD_BASE +11)  # HCI2SUBCMD(PROTEST_SUB_SET_GPIO )

# Indices to bytes in the response packet
TESTOP_RESP_IDX_CMD              = (0)
TESTOP_RESP_IDX_LEN              = (1)
TESTOP_RESP_IDX_SUBCMD           = (2)
TESTOP_RESP_IDX_SEQNUM           = (3)
TESTOP_RESP_IDX_ERRORCODE        = (4)
# The response detail starts in byte 5
TESTOP_RESP_IDX_DETAIL           = (5)

# These map to the HCI error codes define in hci.h except for
TESTOP_ERRCODE_SUCCESS           = (0)
TESTOP_ERRCODE_UNK_CMD           = (0x01)      #// Unknown HCI command
TESTOP_ERRCODE_HW_FAIL           = (0x03)      #// Hardware Failure
TESTOP_ERRCODE_NOT_ALLOWED       = (0x0C)      #// Command disallowed
TESTOP_ERRCODE_BAD_PARAMS        = (0x12)      #// Invalid parameters
TESTOP_ERRCODE_CMD_TIMEOUT       = (0xF0)      # Command timeout
TESTOP_ERRCODE_RESPONSE_PARSE_ERR= (0xF1)      #
TESTOP_ERRCODE_DEVICE_HARD_FAIL  = (0xFC)      # Command failed because device is missing or failed
TESTOP_ERRCODE_TX_BUSY           = (0xFD)      # TX failed because its busy
TESTOP_ERRCODE_TX_ERR            = (0xFE)      # TX failed because of invalid command or args
TESTOP_ERRCODE_CMD_COMPLETE_NOT_RECEIVED = (0xFF)

# Define device Slot values (as per em9304_task.h)
SLOT_REF             = 0        # Reference 9304
SLOT_DUT             = 1        # 9304 Under Test
SLOT_NA              = 1        # Used if the slot argument is not applicable

# Define ADC Channels and current range control
ADC_CHANNEL_1        = 0
ADC_CHANNEL_2        = 1
ADC_CHANNEL_3        = 2
ADC_RANGE_1          = 0
ADC_RANGE_2          = 1
ADC_RANGE_3          = 2

# End of constants defined by command_if.h
# ==========================================================================
# ==========================================================================

# ==========================================================================
# Constants for the SI5351 Clock chip
SI5351_PLL_NONE     = 0xff  # -1
SI5351_PLL_A        = 0x00
SI5351_PLL_B        = 0x01

SI5351_R_DIV_1      = 0
SI5351_R_DIV_2      = 1
SI5351_R_DIV_4      = 2
SI5351_R_DIV_8      = 3
SI5351_R_DIV_16     = 4
SI5351_R_DIV_32     = 5
SI5351_R_DIV_64     = 6
SI5351_R_DIV_128    = 7

# ==========================================================================
# Global data
# ==========================================================================
# default value of the device slot
devSlot = SLOT_DUT
# Command sequence number bumped by one each command sent
cmdSeqNum = 1
# Set verbose > 0 to provide more debug output
verbose = 0
# Last response detail
lastResponseDetailString = "N/R"

# ==========================================================================
#   Global data to hold return parameters (parsed from the response packets)
# ==========================================================================
last_upload_response=0
triggered_current_values =[]
last_PER_value=100
last_ppm=100000
# For the SVLD_MEASUREMENT response
last_SVLD_power_mode  = 0
last_SVLD_measurement = 0
last_Digital_read=-1
# For the GET_MEMORY_USAGE response
last_memusage_memory_pool_size           = 0
last_memusage_retention_memory_used      = 0
last_memusage_nonretention_memory_used   = 0
last_memusage_retention_memory_reserved  = 0
# For the CALCULATE_CRC32_EX response
last_calccrc_crc32                       = 0
# For the READ_ADC1, READ_ADC2 and READ_ADC3 response
last_adc_measurement                     = 0
# For READ_PTB_FW_VER response
last_board_ver_string                    = ''
last_board_serial_num_string             = ''
last_dev_ver_string                      = ''
last_dut_ver_string                      = ''
last_ref_ver_string                      = ''
# For READ_AT_ADDR response
last_read_mem_string                     = ''
# For PATCH_QUERY
last_patch_container_count  = 0
last_patch_transfer_count   = 0
last_patch_system_state     = 0
last_patch_address          = 0
last_patch_size             = 0
last_patch_CRC32            = 0
last_patch_build_num        = 0
last_patch_user_build_num   = 0
last_patch_container_flags  = 0
last_patch_container_version= 0
last_patch_container_type   = 0
last_patch_container_id     = 0
# For TESTOP_SUBCMD_HCI_TRANSMITTER_TEST_END
last_test_number_of_packets = 0

# For SET_RF_POWER_LEVEL_EX
last_max_power_level        = 0
# For XTAL accuracy test
last_xtal_dut_tics          = 0
last_xtal_ref_tics          = 0
# For BD Address
last_BD_address             = 0
# For Advertising Report
last_advertReport_totalReports  = 0
last_advertReport_minRssi       = 0
last_advertReport_maxRssi       = 0
last_advertReport_aveRssi       = 0
last_advertReport_lastRssi      = 0

# Overall Test stats
test_verification_count = 0
test_error_count = 0

last_is_busy = 0

global logger
logger = logging.getLogger("prodvktest")

read_results_index = 0
# ==========================================================================
#   Initialize the logger
#   Parent script should call this during initialization and provide a
#   name that is appropriate for the function of the parent script.
# ==========================================================================
def set_logger(log_name, log_level = logging.WARNING):
    global logger
    logging.basicConfig(level = log_level)
    logger = logging.getLogger(log_name)
    file_hdlr = logging.FileHandler(log_name + '.log')
    logger.addHandler(file_hdlr)

    # pass the logger handle to the configedit module
    #configedit.set_logger(log_name)
    return logger


# ==========================================================================
#     Helper function to return the last calculated CRC32
# ==========================================================================
def get_last_calccrc_crc32():
    return last_calccrc_crc32


# ==========================================================================
#     Helper function to return the last read mem string
# ==========================================================================
def get_last_read_mem_string():
    return last_read_mem_string


# ==========================================================================
#     Helper function to return the last SVLD power mode
# ==========================================================================
def get_last_SVLD_power_mode():
    return last_SVLD_power_mode


# ==========================================================================
#     Helper function to return the last SVLD measured value
# ==========================================================================
def get_last_SVLD_measurement():
    return last_SVLD_measurement


# ==========================================================================
#     Helper function to return the last ADC measurement
# ==========================================================================
def get_last_adc_measurement():
    return last_adc_measurement

# ==========================================================================
#     Helper function to return the last device version string
# ==========================================================================
def get_last_dev_ver_string():
    return last_dev_ver_string

# ==========================================================================
#     Helper function to return the last DUT version string
# ==========================================================================
def get_last_dut_ver_string():
    return last_dut_ver_string


# ==========================================================================
#     Helper function to return the last REF version string
# ==========================================================================
def get_last_ref_ver_string():
    return last_ref_ver_string


# ==========================================================================
#     Helper function to return the last Board FW version string
# ==========================================================================
def get_last_board_ver_string():
    return last_board_ver_string


# ==========================================================================
#     Helper function to return the last max power level
# ==========================================================================
def get_last_max_power_level():
    return last_max_power_level


# ==========================================================================
#     Helper function to return the last Rx/Tx test number_of_packets
# ==========================================================================
def get_last_test_number_of_packets():
    return last_test_number_of_packets

# ==========================================================================
#     Helper function to return the last Rx/Tx test number_of_packets
# ==========================================================================
def get_last_xtal_dut_tics():
    return last_xtal_dut_tics

# ==========================================================================
#     Helper function to return the last Rx/Tx test number_of_packets
# ==========================================================================
def get_last_xtal_ref_tics():
    return last_xtal_ref_tics

# ==========================================================================
#     Helper function to return the BLE Address
# ==========================================================================
def get_last_BD_address():
    return last_BD_address

# ==========================================================================
#     Helper function to return the BLE Address
# ==========================================================================
def get_last_AdvRpt_totalCount():
    return last_advertReport_totalReports

# ==========================================================================
#     Helper function to return the BLE Address
# ==========================================================================
def get_last_AdvRpt_minRssi():
    return last_advertReport_minRssi

# ==========================================================================
#     Helper function to return the BLE Address
# ==========================================================================
def get_last_AdvRpt_maxRssi():
    return last_advertReport_maxRssi

# ==========================================================================
#     Helper function to return all Triggered Current Measurements
# ==========================================================================
def get_Triggered_Current_Values():
    return triggered_current_values

def get_Last_Upload_Response():
    return last_upload_response

# ==========================================================================
#     Helper function to return the last PER Value
# ==========================================================================
def get_last_PER_Value():
    return last_PER_value

def get_last_ppm():
    return last_ppm

def get_last_digital_read():
    return last_Digital_read
# ==========================================================================
# ==========================================================================
def clear_Triggered_Current_Values():
    global triggered_current_values
    triggered_current_values=[]
    return

# For PATCH_QUERY
# ==========================================================================
#     Helper functions to return the last values for the em_patch_query command
# ==========================================================================
def get_last_patch_container_count():   return last_patch_container_count
def get_last_patch_transfer_count():    return last_patch_transfer_count
def get_last_patch_system_state():      return last_patch_system_state
def get_last_patch_address():           return last_patch_address
def get_last_patch_size():              return last_patch_size
def get_last_patch_CRC32():             return last_patch_CRC32
def get_last_patch_build_num():         return last_patch_build_num
def get_last_patch_user_build_num():    return last_patch_user_build_num
def get_last_patch_container_flags():   return last_patch_container_flags
def get_last_patch_container_version(): return last_patch_container_version
def get_last_patch_container_type():    return last_patch_container_type
def get_last_patch_container_id():      return last_patch_container_id

def get_test_error_count():
    global test_error_count
    return test_error_count


# ==========================================================================
#     Helper function to return the BLE Address
# ==========================================================================
def get_last_AdvRpt_aveRssi():
    return last_advertReport_aveRssi

# ==========================================================================
#     Helper function to return the BLE Address
# ==========================================================================
def get_last_AdvRpt_lastRssi():
    return last_advertReport_lastRssi

# ==========================================================================
#     Helper function to return the IsBusy status
# ==========================================================================
def get_last_IsBusy():
    return last_is_busy

# ==========================================================================
#     Helper function to return the IsBusy status
# ==========================================================================
def get_read_results():
    return last_read_results

# ==========================================================================
#     Helper function to generate a standard prefix for test failure log messages
#     Standard format:  Test Failed: Sleep Mode Current:  xxxxxxx
# ==========================================================================
def generate_test_failed_prefix( testTitle):
    return ("Test Failed: " + testTitle + ": ")


# ==========================================================================
#     Helper function to generate a standard prefix for test passed log messages
#     Standard format:  Test Passed: Sleep Mode Current:  xxxxxxx
# ==========================================================================
def generate_test_passed_prefix( testTitle):
    return ("Test Passed: " + testTitle + ": ")


# ==========================================================================
#     Helper function to generate a standard test header
# ==========================================================================
def generate_test_header( test_title_arg):
    global test_title

    # Save the test title for later use in the test summary/footer
    test_title = test_title_arg

    dt = datetime.now()
    dtString  = f'{dt:%Y-%m-%d  %H:%M:%S}'
    logging.basicConfig(level=logging.INFO)
    old_level = logger.getEffectiveLevel()
    logger.setLevel(logging.INFO)
    logger.info("==============> Start Test: " + test_title + ":        " + dtString)
    logger.setLevel(level=old_level)

    return



# ==========================================================================
#     Helper function to generate a test summary
# ==========================================================================
def generate_test_summary():
    global test_error_count
    global test_verification_count
    global test_title

    dt = datetime.now()
    dtString  = f'{dt:%Y-%m-%d  %H:%M:%S}'
    logging.basicConfig(level=logging.INFO)
    old_level = logger.getEffectiveLevel()
    # Force the log level to INFO so we get the "TEST PASSED" even if it is set to just the WARN level
    logger.setLevel(logging.INFO)

    if test_error_count > 0:
        logger.warning('TEST FAILED: ' + test_title + ':  Total Verifications=' + str(test_verification_count) +
                       ' Total Errors= ' + str(test_error_count) + '      ' + dtString + '\n\n')
    else:
        logger.info(   'TEST PASSED: ' + test_title + ':  Total Verifications=' + str(test_verification_count) +
                       '      ' + dtString + '\n\n')

    # Restore the logger level to what it was
    logger.setLevel(level=old_level)
    return


# ==========================================================================
#     Helper function to generate FW version strings for DUT, REF and DVK Board FW
# ==========================================================================
def generate_test_fw_version_log():
    # Query the FW Versions for all the main components
    send_command(TESTOP_SUBCMD_READ_DUT_VER, SLOT_NA)
    dut_fw_version = get_last_dut_ver_string()
    send_command(TESTOP_SUBCMD_READ_REF_VER, SLOT_NA)
    ref_fw_version = get_last_ref_ver_string()
    send_command_read_board_ver()
    board_fw_version = get_last_board_ver_string()

    logging.basicConfig(level=logging.INFO)
    old_level = logger.getEffectiveLevel()
    # Force the log level to INFO so we get this logged even if level is set to just the WARNING level
    logger.setLevel(logging.INFO)

    logger.info('Board FW Version=' + board_fw_version + '  DUT FW Version=' + dut_fw_version + '  REF FW Version=' + ref_fw_version)

    logger.setLevel(old_level)
    return


# ==========================================================================
#   Helper function to log a string to the log file at INFO level even
#    if current level is WARNING
# ==========================================================================
def log_string( log_message ):
    old_level = logger.getEffectiveLevel()
    logger.setLevel(logging.INFO)
    logger.info(log_message)
    logger.setLevel(level=old_level)
    return


# ==========================================================================
#   Perform hard reset on the EM9304 devices (DUT and REF)
# ==========================================================================
def reset_em_devices_hard():
    send_command_hard_reset(  SLOT_DUT )
    send_command_hard_reset(  SLOT_REF )
    return


# ==========================================================================
#   Perform soft reset on the EM9304 devices (DUT and REF)
# ==========================================================================
def reset_em_devices():
    send_command_soft_reset( SLOT_DUT )
    send_command_soft_reset( SLOT_REF )
    return


# ==========================================================================
#     Switch To TestOp Mode
# ==========================================================================
def switch_to_testop_mode():
    logger.info("Switch to TestOp Mode")
    send_command(TESTOP_SUBCMD_MODESWITCH_TO_TESTOP, devSlot)
    return


# ==========================================================================
#   Switch Bridge Mode
# ==========================================================================
def switch_to_bridge_mode():
    logger.info("Switch to Bridge Mode")
    send_command(TESTOP_SUBCMD_MODESWITCH_TO_BRIDGE, devSlot)
    return


# ==========================================================================
#   Convert a string of bytes to printable ASCII string
# ==========================================================================
def convert_bytes_to_string(bytes):
    string1 = "".join([chr(c) for c in bytes])
    string2 = string1.partition('\0')[0]
    return string2


# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with no args (other than devSlot)
# ==========================================================================
def send_command(subcmd, devSlot):
    return send_command_with_args(subcmd, devSlot, 0, 0)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 1 arg (other than devSlot)
# ==========================================================================
def send_command_1argbyte(subcmd, devSlot, argval):
    argvect = [argval]
    return send_command_with_args(subcmd, devSlot, 1, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Version with 1 short-word arg (int16) (other than devSlot)
# ==========================================================================
def send_command_1argshort(subcmd, devSlot, argval1):
    # Use the '<' modifier to force little endian packing
    argvect = struct.pack('<H', argval1)
    return send_command_with_args(subcmd, devSlot, 2, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 1 long-word arg (other than devSlot)
# ==========================================================================
def send_command_1argword(subcmd, devSlot, argval1):
    # Use the '<' modifier to force little endian packing
    argvect = struct.pack('<L', argval1)
    return send_command_with_args(subcmd, devSlot, 4, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 2 long-word args (other than devSlot)
# ==========================================================================
def send_command_2argwords(subcmd, devSlot, argval1, argval2):
    # Use the '<' modifier to force little endian packing
    argvect = struct.pack('<LL', argval1, argval2)
    return send_command_with_args(subcmd, devSlot, 8, argvect)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 2 byte-wide args (other than devSlot)
# ==========================================================================
def send_command_2argbytes(subcmd, devSlot, argval1, argval2):
    argvect = struct.pack('BB', argval1, argval2)
    return send_command_with_args(subcmd, devSlot, 2, argvect)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 3 byte-wide args (other than devSlot)
# ==========================================================================
def send_command_3argbytes(subcmd, devSlot, argval1, argval2, argval3):
    argvect = struct.pack('BBB', argval1, argval2, argval3)
    return send_command_with_args(subcmd, devSlot, 3, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 4 byte-wide args (other than devSlot)
# ==========================================================================
def send_command_4argbytes(subcmd, devSlot, argval1, argval2, argval3, argval4):
    argvect = struct.pack('BBBB', argval1, argval2, argval3, argval4)
    return send_command_with_args(subcmd, devSlot, 4, argvect)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 5 byte-wide args (other than devSlot)
# ==========================================================================
def send_command_5argbytes(subcmd, devSlot, argval1, argval2, argval3, argval4, argval5):
    argvect = struct.pack('BBBBB', argval1, argval2, argval3, argval4, argval5)
    return send_command_with_args(subcmd, devSlot, 5, argvect)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 6 byte-wide args (other than devSlot)
# ==========================================================================
def send_command_6argbytes(subcmd, devSlot, argval1, argval2, argval3, argval4, argval5, argval6):
    argvect = struct.pack('BBBBBB', argval1, argval2, argval3, argval4, argval5, argval6)
    return send_command_with_args(subcmd, devSlot, 6, argvect)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 7 byte-wide args (other than devSlot)
# ==========================================================================
def send_command_7argbytes(subcmd, devSlot, argval1, argval2, argval3, argval4, argval5, argval6, argval7):
    argvect = struct.pack('BBBBBBB', argval1, argval2, argval3, argval4, argval5, argval6, argval7)
    return send_command_with_args(subcmd, devSlot, 7, argvect)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#     Version with 2 long-word args (other than devSlot)
# ==========================================================================
def send_command_2argwordbyte(subcmd, devSlot, argval1, argval2):
    argvect = struct.pack('LB', argval1, argval2)
    return send_command_with_args(subcmd, devSlot, 5, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_CALCULATE_CRC32_EX
# ==========================================================================
def send_command_em_calculate_crc32(dev_slot = SLOT_DUT, start_address = 0x2000, end_address=0x3000):
    return send_command_2argwords(TESTOP_SUBCMD_HCI_EM_CALCULATE_CRC32_EX, dev_slot, start_address, end_address)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_CPU_RESET
# ==========================================================================
def send_command_em_cpu_reset(dev_slot = SLOT_DUT):
    return send_command( TESTOP_SUBCMD_HCI_EM_CPU_RESET, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_END_LE_TEST
# ==========================================================================
def send_command_end_le_test(dev_slot = SLOT_DUT):
    return send_command( TESTOP_SUBCMD_HCI_LE_TEST_END, dev_slot)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_LE_TEST_END
#     Alias for send_command_end_le_test() with more symmetry to the name
#          send_command_em_transmitter_test_end
#          send_command_le_test_end
# ==========================================================================
def send_command_le_test_end(dev_slot = SLOT_DUT):
    return send_command( TESTOP_SUBCMD_HCI_LE_TEST_END, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_GET_MEMORY_USAGE
# ==========================================================================
def send_command_em_get_memory_usage(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_EM_GET_MEMORY_USAGE, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Version specifically for TESTOP_SUBCMD_HCI_EM_PATCH_QUERY
# ==========================================================================
def send_command_em_patch_query(dev_slot = SLOT_DUT, patch_index = 0):
    return send_command_1argshort(TESTOP_SUBCMD_HCI_EM_PATCH_QUERY, dev_slot, patch_index)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_SLEEP
# ==========================================================================
def send_command_protest_sleep(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_SLEEP, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_ACTIVE
# ==========================================================================
def send_command_protest_active(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_ACTIVE, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_TXSTART
# ==========================================================================
def send_command_protest_txstart(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_TXSTART, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_TXSTOP
# ==========================================================================
def send_command_protest_txstop(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_TXSTOP, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_RXSTART
# ==========================================================================
def send_command_protest_rxstart(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_RXSTART, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_RXSTOP
# ==========================================================================
def send_command_protest_rxstop(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_RXSTOP, dev_slot)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_DISABLE
# ==========================================================================
def send_command_protest_hf_xtal_disable(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_DISABLE, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_ENABLE
# ==========================================================================
def send_command_protest_hf_xtal_enable(dev_slot = SLOT_DUT, clock_divider = 1):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_PROTEST_HF_XTAL_ENABLE, dev_slot, clock_divider)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_DISABLE
# ==========================================================================
def send_command_protest_lf_xtal_disable(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_DISABLE, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_DISABLE
# ==========================================================================
def send_command_protest_lf_xtal_enable(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_LF_XTAL_ENABLE, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_GET_SVLD
# ==========================================================================
def send_command_protest_get_svld(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_PROTEST_GET_SVLD, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_PROTEST_SET_GPIO
# ==========================================================================
def send_command_protest_set_gpio(dev_slot = SLOT_DUT, gpio_input_reg=0, gpio_output_reg=0,
                              gpio_pullup_reg=1, gpio_pulldown_reg=0):
    param_length = 16  # 4 32-bit words
    # Pack 4 long words (little endian)
    argvect = struct.pack('<LLLL', gpio_input_reg, gpio_output_reg, gpio_pullup_reg, gpio_pulldown_reg);

    return send_command_with_args(TESTOP_SUBCMD_HCI_PROTEST_SET_GPIO, dev_slot, param_length, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_CLOCK_SOURCE
# ==========================================================================
def send_command_em_set_clock_source(dev_slot = SLOT_DUT, clock_source = 0):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_EM_SET_CLOCK_SOURCE, dev_slot, clock_source)


# ==========================================================================
#     Send command to the DVK board and fetch the result
#     Command: TESTOP_SUBCMD_SET_CURRENT_RANGE_1/2/3
# ==========================================================================
def send_command_set_current_range(adc_range = 0):
    subcmd = TESTOP_SUBCMD_SET_CURRENT_RANGE_1

    if adc_range == 0:
        subcmd = TESTOP_SUBCMD_SET_CURRENT_RANGE_1
    elif adc_range == 1:
        subcmd = TESTOP_SUBCMD_SET_CURRENT_RANGE_2
    elif adc_range == 2:
        subcmd = TESTOP_SUBCMD_SET_CURRENT_RANGE_3

    return send_command(subcmd, SLOT_NA)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_EVENT_MASK
# ==========================================================================
def send_command_em_set_event_mask(dev_slot = SLOT_DUT, event_mask = 1):
    return send_command_1argword(TESTOP_SUBCMD_HCI_EM_SET_EVENT_MASK, dev_slot, event_mask)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_MEMORY_MODE
# ==========================================================================
def send_command_em_set_memory_mode(dev_slot = SLOT_DUT, memory_mode = 0):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_EM_SET_MEMORY_MODE, dev_slot, memory_mode)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_READ_PTB_FW_VER
# ==========================================================================
def send_command_read_board_ver():
    return send_command( TESTOP_SUBCMD_READ_PRODVK_FW_VER, SLOT_NA )


# ==========================================================================
#     Send command to DVK board and fetch the result
#     Command: TESTOP_SUBCMD_HCI_EM_SET_POWER_MODE_EX
# ==========================================================================
def send_command_em_set_power_mode(dev_slot = SLOT_DUT, power_mode = 0):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_EM_SET_POWER_MODE_EX, dev_slot, power_mode)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_PUBLIC_ADDRESS
# ==========================================================================
def send_command_em_set_public_address(dev_slot = SLOT_DUT, public_address = 0):

    paramLength = 6
    # Pack the 6 bytes of the address
    argvect = struct.pack('BBBBBB',
                          public_address[0],
                          public_address[1],
                          public_address[2],
                          public_address[3],
                          public_address[4],
                          public_address[5]);

    return send_command_with_args(TESTOP_SUBCMD_HCI_EM_SET_PUBLIC_ADDRESS, dev_slot, paramLength, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_RF_ACTIVITY_SIGNAL
# ==========================================================================
def send_command_em_set_rf_activity_signal(dev_slot = SLOT_DUT, rf_signal_enable = 0, rf_signal_gpio_output = 0):
    return send_command_2argbytes(TESTOP_SUBCMD_HCI_EM_SET_RF_ACTIVITY_SIGNAL_EX, dev_slot, rf_signal_enable, rf_signal_gpio_output)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_RF_POWER_LEVEL
# ==========================================================================
def send_command_em_set_rf_power_level(dev_slot = SLOT_DUT, transmit_power_level = 0):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_EM_SET_RF_POWER_LEVEL_EX, dev_slot, transmit_power_level)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_SLEEP_OPTIONS
# ==========================================================================
def send_command_em_set_sleep_options(dev_slot = SLOT_DUT, sleep_options = 0):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_EM_SET_SLEEP_OPTIONS, dev_slot, sleep_options)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_SET_REF_CLOCK
# ==========================================================================
def send_command_set_ref_clock(
        pllA_mult = 24, pllA_num = 0, pllA_denom = 1,
        pllB_mult = 24, pllB_num = 0, pllB_denom = 1,
        out0_source = SI5351_PLL_A, out0_div = 1326, out0_num = 0, out0_denom = 1, out0_rdiv = SI5351_R_DIV_16,
        out1_source = SI5351_PLL_A, out1_div = 1326, out1_num = 0, out1_denom = 1, out1_rdiv = SI5351_R_DIV_16,
        out2_source = SI5351_PLL_A, out2_div = 1326, out2_num = 0, out2_denom = 1, out2_rdiv = SI5351_R_DIV_16):

    paramLength = 54
    argvect = struct.pack(
        '<BIIBIIBHIIBBHIIBBHIIB',
        pllA_mult, pllA_num, pllA_denom,                        # PLL A Config (BII)
        pllB_mult, pllB_num, pllB_denom,                        # PLL B Config (BII)
        out0_source, out0_div, out0_num, out0_denom, out0_rdiv, # Output 0 Config (BHIIB)
        out1_source, out1_div, out1_num, out1_denom, out1_rdiv, # Output 1 Config (BHIIB)
        out2_source, out2_div, out2_num, out2_denom, out2_rdiv) # Output 2 Config (BHIIB)

    return send_command_with_args(TESTOP_SUBCMD_SET_REF_CLOCK, SLOT_NA, paramLength, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SET_UART_BAUD_RATE
#       Default baud_rate = 5 -> 19200 bps
# ==========================================================================
def send_command_em_set_uart_baud_rate(dev_slot = SLOT_DUT, baud_rate = 5):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_EM_SET_UART_BAUD_RATE, dev_slot, baud_rate)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_RESET
# ==========================================================================
def send_command_soft_reset(dev_slot = SLOT_DUT):
    return send_command( TESTOP_SUBCMD_HCI_RESET, dev_slot )


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Version specifically for TESTOP_SUBCMD_RESET_9304

# ==========================================================================
def send_command_hard_reset(dev_slot = SLOT_DUT):
    return send_command( TESTOP_SUBCMD_RESET_9304, dev_slot )


# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_READ_BD_ADDR
# ==========================================================================
def send_command_read_BLE_address(dev_slot = SLOT_DUT):
    return send_command( TESTOP_SUBCMD_HCI_READ_BD_ADDR, dev_slot )

# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_LE_SET_ADVERTISING_DATA
# ==========================================================================
def send_command_le_set_Advertising_data(dev_slot = SLOT_DUT, advertising_data_length = 1, advertising_data = [0]):
    paramLength = advertising_data_length + 1
    # First byte must be the number of advertising_data bytes (0-31)
    argvect = struct.pack('B', advertising_data_length)

    # Now add to the vector all the advertising_data bytes
    i = 0
    while i < advertising_data_length:
        argvect = argvect + struct.pack('B', advertising_data[i])
        i = i + 1

    return send_command_with_args(TESTOP_SUBCMD_HCI_SET_ADVERTISING_DATA, dev_slot, paramLength, argvect)

# ==========================================================================
#     Send command to the DVK board and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_SET_ADVERTISING_PARAMETERS
#   Note: the min and max values are mapped to the upper byte of a U16 and
#       the chip uses that U16 value * 0.625ms for the time.  Example: the
#       default value (0x01) => 0x0100 * 0.625ms = 160ms delta time.
# ==========================================================================
def send_command_le_set_Advertising_parameters(dev_slot = SLOT_DUT, min=0x01, max=0x01, AdType=0x00, AdChMap=0x07):
    return send_command_4argbytes( TESTOP_SUBCMD_HCI_SET_ADVERTISING_PARAMETERS, dev_slot, min, max, AdType, AdChMap )

# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_LE_SET_ADVERTISE_ENABLE
#     0 = OFF / 1 = ON
# ==========================================================================
def send_command_le_set_Advertising_enable(dev_slot = SLOT_DUT, enableState=1):
    return send_command_1argbyte( TESTOP_SUBCMD_HCI_LE_SET_ADVERTISE_ENABLE, dev_slot, enableState )

# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_LE_CLEAR_WHITE_LIST (no params)
# ==========================================================================
def send_command_le_clear_White_List(dev_slot = SLOT_REF):
    return send_command( TESTOP_SUBCMD_HCI_LE_CLEAR_WHITE_LIST, dev_slot )

# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_LE_ADD_DEVICE_TO_WHITE_LIST
# ==========================================================================
def send_command_le_add_device_to_White_List(dev_slot = SLOT_REF, addressType = '00', bleAddress = '00'):
    msg = bytearray(addressType.encode()+bleAddress.encode())
    return send_command_with_args( TESTOP_SUBCMD_HCI_LE_ADD_DEVICE_TO_WHITE_LIST, dev_slot, len(msg), msg)

# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_LE_SET_SCAN_PARAMETERS
# ==========================================================================
def send_command_le_set_Scan_Parameters(dev_slot = SLOT_REF, scanType='00', scanIntvl='0004', scanWindow='0004', addrType='00', filterPolicy='00' ):
    msg = bytearray(scanType.encode()+scanIntvl.encode()+scanWindow.encode()+addrType.encode()+filterPolicy.encode())
    return send_command_with_args( TESTOP_SUBCMD_HCI_LE_SET_SCAN_PARAMETERS, dev_slot, len(msg), msg )

# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_LE_SET_SCAN_ENABLE
# ==========================================================================
def send_command_le_set_Scan_Enable(dev_slot = SLOT_REF, enableState=1, filterDups=0):
    return send_command_2argbytes( TESTOP_SUBCMD_HCI_LE_SET_SCAN_ENABLE, dev_slot, enableState, filterDups)

# ==========================================================================
#     Send command to PTB and fetch the result
#     Version specifically for TESTOP_SUBCMD_HCI_LE_GET_ADVERTISING_REPORT
# ==========================================================================
def send_command_get_advertising_report_data(dev_slot = SLOT_REF):
    return send_command( TESTOP_SUBCMD_HCI_LE_GET_ADVERTISING_REPORT, dev_slot)

# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_LE_TRANSMITTER_TEST
# ==========================================================================
def send_command_le_transmitter_test(dev_slot = SLOT_DUT, channel = 1, payload_len = 10, payload_type = 1):
    return send_command_3argbytes(TESTOP_SUBCMD_HCI_LE_TRANSMITTER_TEST, dev_slot, channel, payload_len, payload_type)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_LE_RECEIVER_TEST
# ==========================================================================
def send_command_le_receiver_test(dev_slot = SLOT_DUT, channel = 1):
    return send_command_1argbyte(TESTOP_SUBCMD_HCI_LE_RECEIVER_TEST, dev_slot, channel)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_SVLD_MEASUREMENT
# ==========================================================================
def send_command_em_svld_measurement(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_EM_SVLD_MEASUREMENT, dev_slot)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_READ_AT_ADDRESS
# ==========================================================================
def send_command_em_read_at_address(dev_slot = SLOT_DUT, start_address = 0x1000, bytes_to_read = 4):
    return send_command_2argwordbyte(TESTOP_SUBCMD_HCI_EM_READ_AT_ADDRESS, dev_slot, start_address,   bytes_to_read)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_WRITE_AT_ADDRESS
# ==========================================================================
def send_command_em_write_at_address(dev_slot = SLOT_DUT, start_address = 0x1000, data_buf = [0], bytes_to_write = 1):
    paramLength = 4 + 4
    # Pack the start address
    argvect = struct.pack('L', start_address)

    i = 0
    while i < bytes_to_write:
        argvect = argvect + struct.pack('B', data_buf[i])
        i = i + 1

    return send_command_with_args(TESTOP_SUBCMD_HCI_EM_WRITE_AT_ADDRESS, dev_slot, paramLength, argvect)


# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST
# ==========================================================================
def send_command_em_transmitter_test(dev_slot = SLOT_DUT, test_mode = 0, channel = 0x13, packet_len = 37, payload_type = 0):
    return send_command_4argbytes(TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST, dev_slot, test_mode, channel, packet_len, payload_type)



# ==========================================================================
#   Send command to the DVK board and fetch the result
#   Command: TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST_END
# ==========================================================================
def send_command_em_transmitter_test_end(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST_END, dev_slot)


# ==========================================================================
#     Run test to get the clock tics from XTAL and Ref Clock.  The input is
#       the maximum number of clock to get from the DUT.  This test will
#       then shut down both clocks and report the counts.
#
# ==========================================================================
def execute_xtal_validation(maxDutClocks):
    return send_command_1argword(TESTOP_SUBCMD_EXEC_XTALVALIDATION, SLOT_NA, maxDutClocks)


# ==========================================================================
#     TEMPORARY!  Allows you to set the MUXes to any state in any combination!!
#
# ==========================================================================
def send_command_set_mux_state(source = 0, state = 0):
    return send_command_1argbyte(TESTOP_SUBCMD_SET_MUX_STATE, source, state)


# ==========================================================================
#     Run calibration test
#
# ==========================================================================
def execute_current_calibration(code=0):
    dev_slot = SLOT_DUT     # REF current cannot be calibrated
    return send_command_1argbyte(TESTOP_SUBCMD_EXEC_CALIBRATION, dev_slot, code)

# ==========================================================================
#     Get the current (DUT is in ACTIVE mode and auto adjusts the current calculation)
#
# ==========================================================================
def send_command_read_current(dev_slot = SLOT_DUT):
    return send_command(TESTOP_SUBCMD_READ_CURRENT, dev_slot)

# ==========================================================================
#     Get the current (DUT is in ACTIVE mode and auto adjusts the current calculation)
#
# ==========================================================================
def send_command_read_ADC(dev_slot = SLOT_DUT, adcIndex=0):
    return send_command_1argbyte(TESTOP_SUBCMD_READ_ADC, dev_slot, adcIndex)

# ==========================================================================
#     Convert a two byte array into an int
# ==========================================================================
def convert_array_to_short(array):
    # EM9304 uses little endian byte order
    low_byte = array[0]
    hi_byte  = array[1]
    intval = (hi_byte << 8) + low_byte
    return intval


# ==========================================================================
#     Convert a 4 byte array into a 32 bit long
# ==========================================================================
def convert_array_to_long(array):
    # EM9304 uses little endian byte order
    intval = (array[3] << 24) + (array[2] << 16) + (array[1] << 8) + array[0]
    return intval


# ==========================================================================
#     Convert a 4 byte array into a signed 32 bit long
# ==========================================================================
def convert_array_to_slong(array):
    # EM9304 uses little endian byte order
    intval = (array[3] << 24) + (array[2] << 16) + (array[1] << 8) + array[0]
    if (2147483648 <= intval):
        intval = intval - 4294967296
    return intval

# ==========================================================================
#     Convert a 4 byte array into a signed 32 bit long
# ==========================================================================
def convert_array_to_int(array):
    # EM9304 uses little endian byte order
    intval = (array[0] << 24) + (array[1] << 16) + (array[2] << 8) + array[3]
    if (2147483648 <= intval):
        intval = intval - 4294967296
    return intval


# ==========================================================================
#     Convert int array into hex string for display
# ==========================================================================
def convert_array_to_hex(array, num):
    hexstring = ''
    i = 0
    while i < num:
        #hexstring = hexstring + hex(array[i]) + ' '
        hexstring = hexstring + '{:02x}'.format(array[i]) + ' '
        i = i + 1

    return hexstring


# ==========================================================================
#     Convert int array into hex string for display
# ==========================================================================
def convert_array_to_hex_no_spaces(array, num):
    hexstring = ''
    i = 0
    while i < num:
        hexstring = hexstring + '{:02x}'.format(array[i])
        i = i + 1
    return hexstring

# ==========================================================================
#     Convert error code to string
#
# ==========================================================================
def convert_errcode_to_string(errcode):
    if errcode == TESTOP_ERRCODE_SUCCESS:
        return "Success"
    elif errcode == TESTOP_ERRCODE_UNK_CMD:
        return "Unknown command"
    elif errcode == TESTOP_ERRCODE_HW_FAIL:
        return "HW Failure"
    elif errcode == TESTOP_ERRCODE_NOT_ALLOWED:
        return "Command not allowed"
    elif errcode == TESTOP_ERRCODE_BAD_PARAMS:
        return "Invalid parameters"
    elif errcode == TESTOP_ERRCODE_CMD_TIMEOUT:
        return "Command timeout"
    elif errcode == TESTOP_ERRCODE_TX_BUSY:
        return "Command Tx failed because device is busy"
    elif errcode == TESTOP_ERRCODE_TX_ERR:
        return "Command Tx failed because invalid command"
    elif errcode == TESTOP_ERRCODE_CMD_COMPLETE_NOT_RECEIVED:
        return "Command complete not received"
    else:
        return "unknown error code"


# ==========================================================================
#     Convert a subcmd index to a name string
# ==========================================================================
def convert_subcmd_to_string(subcmd_index):

        if   subcmd_index == TESTOP_SUBCMD_READ_PRODVK_FW_VER:
            return          'TESTOP_SUBCMD_READ_PRODVK_FW_VER'
        elif subcmd_index == TESTOP_SUBCMD_READ_PRODVK_SN:
            return          'TESTOP_SUBCMD_READ_PRODVK_SN'
        elif subcmd_index == TESTOP_SUBCMD_MODESWITCH_TO_TESTOP:
            return          'TESTOP_SUBCMD_MODESWITCH_TO_TESTOP'
        elif subcmd_index == TESTOP_SUBCMD_MODESWITCH_TO_BRIDGE:
            return          'TESTOP_SUBCMD_MODESWITCH_TO_BRIDGE'
        elif subcmd_index == TESTOP_SUBCMD_READ_DUT_VER:
            return          'TESTOP_SUBCMD_READ_DUT_VER'
        elif subcmd_index == TESTOP_SUBCMD_READ_REF_VER:
            return          'TESTOP_SUBCMD_READ_REF_VER'
        elif subcmd_index == TESTOP_SUBCMD_READ_CURRENT:
            return          'TESTOP_SUBCMD_READ_CURRENT'
        elif subcmd_index == TESTOP_SUBCMD_READ_ADC:
            return          'TESTOP_SUBCMD_READ_ADC'
        elif subcmd_index == TESTOP_SUBCMD_UNUSED:
            return          'TESTOP_SUBCMD_UNUSED'
        elif subcmd_index == TESTOP_SUBCMD_READ_STATUS:
            return          'TESTOP_SUBCMD_READ_STATUS'
        elif subcmd_index == TESTOP_SUBCMD_SET_CURRENT_RANGE_1:
            return          'TESTOP_SUBCMD_SET_CURRENT_RANGE_1'
        elif subcmd_index == TESTOP_SUBCMD_SET_CURRENT_RANGE_2:
            return          'TESTOP_SUBCMD_SET_CURRENT_RANGE_2'
        elif subcmd_index == TESTOP_SUBCMD_SET_CURRENT_RANGE_3:
            return          'TESTOP_SUBCMD_SET_CURRENT_RANGE_3'
        elif subcmd_index == TESTOP_SUBCMD_RESET_9304:
            return          'TESTOP_SUBCMD_RESET_9304'
        elif subcmd_index == TESTOP_SUBCMD_SET_REF_CLOCK:
            return          'TESTOP_SUBCMD_SET_REF_CLOCK'
        elif subcmd_index == TESTOP_SUBCMD_EXEC_XTALVALIDATION:
            return          'TESTOP_SUBCMD_EXEC_XTALVALIDATION'
        elif subcmd_index == TESTOP_SUBCMD_HCI_READ_9304_VER:
            return          'TESTOP_SUBCMD_HCI_READ_9304_VER'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_RECEIVER_TEST:
            return          'TESTOP_SUBCMD_HCI_LE_RECEIVER_TEST'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_TRANSMITTER_TEST:
            return          'TESTOP_SUBCMD_HCI_LE_TRANSMITTER_TEST'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_TEST_END:
            return          'TESTOP_SUBCMD_HCI_LE_TEST_END'
        elif subcmd_index == TESTOP_SUBCMD_HCI_RESET:
            return          'TESTOP_SUBCMD_HCI_RESET'
        elif subcmd_index == TESTOP_SUBCMD_HCI_READ_BD_ADDR:
            return          'TESTOP_SUBCMD_HCI_READ_BD_ADDR'
        elif subcmd_index == TESTOP_SUBCMD_HCI_SET_ADVERTISING_DATA:
            return          'TESTOP_SUBCMD_HCI_SET_ADVERTISING_DATA'
        elif subcmd_index == TESTOP_SUBCMD_HCI_SET_ADVERTISING_PARAMETERS:
            return          'TESTOP_SUBCMD_HCI_SET_ADVERTISING_PARAMETERS'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_SET_ADVERTISE_ENABLE:
            return          'TESTOP_SUBCMD_HCI_LE_SET_ADVERTISE_ENABLE'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_CLEAR_WHITE_LIST:
            return          'TESTOP_SUBCMD_HCI_LE_CLEAR_WHITE_LIST'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_ADD_DEVICE_TO_WHITE_LIST:
            return          'TESTOP_SUBCMD_HCI_LE_ADD_DEVICE_TO_WHITE_LIST'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_SET_SCAN_PARAMETERS:
            return          'TESTOP_SUBCMD_HCI_LE_SET_SCAN_PARAMETERS'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_SET_SCAN_ENABLE:
            return          'TESTOP_SUBCMD_HCI_LE_SET_SCAN_ENABLE'
        elif subcmd_index == TESTOP_SUBCMD_HCI_LE_GET_ADVERTISING_REPORT:
            return          'TESTOP_SUBCMD_HCI_LE_GET_ADVERTISING_REPORT'
        elif subcmd_index == TESTOP_SUBCMD_EXEC_CALIBRATION:
            return          'TESTOP_SUBCMD_EXEC_CALIBRATION'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_CURRENT_SLEEP:
            return          'TESTOP_SUBCMD_FUNCTEST_CURRENT_SLEEP'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_CURRENT_ACTIVE:
            return          'TESTOP_SUBCMD_FUNCTEST_CURRENT_ACTIVE'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_CURRENT_RX:
            return          'TESTOP_SUBCMD_FUNCTEST_CURRENT_RX'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_CURRENT_TX:
            return          'TESTOP_SUBCMD_FUNCTEST_CURRENT_TX'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_PER_TX:
            return          'TESTOP_SUBCMD_FUNCTEST_PER_TX'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_PER_RX:
            return          'TESTOP_SUBCMD_FUNCTEST_PER_RX'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_ADVERTISE:
            return 'TESTOP_SUBCMD_FUNCTEST_ADVERTISE'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_RSSI:
            return          'TESTOP_SUBCMD_FUNCTEST_RSSI'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_XTAL:
            return          'TESTOP_SUBCMD_FUNCTEST_XTAL'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_PWR_MODE:
            return          'TESTOP_SUBCMD_FUNCTEST_PWR_MODE'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_SVLD:
            return          'TESTOP_SUBCMD_FUNCTEST_SVLD'
        elif subcmd_index == TESTOP_SUBCMD_FUNCTEST_READ_RESULTS:
            return 'TESTOP_SUBCMD_FUNCTEST_READ_RESULTS'
        elif subcmd_index == TESTOP_SUBCMD_GPIO_CONFIGURE_IO:
            return 'TESTOP_SUBCMD_GPIO_CONFIGURE_IO'
        elif subcmd_index == TESTOP_SUBCMD_GPIO_SET_IO:
            return 'TESTOP_SUBCMD_GPIO_SET_IO'
        elif subcmd_index == TESTOP_SUBCMD_GPIO_READ_DIGITAL_IO:
            return 'TESTOP_SUBCMD_GPIO_READ_DIGITAL_IO'
        elif subcmd_index == TESTOP_SUBCMD_GPIO_READ_ANALOG_IO:
            return 'TESTOP_SUBCMD_GPIO_READ_ANALOG_IO'
        elif subcmd_index == TESTOP_SUBCMD_GPIO_DISABLE_IO_SET:
            return 'TESTOP_SUBCMD_GPIO_DISABLE_IO_SET'
        elif subcmd_index == TESTOP_SUBCMD_MEASURE_TRIGGERED_CURRENT:
            return 'TESTOP_SUBCMD_MEASURE_TRIGGERED_CURRENT'
        elif subcmd_index == TESTOP_SUBCMD_WRITE_DAC_LTC2633:
            return 'TESTOP_SUBCMD_WRITE_DAC_LTC2633'
        elif subcmd_index == TESTOP_SUBCMD_READ_ADC_MAX11614EEE:
            return 'TESTOP_SUBCMD_READ_ADC_MAX11614EEE'
        elif subcmd_index == TESTOP_SUBCMD_UPLOAD_TO_9304:
            return 'TESTOP_SUBCMD_UPLOAD_TO_9304'
        elif subcmd_index == TESTOP_SUBCMD_READ_CRC:
            return 'TESTOP_SUBCMD_READ_CRC'
        elif subcmd_index > 0x40:
            # Use table lookup for the EM Vendor specific commands (which start at 0x40)
            return subcmd_names[subcmd_index - 0x40]
        else:
            return 'unknown'


# ==========================================================================
#     Send command to Production Test Board and fetch the result
#     This version supports additional args as per argcnt (in bytes)
#     The number of bytes indicated are extracted from argvect (a byte array)
# ==========================================================================
def send_command_with_args(subcmd, devSlot, argcnt, argvect):
    global dev
    global cmdSeqNum
    global lastResponseDetailString
    global testErrorCount

    #logger.info('Send Command:   %s %d  %d %s'%(convert_subcmd_to_string(subcmd), devSlot, argcnt,convert_array_to_hex(argvect, argcnt)))
    # Increment the sequence number each invocation
    # Limit it to one byte
    cmdSeqNum = (cmdSeqNum + 1) % 256

    # The number of bytes after the command and length bytes
    # For simplicity, always use at least 3 param bytes even though we don't always use 3
    paramLength = 3 + argcnt

    # Pack the header, including the devSlot (which is technically the first arg byte)
    header = struct.pack('BBBBB',   COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot)

    if argcnt == 0:
        ret = dev.write(DVK_USB_WRITE_EP, struct.pack('BBBBB',    COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot))
    elif argcnt == 1:
        argbyte0 = argvect[0] & 0xff
        ret = dev.write(DVK_USB_WRITE_EP, struct.pack('BBBBBB',   COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot, argbyte0))
    elif argcnt == 2:
        ret = dev.write(DVK_USB_WRITE_EP, struct.pack('BBBBBBB',  COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot, argvect[0], argvect[1]))
    elif argcnt == 3:
        ret = dev.write(DVK_USB_WRITE_EP, struct.pack('BBBBBBBB', COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot,
                                    argvect[0],argvect[1],argvect[2]))
    elif argcnt == 4:
        ret = dev.write(DVK_USB_WRITE_EP, struct.pack('BBBBBBBBB', COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot,
                                    argvect[0], argvect[1], argvect[2], argvect[3]))

    # for larger argcnts, use a loop to construct the argvect
    elif argcnt > 4:
        # Load the command buf with the header and then concatenate all the arg bytes from argvect
        command_buf = header
        i = 0
        while i < argcnt:
            command_buf = command_buf + struct.pack('B', argvect[i])
            i = i + 1

        # Write the command to the USB/HID interface
        ret = dev.write(DVK_USB_WRITE_EP, command_buf)

    else:
        ret = 0
        logger.warning('Command failed.  Invalid number of arguments= ' + str(argcnt))
        testErrorCount = testErrorCount + 1


    # Abort if the command was not successfully sent
    if ret <= 0:
        return TESTOP_ERRCODE_BAD_PARAMS

    # Every command should provide a response
    # If the operation takes longer than the timeout interval, it should still
    # respond indicating the operation is in-progress
    #response = dev.read(DVK_USB_READ_EP, DVK_USB_EP_SIZE, DVK_USB_TIMEOUT)
    response = read_response()

    # Parse the response buffer.
    # A badly formed response might result in a variety of exceptions, so catch
    # exceptions and try to continue
    try:
        errcode = parse_response(subcmd, response)
    except Exception as ex:
        errcode = TESTOP_ERRCODE_RESPONSE_PARSE_ERR
        # Log both the raw buffer and the message from the exception
        logger.warning('Response parse error.  Raw Response buffer: ' + convert_array_to_hex(response, 64))
        logger.warning('Response parse error.  Exception message:   ' + str(getattr(ex, 'message', ex)))


    return errcode

def send_command_message(subcmd, devSlot, msgLen, message):
    global dev
    global cmdSeqNum
    global lastResponseDetailString
    global testErrorCount

    logger.info('Send Command:             ' + convert_subcmd_to_string(subcmd))
    # Increment the sequence number each invocation
    # Limit it to one byte
    cmdSeqNum = (cmdSeqNum + 1) % 256

    # The number of bytes after the command and length bytes
    # For simplicity, always use at least 3 param bytes even though we don't always use 3
    paramLength = 3 + int(msgLen)

    # Pack the header, including the devSlot (which is technically the first arg byte)
    header = struct.pack('BBBBBB',   COMMAND_TESTOP, paramLength, subcmd, cmdSeqNum, devSlot, msgLen )



    command_buf = header
    ret = dev.write(DVK_USB_WRITE_EP, command_buf)



    # for larger argcnts, use a loop to construct the argvect
#     if msgLen > 0:
#         # Load the command buf with the header and then concatenate all the arg bytes from argvect
#         command_buf = header
#         i = 0
#         while i < msgLen:
#             command_buf = command_buf + struct.pack('B', message[i] )
#             i = i + 1
#
#         # Write the command to the USB/HID interface
#         ret = dev.write(DVK_USB_WRITE_EP, command_buf)
#
#     else:
#         ret = 0
#         logger.warning('Command failed.  Invalid length= ' + str(msgLen))
#         #testErrorCount = testErrorCount + 1
#
#     # Abort if the command was not successfully sent
    if ret <= 0:
        return TESTOP_ERRCODE_BAD_PARAMS

    # Every command should provide a response
    # If the operation takes longer than the timeout interval, it should still
    # respond indicating the operation is in-progress
    #response = dev.read(DVK_USB_READ_EP, DVK_USB_EP_SIZE, DVK_USB_TIMEOUT)
    response = read_response()

    # Parse the response buffer.
    # A badly formed response might result in a variety of exceptions, so catch
    # exceptions and try to continue
    try:
        errcode = parse_response(subcmd, response)
    except Exception as ex:
        errcode = TESTOP_ERRCODE_RESPONSE_PARSE_ERR
        # Log both the raw buffer and the message from the exception
        logger.warning('Response parse error.  Raw Response buffer: ' + convert_array_to_hex(response, len(response)))
        logger.warning('Response parse error.  Exception message:   ' + str(getattr(ex, 'message', ex)))


    return errcode

# ==========================================================================
#   Read a response packet
#
# ==========================================================================
def read_response():
    try:
        response = dev.read(DVK_USB_READ_EP, DVK_USB_EP_SIZE, DVK_USB_TIMEOUT)
        if response[0] != COMMAND_TESTOP:
            # Not a valid testop response.  Probably a spontaneous event from the device
            # Discard and read another packet
            #response = dev.read(DVK_USB_READ_EP, DVK_USB_EP_SIZE, DVK_USB_TIMEOUT)
            logger.warning("Spontaneous Response.  Response Header=" + convert_array_to_hex(response, len(response)))

        return response

    except usb.core.USBError:
        logger.error('No response received on USB port! Program exiting.')

        sys.exit(1)

# ==========================================================================
#   Parse the response to the latest command
#
# ==========================================================================
def parse_response(subcmd, response):
    global test_error_count
    global test_verification_count
    global printheader

    # Since we perform verifications on the response, treat each command/response as a verification
    test_verification_count = test_verification_count + 1

    # Convert the response detail to a printable string (if there is some detail bytes)
    response_len           = response[TESTOP_RESP_IDX_LEN]
    # If the response length is greater than 3 then we have some 'detail' bytes
    if response_len > 3:
        response_detail_len = response_len - 3
        response_detail = response[TESTOP_RESP_IDX_DETAIL:(TESTOP_RESP_IDX_DETAIL + response_detail_len)]
        response_detail_string = convert_array_to_hex(response_detail, response_detail_len)
    else:
        response_detail = response[TESTOP_RESP_IDX_DETAIL]
        response_detail_string = ''
        response_detail_len = 0

    errcode = response[TESTOP_RESP_IDX_ERRORCODE]

#    logger.warning("Response Header(%s) %s="%(printheader,errcode) + convert_array_to_hex(response, response_len+2))
    thisResponseSeqNum = response[TESTOP_RESP_IDX_SEQNUM]
    if cmdSeqNum != thisResponseSeqNum:
        logger.warning("Response sequence number does not match command sequence number.  Expected=" + str(cmdSeqNum) +
                       "  Actual=" + str(thisResponseSeqNum))
        logger.warning("Response=" + convert_array_to_hex(response, len(response) ))

    if errcode != TESTOP_ERRCODE_SUCCESS :
        logger.warning('Command failed.  Command = ' + convert_subcmd_to_string(subcmd) +
                       '  ErrorCode = ' + str(errcode) + "  " + convert_errcode_to_string(errcode))
        test_error_count = test_error_count + 1
        # In general, we can't parse the rest of the response if we have an error, so just return
        return errcode


    # Now perform the command-specific parsing
    if subcmd == TESTOP_SUBCMD_READ_STATUS:
        logger.info("Command Response:         " + parse_status(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_GET_MEMORY_USAGE:
        logger.info("Command Response:         " + parse_memory_usage_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_PATCH_QUERY:
        logger.info("Command Response:         " + parse_patch_query(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_READ_AT_ADDRESS:
        logger.info("Command Response:         " + parse_read_mem_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_SVLD_MEASUREMENT:
        logger.info("Command Response:         " + parse_SVLD_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_PROTEST_GET_SVLD:
        logger.info("Command Response:         " + parse_protest_SVLD_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_CALCULATE_CRC32_EX:
        logger.info("Command Response:         " + parse_calculate_crc32_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_READ_CURRENT:
        logger.info("Command Response:         " + parse_read_current_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_READ_ADC:
        logger.info("Command Response:         " + parse_read_ADC_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_UNUSED:
        logger.info("Command Response:  Unused command")
    elif subcmd == TESTOP_SUBCMD_READ_DUT_VER:
        logger.info("Command Response:         " + parse_read_dut_ver_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_READ_REF_VER:
        logger.info("Command Response:         " + parse_read_ref_ver_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_READ_9304_VER:
        logger.info("Command Response:         " + parse_read_dev_ver_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_LE_TEST_END:
        logger.info("Command Response:         " + parse_end_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_TRANSMITTER_TEST_END:
        logger.info("Command Response:         " + parse_end_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_READ_PRODVK_FW_VER:
        logger.info("Command Response:         " + parse_board_ver_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_READ_PRODVK_SN:
        logger.info("Command Response:         " + parse_board_serial_num_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_EM_SET_RF_POWER_LEVEL_EX:
        logger.info("Command Response:         " + parse_set_power_level_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_EXEC_XTALVALIDATION:
        logger.info("Command Response:         " + parse_xtalvalidation_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_READ_BD_ADDR:
        logger.info("Command Response:         " + parse_BD_address_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_HCI_LE_GET_ADVERTISING_REPORT:
        logger.info("Command Response:         " + parse_advertising_report_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_EXEC_CALIBRATION:
        logger.info("Command Response:         " + parse_calibration_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_CURRENT_SLEEP:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_CURRENT_ACTIVE:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_CURRENT_RX:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_CURRENT_TX:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_PER_TX:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_PER_RX:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_ADVERTISE:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_RSSI:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_XTAL:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_PWR_MODE:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_SVLD:
        logger.info("Command Response:         " + parse_read_results_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_FUNCTEST_READ_RESULTS:
        logger.info("Command Response:         " + parse_read_results_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_GPIO_READ_DIGITAL_IO:
        logger.info("Command Response:         " + parse_gpio_read_digital_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_GPIO_READ_ANALOG_IO:
        logger.info("Command Response:         " + parse_gpio_read_analog_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_WRITE_DAC_LTC2633:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_READ_ADC_MAX11614EEE:
        logger.info("Command Response:         " + parse_read_adc_max11614eee_response(response_detail))
    elif subcmd == TESTOP_SUBCMD_UPLOAD_TO_9304:
        global last_upload_response
        last_upload_response=parse_func_test_response(response_detail)
        logger.info("Command Response:         " + last_upload_response)
    elif subcmd == TESTOP_SUBCMD_READ_CRC:
        logger.info("Command Response:         " + parse_func_test_response(response_detail))
    else:
        # If command-specific parsing was not identified but we have detail information
        # format the response in hex
        if response_detail_len > 0:
            logger.info("Command Response:         " + response_detail_string)
        else:
            logger.info("Command Response:         OK")

    return errcode
# ==========================================================================


# ==========================================================================
#   Parse the GET_MEMORY_USAGE response
#  Return Parameters:
#    Bytes 0-3:   Memory Pool Size
#    Bytes 4-7:   Retention Memory Used
#    Bytes 8-11:  Non-retention Memory Used
#    Bytes 12-15: Retention Memory Reserved
# ==========================================================================
def parse_memory_usage_response(response):
    # Parse the response into these globals:
    global last_memusage_memory_pool_size
    global last_memusage_retention_memory_used
    global last_memusage_nonretention_memory_used
    global last_memusage_retention_memory_reserved

    last_memusage_memory_pool_size          = convert_array_to_long(response[0:4])
    last_memusage_retention_memory_used     = convert_array_to_long(response[4:8])
    last_memusage_nonretention_memory_used  = convert_array_to_long(response[8:12])
    last_memusage_retention_memory_reserved = convert_array_to_long(response[12:16])

    return 'MemoryPoolSize='         + str(last_memusage_memory_pool_size)         + '  ' + \
           'RetentionMemoryUsed='    + str(last_memusage_retention_memory_used)    + '  ' + \
           'NonRetentionMemoryUsed=' + str(last_memusage_nonretention_memory_used) + '  ' + \
           'RetentionMemoryReserved='+ str(last_memusage_retention_memory_reserved)

# ==========================================================================
#   Parse the SVLD_MEASUREMENT response
#
# Response:
#   Power Mode (byte 0):
#     0x00 = DCDC Step-Down Configuration.
#     0x01 = DCDC Off Configuration.
#     0x02 = DCDC Step-Up Configuration.
#     0x03 = External DCDC Configuration.
#     0x04 – 0xFF reserved
#   Measured SVLD (byte 1)
# ==========================================================================
def parse_SVLD_response(response):
    # Parse the response into these globals:
    global last_SVLD_power_mode
    global last_SVLD_measurement

    # Extract the return parameters from the response packet
    last_SVLD_power_mode  = response[0]
    last_SVLD_measurement = response[1]


    if   last_SVLD_power_mode == 0:
        power_mode_string = "DCDC Step-Down Configuration"
    elif last_SVLD_power_mode == 1:
        power_mode_string = "DCDC Off Configuration"
    elif last_SVLD_power_mode == 2:
        power_mode_string = "DCDC Step-Up Configuration"
    elif last_SVLD_power_mode == 3:
        power_mode_string = "External DCDC Configuration"
    else:
        power_mode_string = "Invalid"
    last_SVLD_power_mode=power_mode_string
    last_SVLD_measurement=str(last_SVLD_measurement)


    return 'PowerMode=' + power_mode_string + '  SVLDMeasurement=' + str(last_SVLD_measurement)



# ==========================================================================
#   Parse the ProTest version of the SVLD_MEASUREMENT response
#   Same as the EM version, but no Power Mode
#
# Response:
#   Measured SVLD (byte 1)
# ==========================================================================
def parse_protest_SVLD_response(response):
    # Parse the response into these globals:
    global last_SVLD_measurement

    # Extract the return parameters from the response packet
    last_SVLD_measurement = response[0]


    return 'SVLDMeasurement=' + str(last_SVLD_measurement)


# ==========================================================================
#   Parse the CALCULATE_CRC32 response
#  Return Parameters:
#    Bytes 0-3:   CRC32 over the memory range
# ==========================================================================
def parse_calculate_crc32_response(response):
    # Parse the response into these globals:
    global last_calccrc_crc32

    last_calccrc_crc32 = convert_array_to_long(response[0:4])
    return 'CRC32=' + hex(last_calccrc_crc32)

# ==========================================================================
#  Parse the PATCH_QUERY Response
#  Return Parameters:
#    Bytes 0-1:   short int container count
#    Bytes 2-3:   short int transfer count
#    Bytes 4-7:   long  int patch system state
#
#    TODO: parse the rest of the parameters
# ==========================================================================
def parse_patch_query(response):
    # Parse the response into these globals:
    global last_patch_container_count
    global last_patch_transfer_count
    global last_patch_system_state
    global last_patch_address
    global last_patch_size
    global last_patch_CRC32
    global last_patch_build_num
    global last_patch_user_build_num
    global last_patch_container_flags
    global last_patch_container_version
    global last_patch_container_type
    global last_patch_container_id

    last_patch_container_count = convert_array_to_short(response[0:2])
    last_patch_transfer_count  = convert_array_to_short(response[2:4])
    last_patch_system_state     = convert_array_to_long( response[4:8])
    last_patch_address          = convert_array_to_long(response[8:12])
    last_patch_size             = convert_array_to_long(response[12:16])
    last_patch_CRC32            = convert_array_to_long(response[16:20])
    last_patch_build_num        = convert_array_to_short(response[20:22])
    last_patch_user_build_num   = convert_array_to_short(response[22:24])
    last_patch_container_flags  = convert_array_to_hex(response[24:25], 1)
    last_patch_container_version= convert_array_to_hex(response[25:26], 1)
    last_patch_container_type   = convert_array_to_hex(response[26:27], 1)
    last_patch_container_id     = convert_array_to_hex(response[27:28], 1)
    return 'Containers='      + str(last_patch_container_count) +  \
           ' TransferCount=' + str(last_patch_transfer_count)  +  \
           ' State='         + hex(last_patch_system_state)    +  \
           ' Addr='          + hex(last_patch_address)         +  \
           ' Size='          + str(last_patch_size)            +  \
           ' CRC32='         + hex(last_patch_CRC32)           +  \
           ' BuildNum='      + str(last_patch_build_num)       +  \
           ' UserBuildNum='  + str(last_patch_user_build_num) +  \
           ' Flags='         + str(last_patch_container_flags) +  \
           ' Ver='           + str(last_patch_container_version)+ \
           ' Type='          + str(last_patch_container_type)  +  \
           ' ID='            + str(last_patch_container_id)



# ==========================================================================
#   Parse the END_LE_TEST Response and the TRANSMITTER_TEST_END response
#  Return Parameters:
#    Bytes 0-1:   short int count of packets sent/received
# ==========================================================================
def parse_end_test_response(response):
    # Parse the response into these globals:
    global last_test_number_of_packets

    last_test_number_of_packets = convert_array_to_short(response[0:2])
    return 'NumberOfPackets=' + str(last_test_number_of_packets)


# ==========================================================================
#  Parse the response from READ_CURRENT
#  Return Parameters:
#    Formatted string suitable for logging
#  Return global:
#    int last_adc_measurement
# ==========================================================================
def parse_read_current_response(response):
    # Parse the response into these globals:
    global last_adc_measurement

    # The response contains two 32 bit words.
    # The first is the channel; the second word is the measurement
    tmp_last_adc_measurement = convert_array_to_slong(response[4:8])
    if response[0] == 0:
        last_adc_measurement = tmp_last_adc_measurement * 0.00012681845361088004
    elif response[0] == 1:
        last_adc_measurement = tmp_last_adc_measurement * 0.0012681845361088002
    else:
        last_adc_measurement = tmp_last_adc_measurement * 0.012681845361088

    return 'ADCMeasurement CH=' + str(response[0]) + ', Current='+ str(last_adc_measurement)

# ==========================================================================
#  Parse the response from TESTOP_SUBCMD_FUNCTEST_CURRENT_SLEEP
#  Return Parameters:
#    Formatted string suitable for logging
#  Return global:
#    int
# ==========================================================================
def parse_func_test_response(response):

    return convert_bytes_to_string(response)

# ==========================================================================
#  Parse the READ_9304_VER  response
#  Return Parameters:
#    Bytes 0-7:   Version array
#
#  This is the general case of the READ_DUT_VER and READ_REF_VER
#  TODO: deprecate the others
# ==========================================================================
def parse_read_dev_ver_response(response):
    # Parse the response into the global:
    global last_dev_ver_string
    last_dev_ver_string = convert_array_to_hex_no_spaces(response, 8)
    return 'Version=' + last_dev_ver_string


# ==========================================================================
#  Parse the response from READ_ADC
#  Return Parameters:
#    Formatted string suitable for logging
#  Return global:
#    int last_adc_measurement
# ==========================================================================
def parse_read_ADC_response(response):
    global last_adc_measurement

    # The response contains two 32 bit words.
    # The first is the channel; the second word is the measurement
    tmp_last_adc_measurement = convert_array_to_slong(response[4:8])
    if response[0] == 0:
        last_adc_measurement = tmp_last_adc_measurement * 0.00012681845361088004
    elif response[0] == 1:
        last_adc_measurement = tmp_last_adc_measurement * 0.0012681845361088002
    else:
        last_adc_measurement = tmp_last_adc_measurement * 0.012681845361088

    return 'ADCMeasurement CH=' + str(response[0]) + ', Current='+ str(last_adc_measurement)

# ==========================================================================
#  Parse the READ_DUT_VER  response
#  Return Parameters:
#    Bytes 0-7:   Version array
#
# ==========================================================================
def parse_read_dut_ver_response(response):
    # Parse the response into the global:
    global last_dut_ver_string
    last_dut_ver_string = convert_array_to_hex_no_spaces(response, 8)
    return 'Version=' + last_dut_ver_string


# ==========================================================================
#  Parse the READ_REF_VER response
#  Return Parameters:
#    Bytes 0-7:   Version array
#
# ==========================================================================
def parse_read_ref_ver_response(response):
    # Parse the response into the global:
    global last_ref_ver_string
    last_ref_ver_string = convert_array_to_hex_no_spaces(response, 8)
    return 'Version=' + last_ref_ver_string


# ==========================================================================
#  Parse the READ_PTB_FW_VER response
#  Return Parameters:
#    Bytes 0-7:   Version text string
# ==========================================================================
def parse_board_ver_response(response):
    # Parse the response into the global:
    global last_board_ver_string
    last_board_ver_string = convert_bytes_to_string(response[0:8])
    return 'Version=' + last_board_ver_string


# ==========================================================================
#  Parse the READ_PTB_SN response
#  Return Parameters:
#    Bytes 0-31:   Board Serial Number text string
# ==========================================================================
def parse_board_serial_num_response(response):
    # Parse the response into the global:
    global last_board_serial_num_string
    last_board_serial_num_string = convert_bytes_to_string(response[0:32])
    return 'BoardSerialNumber=' + last_board_serial_num_string

# ==========================================================================
#  Parse the READ_AT_ADDR
#  Return Parameters:
#    Bytes 0-7:   Version array
# ==========================================================================
def parse_read_mem_response(response):
    # Parse the response into the global:
    global last_read_mem_string
    last_read_mem_string = convert_array_to_hex(response, len(response))
    return 'ReadMemory=' + last_read_mem_string


# ==========================================================================
#   Parse the SET_POWER_LEVEL_EX Response
#  Return Parameters:
#    Bytes 0:   max power level
# ==========================================================================
def parse_set_power_level_response(response):
    # Parse the response into these globals:
    global last_max_power_level

    # A single byte is returned with the max power level index
    last_max_power_level = response[0]
    return 'MaxPowerLevel=' + str(last_max_power_level)


# ==========================================================================
#   Parse the status packet into formated string
#   See structure definition TESTOP_Status_Packet_s in command_if.h
# ==========================================================================
def parse_status(statusDetail):

    # Format the device state info
    if statusDetail[0] == 0:
        dut_state = "IDLE"
    else:
        dut_state = "BUSY"

    if statusDetail[1] == 0:
        ref_state = "IDLE"
    else:
        ref_state = "BUSY"

    # Extract the command counts for the DUT and REF
    dut_cmd_cnt  = convert_array_to_short(statusDetail[2:4])
    ref_cmd_cnt  = convert_array_to_short(statusDetail[4:6])

    # Format the string
    statusString = "dut_state: " + dut_state + "  ref_state: " + ref_state + \
                   "    dut_cmd_cnt:"   + str(dut_cmd_cnt) + \
                   "    ref_cmd_cnt:"   + str(ref_cmd_cnt)

    return statusString

# ==========================================================================
#   Parse the 'tic count difference' packet into formated string
#   See structure definition TESTOP_Status_Packet_s in command_if.h
# ==========================================================================
def parse_xtalvalidation_response(statusDetail):
    global last_xtal_ref_tics
    global last_xtal_dut_tics

    # Extract the command counts for the REF
    xtal_cnt = convert_array_to_long(statusDetail[0:4])
    last_xtal_ref_tics = xtal_cnt

    # Extract the command counts for the DUT
    xtal_cnt = convert_array_to_long(statusDetail[4:8])
    last_xtal_dut_tics = xtal_cnt

    # Format the string
    statusString = "    Difference in XTAL tics:"   + str(last_xtal_ref_tics - last_xtal_dut_tics)

    return statusString

# ==========================================================================
#   Parse the 'tic count difference' packet into formated string
#   See structure definition TESTOP_Status_Packet_s in command_if.h
# ==========================================================================
def parse_BD_address_response(statusDetail):
    global last_BD_address

    # Extract the BD address [6 bytes]
    last_BD_address = convert_array_to_hex(statusDetail[0:6], 6)

    # Format the string
    statusString = "    BD Address:"   + str(last_BD_address)

    return statusString

# ==========================================================================
#   Parse the 'tic count difference' packet into formated string
#   See structure definition TESTOP_Status_Packet_s in command_if.h
# ==========================================================================
def parse_advertising_report_response(statusDetail):
    global last_advertReport_totalReports
    global last_advertReport_minRssi
    global last_advertReport_maxRssi
    global last_advertReport_aveRssi
    global last_advertReport_lastRssi

    last_advertReport_totalReports = convert_array_to_long(statusDetail[0:4])
    last_advertReport_minRssi = convert_array_to_slong(statusDetail[4:8])
    last_advertReport_maxRssi = convert_array_to_slong(statusDetail[8:12])
    last_advertReport_aveRssi = convert_array_to_slong(statusDetail[12:16])
    last_advertReport_lastRssi = convert_array_to_slong(statusDetail[16:20])

    # Format the string
    statusString = "    Ave RSSI:" + str(last_advertReport_aveRssi) + ", Total Events:" + str(last_advertReport_totalReports)

    return statusString

# ==========================================================================
#   Parse the calibration values
#   3 ints are returned  5ma/100uA/1uA
# ==========================================================================
def parse_calibration_response(statusDetail):
    global last_5mA_calibration
    global last_100uA_calibration
    global last_1uA_calibration

    COUNT_MIN_HIGH=10000
    COUNT_MIN_MID=100
    COUNT_MIN_LOW=1
    COUNT_MAX_HIGH=1000000
    COUNT_MAX_MID=10000
    COUNT_MAX_LOW=100
    ADC_COUNT_DIV=float((2 ** 21) - 1)
    VREF=1.25
    GAIN_HIGH=100
    GAIN_MID=1000
    GAIN_LOW=1000
    RSHUNT_HIGH=47
    RSHUNT_MID=4.7
    RSHUNT_LOW=0.47

    CURRENT_CONVERSION=[(VREF/ADC_COUNT_DIV)/(RSHUNT_HIGH * GAIN_HIGH) * 1000000.0,
                          (VREF / ADC_COUNT_DIV) / (RSHUNT_MID * GAIN_MID) * 1000000.0,
                          (VREF / ADC_COUNT_DIV) / (RSHUNT_LOW * GAIN_LOW) * 1000000.0]

    tmp_5ma_cal = convert_array_to_slong(statusDetail[0:4])
    last_5mA_calibration = tmp_5ma_cal * CURRENT_CONVERSION[0]
    tmp_100ua_cal = convert_array_to_slong(statusDetail[4:8])
    last_100uA_calibration = tmp_100ua_cal * CURRENT_CONVERSION[1]
    tmp_1ua_cal = convert_array_to_slong(statusDetail[8:12])
    last_1uA_calibration = tmp_1ua_cal * CURRENT_CONVERSION[2]

    # Format the string
    statusString = "    5ma:" + str(last_5mA_calibration) + ", 100uA:" + str(last_100uA_calibration) + ", 1uA:" + str(last_1uA_calibration)

    return statusString

# ==========================================================================
#   Parse the isBusy result
#
# ==========================================================================
def parse_is_busy_response(response):
    global last_is_busy

    if response[0] == 0:
        last_is_busy = 0
    elif response[0] == 1:
        last_is_busy = 1

    return "IsBusy = " + str(last_is_busy)

# ==========================================================================
#   Parse the read_results result
#
# ==========================================================================
def parse_read_results_response(response):
    global last_is_busy
    global last_adc_measurement
    global read_results_index
    global last_SVLD_power_mode
    global last_SVLD_measurement
    global triggered_current_values
    global last_PER_value
    global last_ppm

    if response[0] & 0x80 == 0:
        last_is_busy = 0
    else:
        last_is_busy = 1

    if last_is_busy == 0:
        if (response[0] == TESTOP_SUBCMD_FUNCTEST_CURRENT_SLEEP or response[0] == TESTOP_SUBCMD_FUNCTEST_CURRENT_ACTIVE or
                response[0] == TESTOP_SUBCMD_FUNCTEST_CURRENT_RX or response[0] == TESTOP_SUBCMD_FUNCTEST_CURRENT_TX):
            # The response contains two 32 bit words.
            # The first is the channel; the second word is the measurement
            tmp_last_adc_measurement = convert_array_to_slong(response[8:12])
            if response[4] == 0:
                last_adc_measurement = tmp_last_adc_measurement * 0.00012681845361088004
            elif response[4] == 1:
                last_adc_measurement = tmp_last_adc_measurement * 0.0012681845361088002
            else:
                last_adc_measurement = tmp_last_adc_measurement * 0.012681845361088

            return 'ADCMeasurement CH=' + str(response[4]) + ', Current=' + str(last_adc_measurement)

        elif (response[0] == TESTOP_SUBCMD_FUNCTEST_PER_TX or response[0] == TESTOP_SUBCMD_FUNCTEST_PER_RX):

            sent_packets = convert_array_to_long(response[4:0:-1])
            received_packets = convert_array_to_long(response[8:4:-1])

            if (sent_packets != 0):
                packet_error_rate = ((sent_packets - received_packets) / sent_packets) * 100
            else:
                packet_error_rate = 100

            # Output and Log Test
            #printData("%.2f" % (packet_error_rate), "%", "PER_Test", [0, numOfPackets, channel, powerLevel])
            last_PER_value=packet_error_rate
            return 'Packet error rate =' + str(packet_error_rate)

        elif (response[0] == TESTOP_SUBCMD_FUNCTEST_ADVERTISE or response[0] == TESTOP_SUBCMD_FUNCTEST_RSSI):
            totalReports = convert_array_to_long(response[0:4])
            minRssi = convert_array_to_slong(response[4:8])
            maxRssi = convert_array_to_slong(response[8:12])
            aveRssi = convert_array_to_slong(response[12:16])
            lastRssi = convert_array_to_slong(response[16:20])

            # Format the string
            statusString = "    Ave RSSI:" + str(aveRssi) + ", Total Events:" + str(totalReports)
            return statusString

        elif response[0] == TESTOP_SUBCMD_FUNCTEST_XTAL:
            ref_count = convert_array_to_long(response[4:8])

            # Extract the command counts for the DUT
            dut_count = convert_array_to_long(response[8:12])

            try:
                # ppm=10**6-((dut_count/dut_freq)/(ref_count/ref_freq))*10**6
                ppm = 10 ** 6 - (dut_count / ref_count) * 10 ** 6
                if ppm < 0:
                    ppm = ppm * -1
            except:
                ppm = -1

            statusString = "ppm = %.2f"%ppm
            last_ppm=ppm
            return statusString

        elif (response[0] == TESTOP_SUBCMD_FUNCTEST_PWR_MODE or response[0] == TESTOP_SUBCMD_FUNCTEST_SVLD):

            # Extract the return parameters from the response packet
            last_SVLD_power_mode = response[1]
            last_SVLD_measurement = ''.join(chr(i) for i in response[2:10])

            if last_SVLD_power_mode == 0:
                power_mode_string = "DCDC Step-Down Configuration"
            elif last_SVLD_power_mode == 1:
                power_mode_string = "DCDC Off Configuration"
            elif last_SVLD_power_mode == 2:
                power_mode_string = "DCDC Step-Up Configuration"
            elif last_SVLD_power_mode == 3:
                power_mode_string = "External DCDC Configuration"
            else:
                power_mode_string = "Invalid"

            last_SVLD_power_mode=power_mode_string

            if response[0] == TESTOP_SUBCMD_FUNCTEST_PWR_MODE:
                return 'PowerMode=' + power_mode_string + '  SVLDMeasurement=' + str(last_SVLD_measurement)
            elif response[0] == TESTOP_SUBCMD_FUNCTEST_SVLD:
                return 'PowerMode=' + power_mode_string + '  SVLDMeasurement=' + str(last_SVLD_measurement)

        elif response[0] == TESTOP_SUBCMD_MEASURE_TRIGGERED_CURRENT:
            # The response contains a variable number of two 32 bit word pairs.
            # The first is the channel; the second word is the measurement
            responseString = ''

            for x in range(response[1]):
                responseString += '\nADCMeasurement CH=' + str(response[2 + x]) + ', Current='

                tmp_last_adc_measurement = convert_array_to_slong(response[(12 + (x * 4)):(16 + (x * 4))])
                if response[2 + x] == 0:
                    last_adc_measurement = tmp_last_adc_measurement * 0.00012681845361088004
                elif response[2 + x] == 1:
                    last_adc_measurement = tmp_last_adc_measurement * 0.0012681845361088002
                else:
                    last_adc_measurement = tmp_last_adc_measurement * 0.012681845361088

                responseString += str(last_adc_measurement)
                triggered_current_values.append(last_adc_measurement)

            return responseString #'ADCMeasurement CH=' + str(response[4]) + ', Current=' + str(last_adc_measurement)

    return "ReadResults = " + str(last_is_busy)


# ==========================================================================
#   Parse the read_results result
#
# ==========================================================================
def parse_gpio_read_digital_response(response):
    global last_Digital_read
    last_Digital_read=convert_bytes_to_string(response)
    return "GPIO Digital Read = " + convert_bytes_to_string(response)

# ==========================================================================
#   Parse the read_results result
#
# ==========================================================================
def parse_gpio_read_analog_response(response):
    return "GPIO Analog Read = " + convert_bytes_to_string(response)

# ==========================================================================
#   Parse the read_adc_max11614eee result
#
# ==========================================================================
def parse_read_adc_max11614eee_response(response):

    responseString = 'Channel '
    responseString += str(response[0])
    responseString += ' = '

    low_byte = response[2]
    hi_byte = response[1]
    intval = (hi_byte << 8) + low_byte

    responseString += str(intval)
    responseString += ' millivolts'

    return responseString

# ==========================================================================
#     Connect to the PTB Device
#
# ==========================================================================
def connect_to_device(serial_num):
    global dev

    # Find the production test DVK device.
    # If a serial number was provided then use it
    if serial_num is None :
        # Don't bother with the serial number match
        dev = usb.core.find(idVendor=DVK_USB_VID, idProduct=DVK_USB_PID)
    elif serial_num == "" :
        # Don't bother with the serial number match
        dev = usb.core.find(idVendor=DVK_USB_VID, idProduct=DVK_USB_PID)
    else:
        dev = usb.core.find(idVendor=DVK_USB_VID, idProduct=DVK_USB_PID,
            custom_match=lambda d: d.serial_number == serial_num)


    if dev is None:
        logger.error(    'Device not found with VID/PID/SeriaNum attributes')
        raise ValueError('Device not found with VID/PID/SeriaNum attributes')

    last_ptb_product_name_string = usb.util.get_string(dev, DVK_USB_PRODUCT_NAME_STRING_DESCRIPTOR_INDEX)
    last_ptb_serial_num_string = usb.util.get_string(dev, DVK_USB_SERIAL_NUM_STRING_DESCRIPTOR_INDEX)
    logger.info('Board Name: ' + last_ptb_product_name_string + '  Board Serial Number:' + last_ptb_serial_num_string)

    # Start in the default TestOp mode
    switch_to_testop_mode()

    return last_ptb_serial_num_string

def disconnect_proDVK():
    global dev
    usb.util.dispose_resources(dev)
    return

# ==========================================================================
#   Exit the current test script
#     exit code = 0 if no errors occured, otherwise exit code = 1
# ==========================================================================
def exit_test():
    if get_test_error_count() > 0:
        sys.exit(1)
    else:
        sys.exit(0)


# ==========================================================================
# That's all, folks.
