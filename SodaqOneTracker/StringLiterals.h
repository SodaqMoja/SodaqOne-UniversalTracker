/*
* Copyright (c) 2015 SODAQ. All rights reserved.
*
* This file is part of MicrochipLoRaWAN.
*
* MicrochipLoRaWAN is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of
* the License, or(at your option) any later version.
*
* MicrochipLoRaWAN is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with MicrochipLoRaWAN.  If not, see
* <http://www.gnu.org/licenses/>.
*/

#ifndef _STRINGLITERALS_h
#define _STRINGLITERALS_h

#define CRLF "\r\n"

#define STR_RESULT_OK "ok"
#define STR_RESULT_INVALID_PARAM "invalid_param"
#define STR_RESULT_MAC_ERROR "mac_err"
#define STR_RESULT_MAC_RX "mac_rx"
#define STR_RESULT_MAC_TX_OK "mac_tx_ok"

#define STR_RESULT_NOT_JOINED "not_joined"
#define STR_RESULT_NO_FREE_CHANNEL "no_free_ch"
#define STR_RESULT_SILENT "silent"
#define STR_RESULT_FRAME_COUNTER_ERROR "frame_counter_err_rejoin_needed"
#define STR_RESULT_BUSY "busy"
#define STR_RESULT_MAC_PAUSED "mac_paused"
#define STR_RESULT_INVALID_DATA_LEN "invalid_data_len"

#define STR_CMD_RESET "sys reset"
#define STR_DEVICE_TYPE_RN "RN"
#define STR_DEVICE_TYPE_RN2483 "RN2483"
#define STR_DEVICE_TYPE_RN2903 "RN2903"

#define STR_CMD_SET "mac set "
#define STR_RETRIES "retx "
#define STR_DEV_ADDR "devaddr "
#define STR_APP_SESSION_KEY "appskey "
#define STR_NETWORK_SESSION_KEY "nwkskey "
#define STR_DEV_EUI "deveui "
#define STR_APP_EUI "appeui "
#define STR_APP_KEY "appkey "
#define STR_ADR "adr "
#define STR_PWR_IDX "pwridx "
#define STR_DATARATE "dr "

#define STR_CMD_JOIN "mac join "
#define STR_OTAA "otaa"
#define STR_ABP "abp"
#define STR_ACCEPTED "accepted"

#define STR_CMD_MAC_TX "mac tx "
#define STR_CONFIRMED "cnf "
#define STR_UNCONFIRMED "uncnf "

#define STR_CMD_SLEEP "sys sleep 259200000" // 3 days
#define STR_CMD_GET_HWEUI "sys get hweui"
#define STR_CMD_SET_CHANNEL_STATUS "mac set ch status "

#endif
