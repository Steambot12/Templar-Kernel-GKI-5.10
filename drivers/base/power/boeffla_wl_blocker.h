/*
 * Author: andip71, 01.09.2017
 *
 * Version 1.2.0
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define BOEFFLA_WL_BLOCKER_VERSION	"1.2.0"

/*
 * Default block list. Portable across MTK/QCOM: every entry is either
 * vendor-namespaced (inert where the SoC differs) or a WLAN background-offload
 * wakelock (WiFi still associates on wake; only background scan/roam wakeups
 * are suppressed). No device-specific input/i2c/spi nodes and no
 * alarmtimer/[timerfd] -- those break wake-on-input and RTC alarms, and their
 * numbering is not portable. Per-device offenders go in the runtime sysfs
 * 'wakelock_blocker' node instead.
 */
#define LIST_WL_DEFAULT				"RMNET_DFC;DIAG_WS;qcom_rx_wakelock;wlan;wlan_wow_wl;wlan_extscan_wl;wlan_pno_wl;wlan_roam_wl;wlan_ipa;netmgr_wl;NETLINK;a600000.ssusb;998000.qcom,qup_uart;hal_bluetooth_lock;IPA_WS;IPA_CLIENT_APPS_WAN_COAL_CONS;IPA_CLIENT_APPS_WAN_LOW_LAT_CONS;IPA_CLIENT_APPS_LAN_CONS;rmnet_ipa%d;rmnet_ctl;RMNET_SHS"

#define LENGTH_LIST_WL				1024
/* Both lists are sysfs-writable, so both get the full capacity. Deriving this
 * from strlen(LIST_WL_DEFAULT) is not a constant expression and capped any
 * replacement at the compiled default's length. */
#define LENGTH_LIST_WL_DEFAULT			LENGTH_LIST_WL
/* ";default;user;" -- three delimiters plus the terminator. */
#define LENGTH_LIST_WL_SEARCH			(LENGTH_LIST_WL_DEFAULT + LENGTH_LIST_WL + 4)
