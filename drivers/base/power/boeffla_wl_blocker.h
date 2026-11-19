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
 * vendor-namespaced (inert where the SoC differs) or a WLAN/BT background
 * idle-poll wakelock (connectivity still works; only the high-frequency
 * firmware poll that keeps the SoC out of deep sleep is suppressed).
 * No alarmtimer/[timerfd] -- those break RTC alarms and their numbering
 * is not portable. No input/i2c/spi nodes -- those break wake-on-touch.
 * No modem keep-alive (mtk_md, mtk_md_wl) -- the 5G modem requires the
 * link to stay up through idle; blocking it drops the data connection.
 * Per-device offenders go in the runtime sysfs 'wakelock_blocker' node.
 *
 * Q6BIOS_WLAN : QCOM WLAN firmware idle poll (WiFi stays associated)
 * mtk_wmt_top : MTK Wireless Module Top idle poll (inert on QCOM)
 * btsdio_wl   : QCOM/MTK BT controller idle poll (BT stays paired)
 * mgmt_txrx   : WLAN mgmt-frame TX/RX idle poll -- beacon/probe/assoc
 *               churn; association is kept by the firmware WoW offload, a
 *               real deauth/wake still fires via the hard-event path.
 */
#define LIST_WL_DEFAULT \
	"DIAG_WS;wlan_extscan_wl;wlan_pno_wl;wlan_wow_wl;" \
	"a600000.ssusb;998000.qcom,qup_uart;hal_bluetooth_lock;bluetooth_timer;" \
	"Q6BIOS_WLAN;mtk_wmt_top;btsdio_wl;mgmt_txrx;" \
	"wlan_multicast_wl;wlan_multicast;WifiMulticast;WifiMulticastKeeper;*wal*;" \
	"qcom_vad_wakelock;qcom-vad;wlan_pno_rx_wl;wlan_scan_wl;" \
	"mi_connect;mi_link;MiSmartHub;MiShare;AnalyticsCore;CloudService;FindDevice"
/*
 * fpc_ttw_wl (fingerprint touch-to-wake) was added here to silence pocket
 * wakes, but it breaks the primary screen-off wake path on in-display
 * fingerprint devices: the FPC sensor is the T2W sensor, and the blocker
 * kills its activation, so the first tap on an off screen does not
 * register. Pocket-wake complaints get it via the runtime
 * 'wakelock_blocker' sysfs node instead.
 */

#define LENGTH_LIST_WL				1024
/* Both lists are sysfs-writable, so both get the full capacity. Deriving this
 * from strlen(LIST_WL_DEFAULT) is not a constant expression and capped any
 * replacement at the compiled default's length. */
#define LENGTH_LIST_WL_DEFAULT			LENGTH_LIST_WL
/* ";default;user;" -- three delimiters plus the terminator. */
#define LENGTH_LIST_WL_SEARCH			(LENGTH_LIST_WL_DEFAULT + LENGTH_LIST_WL + 4)
