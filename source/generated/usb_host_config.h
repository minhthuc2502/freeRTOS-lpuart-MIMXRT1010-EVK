#ifndef _USB_HOST_CONFIG_HID_H_
#define _USB_HOST_CONFIG_HID_H_

/*!
 * @brief host HID class instance count, meantime it indicates HID class enable or disable.
 */
#define USB_HOST_CONFIG_HID (1U)

/*!
 * @brief the max endpoint for one interface.
 * the max endpoint descriptor number that one interface descriptor contain.
 */
#define USB_HOST_CONFIG_INTERFACE_MAX_EP 2U
/*!
 * @brief the max interface for one configuration.
 * the max interface descriptor number that one configuration descriptor can contain.
 */
#define USB_HOST_CONFIG_CONFIGURATION_MAX_INTERFACE 1U
/*!
 * @brief host transfer max count.
 * transfer is the host driver resource for data transmission mission, one transmission mission need one transfer.
 */
#define USB_HOST_CONFIG_MAX_TRANSFERS 16U
/*!
 * @brief the max power for one device.
 * the max power the host can provide for one device.
 */
#define USB_HOST_CONFIG_MAX_POWER 500
/*!
 * @brief the max NAK count for one transaction.
 * when nak count reach to the value, the transaction fail.
 */
#define USB_HOST_CONFIG_MAX_NAK 3000U
/*!
 * @brief host driver instance max count.
 * for example: 2 - one for khci, one for ehci.
 */
#define USB_HOST_CONFIG_MAX_HOST  1
/*!
 * @brief the max retries for enumeration setup stall.
 * the max times for one transfer can stall.
 */
#define USB_HOST_CONFIG_ENUMERATION_MAX_STALL_RETRIES 1U
/*!
 * @brief the max retries for enumeration.
 * retry time when enumeration fail.
 */
#define USB_HOST_CONFIG_ENUMERATION_MAX_RETRIES 3U

#endif