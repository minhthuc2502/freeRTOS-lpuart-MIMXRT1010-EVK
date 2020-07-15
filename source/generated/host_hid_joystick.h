#ifndef _HOST_HID_JOYSTICK_H_
#define _HOST_HID_JOYSTICK_H_
#include "usb_host.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief size of buffer for receiving report descriptor and data */
#define HID_IN_BUFFER_SIZE 100U
/*! @brief size of buffer for sending data */
#define HID_OUT_BUFFER_SIZE 8U
/*! @brief host app run status */
typedef enum usb_host_hid_run_state
{
    USB_HostHidRunIdle = 0,                /*!< idle */
    USB_HostHidRunSetInterface,            /*!< execute set interface code */
    USB_HostHidRunWaitSetInterface,        /*!< wait set interface done */
    USB_HostHidRunSetInterfaceDone,        /*!< set interface is done, execute next step */
    USB_HostHidRunWaitSetIdle,             /*!< wait set idle done */
    USB_HostHidRunSetIdleDone,             /*!< set idle is done, execute next step */
    USB_HostHidRunWaitGetReportDescriptor, /*!< wait get report descriptor done */
    USB_HostHidRunGetReportDescriptorDone, /*!< get report descriptor is done, execute next step */
    USB_HostHidRunWaitSetProtocol,         /*!< wait set protocol done */
    USB_HostHidRunSetProtocolDone,         /*!< set protocol is done, execute next step */
    USB_HostHidRunWaitDataReceived,        /*!< wait interrupt in data */
    USB_HostHidRunDataReceived,            /*!< interrupt in data received */
    USB_HostHidRunPrimeDataReceive,        /*!< prime interrupt in receive */
} usb_host_hid_run_state_t;

/*! @brief USB host hid for joystick instance structure */
typedef struct  usb_host_hid_joystick_instance
{
    usb_host_configuration_handle configHandle;
    usb_host_interface_handle interfaceHandle;
    usb_host_class_handle classHandle;
    usb_device_handle deviceHandle;
    uint8_t deviceState;                           /*!< device attach/detach status */
    uint8_t devicePreviousState;                   /*!< device attach/detach previous status */
    uint8_t runState;                              /*!< run state of device attached */
    uint8_t runWaitState;                          /*!< run wait state of device attached */
    uint16_t inMaxPacketSize;                      /*!< Interrupt in max packet size */
    uint16_t outMaxPacketSize;                     /*!< Interrupt out max packet size */
    uint8_t *joystickInBuffer;                     /*!< Buffer for in pipe  */
    uint8_t *joystickOutBuffer;                    /*!< Buffer for out pipe  */
    uint16_t sendIndex;                            /*!< data sending position */
} usb_host_hid_joystick_instance_t;

/*! @brief This function implements the host hid action, it defines which action that the host does */
void usb_host_joystick_task(void *param);

/*! @brief This function handle the event attach/dettach. With a defined event, this function can allocate or free ressources */
usb_status_t usb_host_joystick_event(usb_device_handle deviceHandle, usb_host_configuration_handle configurationhandle, uint32_t eventcode);


#endif
