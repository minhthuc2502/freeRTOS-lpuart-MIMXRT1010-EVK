#include "host_hid_joystick.h"
#include "usb_host.h"
#include "usb_host_hid.h"
#include "usb_host_config.h"

/**
 * Function process data recieved from joystick
*/
static void usb_host_joystick_process_buffer(usb_host_hid_joystick_instance_t *joystickHandle);

/**
 * Function callback to signal to the return of the functions control usb device (ex. joystick)
*/
static void usb_host_joystick_control_callback(void *param, uint8_t *data, uint32_t dataLen, usb_status_t status);

/*******************************************************************************
 * Variables
 ******************************************************************************/
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t inBuffer[HID_IN_BUFFER_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
uint8_t outBuffer[HID_OUT_BUFFER_SIZE];

/*******************************************************************************
 * Prototype
 ******************************************************************************/
static void usb_host_joystick_process_buffer(usb_host_hid_joystick_instance_t *joystickHandle)
{
    joystickHandle->joystickInBuffer[joystickHandle->inMaxPacketSize] = 0;
    usb_echo("%s", joystickHandle->joystickInBuffer); 
}

static void usb_host_joystick_control_callback(void *param, uint8_t *data, uint32_t dataLen, usb_status_t status)
{
    usb_host_hid_joystick_instance_t *joystickInstance = (usb_host_hid_joystick_instance_t*)param;
    if (status == kStatus_USB_TransferStall)
    {
        usb_echo("this device is not supported this request \r\n");
    }
    else if (status != kStatus_USB_Success)
    {
        usb_echo("control request failed \r\n");
    }
    else {}
    if (joystickInstance->runWaitState == USB_HostHidRunWaitSetInterface)
    {
        joystickInstance->runState = USB_HostHidRunSetInterfaceDone;
    }
    else if (joystickInstance->runWaitState == USB_HostHidRunWaitSetIdle)
    {
        joystickInstance->runState = USB_HostHidRunSetIdleDone;
    }
    else if (joystickInstance->runWaitState == USB_HostHidRunWaitGetReportDescriptor)
    {
        joystickInstance->runState = USB_HostHidRunGetReportDescriptorDone;
    }
    else if (joystickInstance->runWaitState == USB_HostHidRunWaitSetProtocol)
    {
        joystickInstance->runState = USB_HostHidRunSetProtocolDone;
    }
    else {}
}

