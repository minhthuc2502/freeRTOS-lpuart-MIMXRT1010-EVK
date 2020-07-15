#include "host_hid_joystick.h"
#include "usb_host.h"
#include "usb_host_hid.h"
#include "usb_host_config.h"
#include "freertos_lpuart.h"
/**
 * Function process data recieved from joystick
*/
static void usb_host_joystick_process_buffer(usb_host_hid_joystick_instance_t *joystickHandle);

/**
 * Function callback to signal to the return of the functions control usb device (ex. joystick)
*/
static void usb_host_joystick_control_callback(void *param, uint8_t *data, uint32_t dataLen, usb_status_t status);

/**
 * This function is used as callback function when call USB_HostHidRecv .
 * param: the host hid of device (joystick)
 * status: transfer result status
*/
static void usb_host_joystick_in_callback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status);

/**
 * This function is used as callback function when call USB_HostHidRecv .
 * param: the host hid of device (joystick)
 * status: transfer result status
*/
//static void usb_host_joystick_out_callback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status);

/*! @brief This function implements the host hid action, it defines which action that the host does */
void usb_host_joystick_task(void *param);

/*! @brief This function handle the event attach/dettach. With a defined event, this function can allocate or free ressources */
usb_status_t usb_host_joystick_event(usb_device_handle deviceHandle, usb_host_configuration_handle configurationhandle, uint32_t eventcode);
/*******************************************************************************
 * Variables
 ******************************************************************************/
usb_host_hid_joystick_instance_t g_HostJoystick;    // instance of joystick
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
    if (joystickInstance->runWaitState == USB_HostHidRunWaitSetInterface)           // Prepare for state set interface
    {
        joystickInstance->runState = USB_HostHidRunSetInterfaceDone;
    }
    else if (joystickInstance->runWaitState == USB_HostHidRunWaitSetIdle)           // Prepare for state set idle: tell joystick respond only when there is a new event occurs or respond any time if an event occurs
    {
        joystickInstance->runState = USB_HostHidRunSetIdleDone;
    }
    else if (joystickInstance->runWaitState == USB_HostHidRunWaitGetReportDescriptor)     // Prepare for state get report Descriptor: rules of package data. for an event on joystick, it send data to host with a report changed. 
    {
        joystickInstance->runState = USB_HostHidRunGetReportDescriptorDone;
    }
    else if (joystickInstance->runWaitState == USB_HostHidRunWaitSetProtocol)       // Prepare for state set protocol: set boot protocol or report protocol
    {
        joystickInstance->runState = USB_HostHidRunSetProtocolDone;
    }
    else {}
}

static void usb_host_joystick_in_callback(void *param, uint8_t *data, uint32_t dataLength, usb_status_t status)
{
    usb_host_hid_joystick_instance_t *instance = (usb_host_hid_joystick_instance_t*) param;
    if (instance->runWaitState == USB_HostHidRunWaitDataReceived)
    {
        if (status == kStatus_USB_Success)
        {
            instance->runState = USB_HostHidRunDataReceived;
        }
        else
        {
            if (instance->deviceState == kStatus_DEV_Attached)
            {
                instance->runState = USB_HostHidRunPrimeDataReceive;
            }
        }
    }
}

void usb_host_joystick_task(void *param)
{
    usb_host_hid_descriptor_t *hidDescriptor;
    uint32_t hidReportLength = 0;
    uint8_t *descriptor;
    uint32_t endPosition;
    usb_host_hid_joystick_instance_t *instance = (usb_host_hid_joystick_instance_t*)param;
    // for each times state changes, process one for each state
    if (instance->deviceState != instance->devicePreviousState)
    {
        instance->devicePreviousState = instance->deviceState;
        switch (instance->deviceState)
        {
        case kStatus_DEV_Idle:
            break;
        case kStatus_DEV_Attached:
            instance->runState = USB_HostHidRunSetInterface;
            if (USB_HostHidInit(instance->deviceHandle, instance->classHandle) != kStatus_USB_Success)
            {
                usb_echo("host hid initialize fail!\r\n");
            }
            else
            {
                usb_echo("host hid initialize succes!\r\n");
            }
            instance->sendIndex = 0;
            break;
        case kStatus_DEV_Detached:
            instance->runState = USB_HostHidRunIdle;
            instance->deviceState = kStatus_DEV_Idle;
            USB_HostHidDeinit(instance->deviceHandle, instance->classHandle);
            instance->classHandle = NULL;
            usb_echo("host hid released!\r\n");
            break;
        default:
            break;
        }
    }
    switch (instance->runState)
    {
    case USB_HostHidRunIdle:
        break;
    case USB_HostHidRunSetInterface:
        instance->runWaitState = USB_HostHidRunWaitSetInterface;
        instance->runState = USB_HostHidRunIdle;
        // bind interface handler to device information (device information recieved after event attached)
        if (USB_HostHidSetInterface(instance->classHandle, instance->interfaceHandle, 0, usb_host_joystick_control_callback, instance) != kStatus_USB_Success)
        {
            usb_echo("Set interface error\r\n");
        }
        break;
    case USB_HostHidRunSetInterfaceDone:
        instance->inMaxPacketSize = USB_HostHidGetPacketsize(instance->classHandle, USB_ENDPOINT_INTERRUPT, USB_IN);
        instance->outMaxPacketSize = USB_HostHidGetPacketsize(instance->classHandle, USB_ENDPOINT_INTERRUPT, USB_OUT);
        instance->runState = USB_HostHidRunWaitSetIdle;
        instance->runWaitState = USB_HostHidRunIdle;
        // report id: 0 -> apply report rate for all report
        // report id: != 0 -> apply report rate for just the report id defined
        // report rate: the frequency at which device report data when no events have occured
        // for joystick, report rate = ? ~ infinity
        if (USB_HostHidSetIdle(instance->classHandle, 0, 0, usb_host_joystick_control_callback, instance) != kStatus_USB_Success)
        {
            usb_echo("Set idle error\r\n");
        }
    case USB_HostHidRunSetIdleDone:
        instance->runWaitState = USB_HostHidRunWaitGetReportDescriptor;
        instance->runState = USB_HostHidRunIdle;
        hidDescriptor = NULL;
        descriptor = (uint8_t*)((usb_host_interface_t *)instance->interfaceHandle)->interfaceExtension;        // pointer to descriptor in interface
        endPosition = (uint32_t)descriptor + (uint16_t)((usb_host_interface_t *)instance->interfaceHandle)->interfaceExtensionLength; // the position of the last descriptor
        while ((uint32_t)descriptor < endPosition)
        {
            if (*(descriptor + 1) == USB_DESCRIPTOR_TYPE_HID)       // type descriptor is in index +1: see usb_host_hid_descriptor_t
            {
                hidDescriptor = (usb_host_hid_descriptor_t*)descriptor;
            }
            else
            {
                descriptor = (uint8_t*)((uint32_t)descriptor + (*descriptor));  // next descriptor: current address + length of descriptor
            }
        }
        if (hidDescriptor != NULL)
        {
            usb_host_hid_class_descriptor_t *hidClassDescriptor;
            hidClassDescriptor = (usb_host_hid_class_descriptor_t*)&hidDescriptor->bHidDescriptorType;  // pointer hidClassDescriptor to address of bHidDescriptorType
            for (uint8_t i = 0; i < hidDescriptor->bNumDescriptors; ++i)
            {
                hidClassDescriptor += i;
                if (hidClassDescriptor->bHidDescriptorType == USB_DESCRIPTOR_TYPE_HID_REPORT)
                {
                    hidReportLength = (uint16_t)USB_SHORT_FROM_LITTLE_ENDIAN_ADDRESS(hidClassDescriptor->wDescriptorLength);  // get length of report
                    break;
                }
            }
        }
        if (hidReportLength > HID_IN_BUFFER_SIZE)
        {
            usb_echo("hid buffer is too small\r\n");
            return;
        }
        USB_HostHidGetReportDescriptor(instance->classHandle, instance->joystickInBuffer, hidReportLength, usb_host_joystick_control_callback, instance);
        break;
    case USB_HostHidRunGetReportDescriptorDone:
        instance->runWaitState = USB_HostHidRunWaitSetProtocol;
        instance->runState = USB_HostHidRunIdle;
        if (USB_HostHidSetProtocol(instance->classHandle, USB_HOST_HID_GET_PROTOCOL, usb_host_joystick_control_callback, instance) != kStatus_USB_Success)
        {
            usb_echo("get protocol error\r\n");
        }
        break;
    case USB_HostHidRunSetProtocolDone:                // start receive the first time data from device
        instance->runWaitState = USB_HostHidRunWaitDataReceived;
        instance->runState = USB_HostHidRunIdle;
        if (USB_HostHidRecv(instance->classHandle, instance->joystickInBuffer, instance->inMaxPacketSize, usb_host_joystick_in_callback, instance) != kStatus_USB_Success)
        {
            usb_echo("get data error\r\n");
        }
        break;
    case USB_HostHidRunDataReceived:                   // process data (print, or create command to arm), polling to receive next data, This case run only host receive successfully data
        usb_host_joystick_process_buffer(instance);
        instance->runWaitState = USB_HostHidRunWaitDataReceived;
        instance->runState = USB_HostHidRunIdle;
        if (USB_HostHidRecv(instance->classHandle, instance->joystickInBuffer, instance->inMaxPacketSize, usb_host_joystick_in_callback, instance) != kStatus_USB_Success)
        {
            usb_echo("get data error\r\n");
        }
    case USB_HostHidRunPrimeDataReceive:               // just receive data, this case is polled many time until receive succesfully data from device. When data receive sucessfully, USB_HostHidRunDataReceived is active 
        instance->runWaitState = USB_HostHidRunWaitDataReceived;
        instance->runState = USB_HostHidRunIdle;
        if (USB_HostHidRecv(instance->classHandle, instance->joystickInBuffer, instance->inMaxPacketSize, usb_host_joystick_in_callback, instance) != kStatus_USB_Success)
        {
            usb_echo("get data error\r\n");
        }
    default:
        break;
    }
}

usb_status_t usb_host_joystick_event(usb_device_handle deviceHandle, usb_host_configuration_handle configurationhandle, uint32_t eventcode)
{
    usb_host_configuration_t *configuration;
    usb_host_interface_t *interface;
    uint8_t id;
    uint32_t pid, vid, infovalue;
    usb_status_t status = kStatus_USB_Success; 
    switch (eventcode)
    {
        case kUSB_HostEventAttach:
            configuration = (usb_host_configuration_t*) configurationhandle;
            for (uint8_t index = 0; index < configuration->interfaceCount; index++)
            {
                interface = &configuration->interfaceList[index];
                id = interface->interfaceDesc->bInterfaceClass;
                if (id != USB_HOST_HID_CLASS_CODE)
                {
                    continue;
                }
                if (interface->interfaceDesc->bInterfaceSubClass != USB_HOST_HID_SUBCLASS_CODE_NONE)
                {
                    continue;
                }
                USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDevicePID, &pid);      // get id product
                USB_HostHelperGetPeripheralInformation(deviceHandle, kUSB_HostGetDeviceVID, &vid);      // get id vendor
                if ((pid == 0x0268) && (vid == 0x054c))
                {
                    if (g_HostJoystick.deviceState == kStatus_DEV_Idle)
                    {
                        g_HostJoystick.joystickInBuffer = inBuffer;
                        g_HostJoystick.joystickOutBuffer = outBuffer;
                        g_HostJoystick.deviceHandle = deviceHandle;
                        g_HostJoystick.configHandle = configurationhandle;
                        g_HostJoystick.interfaceHandle = interface;
                        return kStatus_USB_Success;
                    }
                    else
                    {
                        continue;
                    }
                }
            }
            status = kStatus_USB_NotSupported;
            break;
        case kUSB_HostEventNotSupported:
            break;
        case kUSB_HostEventEnumerationDone:         // This event is generated if kStatus_USB_Success in event attach is returned to host stack
            if (g_HostJoystick.configHandle == configurationhandle)
            {
                if (g_HostJoystick.interfaceHandle != NULL && g_HostJoystick.deviceHandle != NULL)
                {
                    g_HostJoystick.deviceState = kStatus_DEV_Attached;
                    // The device is enumerated
                    USB_HostHelperGetPeripheralInformation(g_HostJoystick.deviceHandle, kUSB_HostGetDevicePID, &infovalue);
                    usb_echo("Playstation PS3 attached: pid=0x%x", infovalue);
                    USB_HostHelperGetPeripheralInformation(g_HostJoystick.deviceHandle, kUSB_HostGetDeviceVID, &infovalue);
                    usb_echo(" vid=0x%x", infovalue);
                    USB_HostHelperGetPeripheralInformation(g_HostJoystick.deviceHandle, kUSB_HostGetDeviceAddress, &infovalue);
                    usb_echo("address = %d\r\n",infovalue);
                }
                else
                {
                    usb_echo("not idle joystick instance\r\n");
                    return kStatus_USB_Error;
                }
            }
            break;
        case kUSB_HostEventDetach:
            if (g_HostJoystick.configHandle == configurationhandle)
            {
                g_HostJoystick.configHandle = NULL;
                if (g_HostJoystick.interfaceHandle != NULL)
                    g_HostJoystick.interfaceHandle = NULL;
                // device deatached
                if (g_HostJoystick.deviceState !=  kStatus_DEV_Idle)
                    g_HostJoystick.deviceState = kStatus_DEV_Detached;
            }
            break;
        default:
            break;
    }
    return status;
}
