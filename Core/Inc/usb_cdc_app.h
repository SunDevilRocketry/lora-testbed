/**
 * Application glue for STM32 USB CDC (virtual COM) used from Core/Src/main.c.
 * Not part of the shared driver/usb API.
 */
#ifndef USB_CDC_APP_H
#define USB_CDC_APP_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Invoked from the USB ISR when an interrupt-mode RX finishes (see usb_receive_IT). */
void usb_receive_complete_callback( void *data, size_t len );

/* Invoked from the USB ISR when an interrupt-mode TX finishes (see usb_transmit_IT). */
void usb_transmit_complete_callback( void *data, size_t len );

/* Feeds one OUT packet from CDC_Receive_FS into the RX state machine. */
void usb_process_cdc_rx( uint8_t *buf, uint32_t len );

/* Called from usbd_cdc when the bulk IN transfer is done. */
void USBD_App_CDC_TxComplete( void );

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_APP_H */
