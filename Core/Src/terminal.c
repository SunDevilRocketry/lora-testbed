/*******************************************************************************
*
* FILE: 
*       terminal.c
*
* DESCRIPTION: 
*		Terminal command loop for reciever.
*
*******************************************************************************/


/*------------------------------------------------------------------------------
 Standard Includes                                                                     
------------------------------------------------------------------------------*/


/*------------------------------------------------------------------------------
 Project Includes                                                                     
------------------------------------------------------------------------------*/

/* Pin definitions and main prototypes */
#include "main.h"

/* SDR Modules */
#include "math_sdr.h"
#include "usb.h"
#include "commands.h"
#include "error_sdr.h"
#include "lora.h"
#include "telemetry.h"

/*------------------------------------------------------------------------------
 Globals                                                                    
------------------------------------------------------------------------------*/
extern LORA_PRESET lora_preset;
extern uint8_t usb_tx_byte[ USB_BUF_SIZE ];
extern uint8_t usb_rx_byte[ USB_BUF_SIZE ];
extern bool start_lora;
extern LORA_MESSAGE last_lora_message;

/*------------------------------------------------------------------------------
 Procedures                                                 
------------------------------------------------------------------------------*/

USB_STATUS terminal_loop
	(
	void
	)
{
/*------------------------------------------------------------------------------
 Local Variables 
------------------------------------------------------------------------------*/
uint8_t     command_code = usb_rx_byte[0];             /* Command opcode              */
static USB_STATUS usb_status = USB_OK;  			   /* Status of USB module        */                

/*------------------------------------------------------------------------------
 Terminal Handler                                                                  
------------------------------------------------------------------------------*/

if ( usb_status == USB_OK )
	{
	switch ( command_code )
		{
		/*-------------------------------------------------------------
			CONNECT_OP	
		-------------------------------------------------------------*/
		case CONNECT_OP:
			{
			/* Send firmware identifying code */
            usb_tx_byte[0] = 0x10;
            usb_tx_byte[1] = FIRMWARE_RECEIVER;
			usb_status = usb_transmit_IT( usb_tx_byte, 
						                  sizeof( uint8_t ) * 2 );
			break;
			} /* CONNECT_OP */
		/*-------------------------------------------------------------
			DASHBOARD_OP	
		-------------------------------------------------------------*/
		case DASHBOARD_OP:
			{
            /* begin LoRa polling mode*/
            start_lora = true;

			/* Get dashboard data */
			memcpy(usb_tx_byte, &last_lora_message, LORA_MESSAGE_SIZE);

            /* transmit */
            usb_status = usb_transmit_IT(usb_tx_byte, LORA_MESSAGE_SIZE);
			break;
			} /* DASHBOARD_OP */
		/*-------------------------------------------------------------
			Unrecognized command code  
		-------------------------------------------------------------*/
		default:
			{
			break;
			}

		} /* switch( usb_rx_data ) */
	} /* if ( usb_status == USB_OK ) */

return usb_status;

} /* terminal_loop */


/*******************************************************************************
* END OF FILE                                                                  *
*******************************************************************************/