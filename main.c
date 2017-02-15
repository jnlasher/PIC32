// Name:            Joel Lasher
// ECE 2534
// Purpose:         Uses a sinusoidal wave at 20000 Hz and utilizes pulse width
//					modulation to change the duty cycle of the wave. Song data is
//					written to the EEPROM.
//                  
// Resources:       Uses timer2 with 50 microsecond interrupts to update the
//					sinusoid and modulate the frequency
// Written by:      JNL

#include <plib.h>
#include "delay.h"

// Cerebot board configuration
#pragma config ICESEL       = ICS_PGx1  // ICE/ICD Comm Channel Select
#pragma config DEBUG        = OFF       // Debugger Disabled for Starter Kit
#pragma config FNOSC        = PRIPLL	// Oscillator selection
#pragma config POSCMOD      = XT	// Primary oscillator mode
#pragma config FPLLIDIV     = DIV_2	// PLL input divider
#pragma config FPLLMUL      = MUL_20	// PLL multiplier
#pragma config FPLLODIV     = DIV_1	// PLL output divider
#pragma config FPBDIV       = DIV_8	// Peripheral bus clock divider
#pragma config FSOSCEN      = OFF	// Secondary oscillator enable

// EEPROM configuration
#define SYS_FREQ (80000000L)
#define MASTER_MODULE I2C2
#define EEPROM_WRITE_ADDRESS 0xA0 // 0b1010000 Serial EEPROM address
#define EEPROM_READ_ADDRESS 0xA1 // 0b1010000 Serial EEPROM address
#define GetPBClock() 10000000
#define PBCLK GetPBClock()

// UART configuration
#define GetPeripheralClock()        (SYS_FREQ/(1 << OSCCONbits.PBDIV))

// Global variables
unsigned int sec50u; // This is updated 20000 times per second by interrupt handler
unsigned int note_time;
unsigned int frequency = 440;
UINT16 mem_loc = 0x0000; //start address of EEPROM data
UINT16 mem_end = 0x0000; //last address of EEPROM data
UINT16 mem_temp = 0x0000; //temporary memory address
BOOL store_flag = FALSE; //flag when UART input starts
BOOL end_flag = FALSE; //flag when UART input ends

// Function declarations
void InterruptHandler( void );
BOOL SendEEPROMByte(BYTE data);
BYTE ReadEEPROMByte();

//main function
int main()
{
   // System configuration
   SYSTEMConfig(SYS_FREQ, SYS_CFG_WAIT_STATES | SYS_CFG_PCACHE);

   // Variables hold the pulse start and stop for OC initialization
   unsigned int pulse_start = 0;
   unsigned int pulse_stop = 0;

   // Initialize GPIO for BTN1, BTN2, and LED 1-4
   PORTGCLR = 0xF000;   // Clear PortG to reset LEDs
   TRISGCLR = 0xF000;   // Clear PortG bit associated with LED 1-4
   ODCGCLR  = 0xF000;   // Normal output for LED 1-4 (not open drain)
   TRISGSET = 0x40;     // Set PortG bit associated with BTN1 and BTN2

   // Set JD Pin 10 to HIGH
   TRISDCLR = 0x1000;
   LATDSET = 0x1000;

   // Boolean values for debouncing
   BOOL playback = FALSE;   // Flag marks audio playback
   BOOL pause = FALSE;      // Flag marks audio playback pause
   BOOL button1 = FALSE;    // Button 1 flag for debouncing

   // Initialize interrupt and ADC module
   DelayInit();

   // Configure and enable the UART1 module to
   // Configre UART1 to transmit and receive data
   UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
   UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
   UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
   UARTSetDataRate(UART1, GetPeripheralClock(), 9600);			//set the baud rate
   UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));

   // Initialize I2C module
   I2CEnable(EEPROM, FALSE);
   I2CConfigure(EEPROM, 0);
   I2CSetFrequency(EEPROM, PBCLK, 400000);
   I2CEnable(EEPROM, TRUE);

   // Initialize SPI module
   int channel = 4;
   OpenSPI2(SPI_MODE16_ON, SPI_ENABLE);
   SpiChnOpen(channel, SPI_OPEN_MSTEN | SPI_CONFIG_MSSEN | SPI_CONFIG_MODE16 | SPI_CONFIG_ON , 2);
   //disable slave select SPIOPENSMPM SPI_OPEN_MSTEN SPI_OPEN_MSSEN

   // Set up timer 2
   OpenTimer2(T2_ON         |
             T2_IDLE_CON    |
             T2_SOURCE_INT  |
             T2_PS_1_2      |
             T2_GATE_OFF,
             250);  // freq = 10MHz/2/250 = 20 kHz

   // Initialize Output Compare 2 module
   OpenOC2( OC_ON | OC_TIMER_MODE16 | OC_TIMER2_SRC | OC_CONTINUE_PULSE | OC_LOW_HIGH, pulse_start, pulse_stop);
   SetDCOC2PWM(250);

   // Variable holds the duty cycle.  Adjusted depending on the current interrupt state
   unsigned int duty_cycle;

   // Set up CPU to respond to interrupts from Timer2
   mT2SetIntPriority(1);
   INTEnableSystemSingleVectoredInt();
   mT2IntEnable(1);

   // States of the state machine
   typedef enum {state_init, state_UART, state_write_EEPROM, state_read_EEPROM, state_MIDI,
                 state_pause, state_mic, state_amplifier} states;
   states state = state_init;

   // Character read from the UART
   char character;
   char buffer[256];
   unsigned int buf_iterator = 0;
   unsigned int buf_end = 0;
   char prevChar;

   // Character read from EEPROM
   char MIDI[2];
   char duration;

   // Integers to hold sound and MIDI data
   unsigned int note;
   unsigned int period;
   unsigned int note_end;
   unsigned int note_next;

   // Data to send and receive from the SPI
   unsigned int sendData;
   unsigned int receiveData;

   // Determines if a song is stored in the EEPROM
   mem_loc = 0x0000;
   if(ReadEEPROMByte() == '1')
   {
       mem_loc = 0x0001;
       mem_end = ReadEEPROMByte();
       mPORTGSetBits(0x8000);
   }
   else
       mem_end = 0x0002;

   // Display welcome message
   putsUART1("ECE 2524 Hokie Karaoke!\n\n");
   state = state_init;

   // Infinite loop
   while (1)
   {
       // State machine
       switch(state)
       {
           // State to to initialize the karaoke machine
           case state_init:
           {
               // Clear bits assosicated with Mic
               mPORTGClearBits(0x1000);

               // if button 1 is pressed, begin song playback
               if((PORTG & 0x40) && (button1 == FALSE))
               {
                   mem_loc = 0x0000;
                   // If there is a song stored in the EEPROM
                   if(ReadEEPROMByte() == '1')
                   {
                       mPORTGSetBits(0x1000);
                       button1 = TRUE;
                       playback = TRUE;
                       mem_loc = 0x0002;
                       state = state_read_EEPROM;
                       break;
                   }
                   else
                       state = state_init;
                   break;
               }

               // If there is data in the UART, input the data
               if(DataRdyUART1() == 1)
                   state = state_UART;
               else
                   state = state_init;

               break;
           }
           // State to input data from the UART
           case state_UART:
           {
               // Store the data from the UART in a variable
               prevChar = character;
               character = ReadUART1();
               WriteUART1(character);

               // Case statement for each character input in the UART
               switch(character)
               {
                   // Start of new song transmission
                   case 'S':
                   {
                       store_flag = TRUE;
                       buf_iterator = 0;
                       break;
                   }
                   // Start of new song transmission
                   case 's':
                   {
                       store_flag = TRUE;
                       buf_iterator = 0;
                       break;
                   }
                   // Delete a song in the EEPROM
                   case 'D':
                   {
                       if(store_flag == FALSE)
                       {
                           mem_loc = 0x0000;
                           while(!SendEEPROMByte('0'));
                           mPORTGClearBits(0x8000);
                           putsUART1("\nOK\n");
                           mem_loc = 0x0002;
                       }
                       // If the D character is a duration, not a delete command
                       else
                       {
                           buffer[buf_iterator] = character;
                           buf_iterator++;
                       }
                       break;
                   }
                   // Delete a song in the EEPROM
                   case 'd':
                   {
                       mem_loc = 0x0000;
                       while(!SendEEPROMByte('0'));
                       mPORTGClearBits(0x8000);
                       putsUART1("\nOK\n");
                       mem_loc = 0x0002;
                       break;
                   }
                   // Tells the user if a song is stored in the EEPROM
                   case '?':
                   {
                       mem_loc = 0x0000;
                       if(ReadEEPROMByte() == '1')
                           putsUART1("\n1 song is stored in the Hokie Karaoke Machine\n");
                       else
                           putsUART1("\n0 songs are stored in the Hokie Karaoke Machine\n");
                       mem_loc = 0x0002;
                       break;
                   }
                   case ' ':
                   {
                       break;
                   }
                   // Ends the song if two conesctutive nines are entered
                   case '9':
                   {
                       if(prevChar == '9')
                       {
                           store_flag = FALSE;
                           buf_end = buf_iterator - 2;
                           end_flag = TRUE;
                           putsUART1("\nOK\n");
                       }
                       // If the nine is non-consecutive
                       else
                       {
                           buffer[buf_iterator] = character;
                           buf_iterator++;
                       }
                       break;
                   }
                   default:
                   {
                       buffer[buf_iterator] = character;
                       buf_iterator++;
                       break;
                   }
               }

               // If there is data in the UART, continue reading in data
               if(DataRdyUART1() == 1)
                   state = state_UART;
               if(end_flag == TRUE)
               {
                   end_flag = FALSE;
                   // Go to state to write UART input buffer to the EEPROM
                   state = state_write_EEPROM;
               }
               else
                   state = state_init;

               break;
           }
           // State to write UART input buffer data to EEPROM
           case state_write_EEPROM:
           {
               mem_loc = 0x0000;
               while(!(SendEEPROMByte('1')));
               mPORTGSetBits(0x8000);
               mem_loc = 0x0002;

               putsUART1("    ");

               buf_iterator = 0;

               // Write data from the input buffer to the EEPROM
               while(buf_iterator <= buf_end)
               {
                   character = buffer[buf_iterator];
                   if(character != ' ')
                   {
                       while(!(SendEEPROMByte(character)));
                   }
                   putsUART1("    ");
                   buf_iterator++;
               }

               // Return to the beginnging memory location and go back to the inital state
               mem_end = mem_loc - 3;
               mem_loc = 0x0001;
               while(!(SendEEPROMByte(mem_end)));
               mem_loc = 0x0002;

               state = state_init;

               break;
           }
           // State to read data from the EEPROM
           case state_read_EEPROM:
           {
               // Read the note and duration from the EEPROM
               if(mem_loc <= mem_end)
               {
                   MIDI[0] = ReadEEPROMByte();
                   MIDI[1] = ReadEEPROMByte();
                   duration = ReadEEPROMByte();
               }
               else
               {
                   button1 = FALSE;
                   playback = FALSE;
                   state = state_init;
                   break;
               }
               // Go to the MIDI convert state
               state = state_MIDI;
               break;
           }
           // State to convert MIDI data to an audio signal
           case state_MIDI:
           {
               note_time = 0;

               // Determines the duration of the note
               switch(duration)
               {
                   case 'F':
                   {
                       note_end = 36000;
                       note_next = 40000;
                       break;
                   }
                   case 'E':
                   {
                       note_end = 27000;
                       note_next = 30000;
                       break;
                   }
                   case 'D':
                   {
                       note_end = 18000;
                       note_next = 20000;
                       break;
                   }
                   case 'C':
                   {
                       note_end = 13500;
                       note_next = 15000;
                       break;
                   }
                   case 'B':
                   {
                       note_end = 9000;
                       note_next = 10000;
                       break;
                   }
                   case 'A':
                   {
                       note_end = 4500;
                       note_next = 5000;
                       break;
                   }
               }

               // Convert the ASCII note into an integer
               note = ((10*(MIDI[0] - 48)) + (MIDI[1] - 48));

               // Start at the base frequency and move up to the actual note frequency
               frequency = 440;
               int i = 69;
               while(i <= note)
               {
                   frequency = frequency * 1.0594631;
                   i++;
               }

               state = state_pause;

               break;
           }
           // State to determine if the playback is paused
           case state_pause:
           {
               // When button one is pressed, pause of unpause
               if((PORTG & 0x40) && (button1 == FALSE))
               {
                   button1 == TRUE;
                   
                   if(pause == FALSE)
                       pause = TRUE;
                   else
                       pause = FALSE;
               }

               // If paused, turn on LED 2
               if(pause == TRUE)
               {
                   mPORTGSetBits(0x2000);
               }
               // If unpaused, turn off LED 2
               else
               {
                   mPORTGClearBits(0x2000);
               }

               state = state_mic;

               // If there is data in the UART during playback
               if(DataRdyUART1() == 1)
               {
                   // Store the data from the UART in a variable
                   character = ReadUART1();
                   WriteUART1(character);

                   // Case statement for each character input in the UART
                   switch(character)
                   {
                       // Delete the song stored in the EEPROM and stop playback
                       case 'd':
                       {
                           playback = FALSE;
                           pause = FALSE;
                           mem_loc = 0x0000;
                           while(!SendEEPROMByte('0'));
                           mPORTGClearBits(0x8000);
                           putsUART1("\nOK\n");
                           mem_loc = 0x0002;
                           state = state_init;
                           break;
                       }
                       // Delete the song stored in the EEPROM and stop playback
                       case 'D':
                       {
                           playback = FALSE;
                           pause = FALSE;
                           mem_loc = 0x0000;
                           while(!SendEEPROMByte('0'));
                           mPORTGClearBits(0x8000);
                           putsUART1("\nOK\n");
                           mem_loc = 0x0002;
                           state = state_init;
                           break;
                       }
                       // Determine if a song is stored in the UART, continue playback
                       case '?':
                       {
                           mem_temp = mem_loc;
                           mem_loc = 0x0000;
                           if(ReadEEPROMByte() == '1')
                               putsUART1("\n1 song is stored in the Hokie Karaoke Machine\n");
                           else
                               putsUART1("\n0 songs are stored in the Hokie Karaoke Machine\n");
                           mem_loc = mem_temp;
                           break;
                       }
                       // Store a new song in the EEPROM, stop playback
                       case 'S':
                       {
                           playback = FALSE;
                           pause = FALSE;
                           store_flag = TRUE;
                           buf_iterator = 0;
                           state = state_UART;
                           break;
                       }
                       // Store a new song in the EEPROM, stop playback
                       case 's':
                       {
                           playback = FALSE;
                           pause = FALSE;
                           store_flag = TRUE;
                           buf_iterator = 0;
                           state = state_UART;
                           break;
                       }
                   }
               }

               // Button debouncing
               if(button1 == TRUE && (!(PORTG & 0x40)))
               {
                   button1 = FALSE;
               }

               break;
           }
           // State to get data from the microphone
           case state_mic:
           {
               sendData = 0xffff;
	       SpiChnPutC(channel, sendData);          // send data
               receiveData = SpiChnGetC(channel);      // retreive the received data

               // convert the data into a usable
               receiveData = receiveData & 0x0fff;
               receiveData = ((receiveData*500)/4096);

               state = state_amplifier;

               break;
           }
           // State to play the audio signal to the amplifier
           case state_amplifier:
           {
               // Get the period of the wave
               period = 20000/frequency;

               // Divides the slope of the triangle wave
               if(sec50u > (period/2))
                   duty_cycle = (period - sec50u);
               else
                   duty_cycle = sec50u;

               // Multiply the duty cycle by 5 since the period register is 250
               duty_cycle = (4*duty_cycle) + receiveData;

               // Set the next duty cycle by writing to OCxRS
               // Plays both the MIDI and microphone signal
               if(pause == FALSE)
               {
                   if(note_time < note_end)
                       SetDCOC2PWM(duty_cycle);
                   else
                       SetDCOC2PWM(receiveData);

                   if(note_time < note_next)
                       state = state_pause;
                   else
                   {
                       state = state_read_EEPROM;
                   }
               }
               // If the playback is paused, play only microphone data
               else
               {
                   SetDCOC2PWM(receiveData);
                   state = state_pause;
               }

               break;
           }
       }
   }

   // Close the Output Compare 2 module
   CloseOC2();

   // End of main function
   return 0;
}

// Interrupt handler - respond to timer-generated interrupt
#pragma interrupt InterruptHandler ipl1 vector 0
void InterruptHandler( void )
{
   if( INTGetFlag(INT_T2) )    // Verify source of interrupt
   {
      sec50u++;            // Update global variable

      if(sec50u > (20000/frequency))      // Only counts from 0 - 45
          sec50u = 0;      // Because 20000Hz divided by 440 Hz is 45

      note_time++;

      if(note_time > 40000)
          note_time = 0;

      INTClearFlag(INT_T2);    // Acknowledge interrupt
   }
}

// This function sends a byte of data to the EEPROM
// for non-volitale storage
BOOL SendEEPROMByte(BYTE data)
{
	// Send a high and low address for data storage
    BYTE high_address = ((mem_loc & 0xFF00) >> 8);
    BYTE low_address = (mem_loc & 0x00FF);

    // Wait until the EEPROM is idle
    while(!(I2CBusIsIdle(MASTER_MODULE)));

    // Send a start bit
    I2CStart(MASTER_MODULE);
    while(!(I2CGetStatus(MASTER_MODULE) & I2C_START));

    // Tell the EEPROM to write
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, EEPROM_WRITE_ADDRESS);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Send the high byte of the address
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, high_address);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Send the low byte of the address
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, low_address);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Send the data to the address
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, data);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Increment the address counter by 1 and send a stop bit
    mem_loc = mem_loc + 0x0001;
    I2CStop(MASTER_MODULE);
    while(!(I2CGetStatus(MASTER_MODULE) & I2C_STOP));

    return TRUE;
}

// This function retrieves the a byte of data from the
// EEPROM (non-volitale storage)
BYTE ReadEEPROMByte()
{
    // Address to which to store the data
    BYTE high_address = (mem_loc & 0xFF00) >> 8;
    BYTE low_address = (mem_loc & 0x00FF);
    
    BYTE data;

    // Wait until the EEPROM is idle
    while(!(I2CBusIsIdle(MASTER_MODULE)));

    // Send a start bit
    I2CStart(MASTER_MODULE);
    while(!(I2CGetStatus(MASTER_MODULE) & I2C_START));

    // Tell the EEPROM to write
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, EEPROM_WRITE_ADDRESS);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Send the high byte of the address
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, high_address);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Send the low byte of the address
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, low_address);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Repeat the start send bit
    I2CRepeatStart(MASTER_MODULE);
    while(!(I2CGetStatus(MASTER_MODULE) & I2C_START));

    // Tell the EEPROM to read
    while(!(I2CTransmitterIsReady(MASTER_MODULE)));
    I2CSendByte(MASTER_MODULE, EEPROM_READ_ADDRESS);
    while(!(I2CTransmissionHasCompleted(MASTER_MODULE)));
    while(!(I2CByteWasAcknowledged(MASTER_MODULE)));

    // Read the data from the EEPROM
    I2CReceiverEnable(MASTER_MODULE, TRUE);
    while(!(I2CReceivedDataIsAvailable(MASTER_MODULE)));
    data = I2CGetByte(MASTER_MODULE);
    I2CAcknowledgeByte(MASTER_MODULE, FALSE);
    while(!(I2CAcknowledgeHasCompleted(MASTER_MODULE)));

    // Increment the address counter by 1 and send a stop bit
    mem_loc = mem_loc + 0x0001;
    I2CStop(MASTER_MODULE);
    while(!(I2CGetStatus(MASTER_MODULE) & I2C_STOP));

    return data;
}
