/*
 * serial.c:
 *	Handle a serial port
 */

#include <stdio.h>		/* Standard input/output definitions */
#include <stdlib.h>		/* Standard general utilities library */
#include <stdint.h>		/* Standard integer types */
#include <stdarg.h>		/* Variable argument handling */
#include <string.h>		/* Standard string manipulation functions */
#include <termios.h>	/* POSIX terminal control definitons */
#include <unistd.h>		/* UNIX standard function definitios */
#include <fcntl.h>		/* File control definitions */
#include <sys/ioctl.h>	/* System I/O definitions and structures */
#include <sys/types.h>	/* Standard system types. Ex.: types for mutex */
#include <sys/stat.h>	/* Used to facilitate getting information about file attributes */

#include "serial.h"

/*
 * serialOpen:
 *	Open and initialise the serial port, setting all the right
 *	port parameters - or as many as are required.
 *********************************************************************************
 */

int serialOpen (char *device, int baud)
{
	struct termios options ;
	speed_t myBaud ;
	int     status, fd ;

	switch (baud)
	{
	case     50:	myBaud =     B50 ; break ;
	case     75:	myBaud =     B75 ; break ;
	case    110:	myBaud =    B110 ; break ;
	case    134:	myBaud =    B134 ; break ;
	case    150:	myBaud =    B150 ; break ;
	case    200:	myBaud =    B200 ; break ;
	case    300:	myBaud =    B300 ; break ;
	case    600:	myBaud =    B600 ; break ;
	case   1200:	myBaud =   B1200 ; break ;
	case   1800:	myBaud =   B1800 ; break ;
	case   2400:	myBaud =   B2400 ; break ;
	case   9600:	myBaud =   B9600 ; break ;
	case  19200:	myBaud =  B19200 ; break ;
	case  38400:	myBaud =  B38400 ; break ;
	case  57600:	myBaud =  B57600 ; break ;
	case 115200:	myBaud = B115200 ; break ;
	case 230400:	myBaud = B230400 ; break ;

	default:
	  return -2 ;
	}

	// Open serial port "device"
	// O_RDWR 	- allows to read and write to the serial port
	// O_NOCTTY - tells UNIX that this program doesn't want to be the controlling entity for that port. If you do not specify this, the device file
	//			  will be owned by you, and any input (such as keyboard abort signals) will afect your process.
	// O_NDELAY - this flag tells that we don't care whether the other end of the port is up and running (i.e. if a data carrier detect (DCD) signal is present)
	//			  If you do not specify this flag, your process will be put to sleep until the DCD signal line is set to the space voltage.
	if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
		return -1 ;

	// Get the current options:
	tcgetattr (fd, &options) ;

    // Set the terminal to the raw mode, i.e., input is available character by character, echoing is disabled and special processing
    // of terminal input and output character is disabled.
    // The terminal attributes are set as follows:
    // termios_p->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    // termios_p->c_oflag &= ~OPOST;
    // termios_p->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    // termios_p->c_lflag &= ~(CSIZE | PARENB);
    // termios_p->c_lflag |= CS8;
    cfmakeraw   (&options) ;
    
    // Set input baud rate speed
    cfsetispeed (&options, myBaud) ;
    
    // Set output baud rate speed
    cfsetospeed (&options, myBaud) ;

    // CLOCAL - Ignore modem control lines
    // CREAD  - Enable receiver
    options.c_cflag |= (CLOCAL | CREAD) ;
    
    // PARENB - Enable parity generation on output and parity checking for input (disabled)
    options.c_cflag &= ~PARENB ;
    
    // CSTOPB - Set two stop bits, rather than one (disabled)
    options.c_cflag &= ~CSTOPB ;
    
    // CSIZE - Character size mask (CS5, CS6, CS7, or CS8) - Clear character size info
    options.c_cflag &= ~CSIZE ;
    
    // Set 8 bit character
    options.c_cflag |= CS8 ;
    
    // Disable the canonical mode, the echo input characters and some signal related to INTR, QUIT, SUSP and DSUSP characters
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    
    // Disable implementation-defined output processing
    options.c_oflag &= ~OPOST ;

    // There is no limit of characters in order to return a read command
    // returns either when at least one byte of data is available, or when the timer expires.
    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

	// Set modify options
	// TCSANOW - set now or ...
	// TCSAFLUSH - set after all output has been transmitted. Also, all input that has been received but not read will be discarted
	// 			   before the change is made.
	tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

	// Get the status of the modem bits.
	ioctl (fd, TIOCMGET, &status);

	// Set data terminal ready (DTR) in the status flag
	status |= TIOCM_DTR ;
	
	// Set request to send (RTS) in the status flag
	status |= TIOCM_RTS ;

	// Set this new status flag
	ioctl (fd, TIOCMSET, &status);

	usleep (10000) ;	// 10mS

	// Return the file descriptor
	return fd ;
}


/*
 * serialFlush:
 *	Discard data written to the serial port but not transmitted, or data received but not read; Flush both buffers (tx & rx)
 *********************************************************************************
 */

void serialFlush (int fd)
{
  tcflush (fd, TCIOFLUSH) ;
}


/*
 * serialClose:
 *	Release the serial port
 *********************************************************************************
 */

void serialClose (int fd)
{
  close (fd) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

int serialPutchar (int fd, unsigned char c)
{
  return write (fd, &c, 1) ;
}


/*
 * serialPutchar:
 *	Send a single character to the serial port
 *********************************************************************************
 */

int serialPutBuffer (int fd, char *s, int len)
{
   return write (fd, s, len) ;
}

/*
 * serialPuts:
 *	Send a string to the serial port
 *********************************************************************************
 */

int serialPuts (int fd, char *s)
{
  return write (fd, s, strlen (s)) ;
}

/*
 * serialPrintf:
 *	Printf over Serial
 *********************************************************************************
 */

void serialPrintf (int fd, char *message, ...)
{
  va_list argp ;
  char buffer [1024] ;

  va_start (argp, message) ;
    vsnprintf (buffer, 1023, message, argp) ;
  va_end (argp) ;

  serialPuts (fd, buffer) ;
}


/*
 * serialDataAvail:
 *	Return the number of bytes of data avalable to be read in the serial port
 *********************************************************************************
 */

int serialDataAvail (int fd)
{
  int result ;

  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;

  return result ;
}


/*
 * serialGetchar:
 *	Get a single character from the serial device.
 *	Note: Zero is a valid character and this function will time-out after
 *	10 seconds.
 *********************************************************************************
 */

int serialGetchar (int fd, char *data)
{
  uint8_t x ;
  
  fcntl(fd, F_SETFL, 0);			/* causes read to block until new characters are present */
  //fcntl(fd, F_SETFL, FNDELAY);	/* return immediately from the read function */

  if (read (fd, &x, 1) != 1)
    return -1 ;
    
  *data = ((char)x) & 0xFF;

  return 0;
}
