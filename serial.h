/*
 * serial.h:
 *	Handle a serial port
 ***********************************************************************
 */

#ifdef __cplusplus
extern "C" {
#endif

extern int   serialOpen      (char *device, int baud) ;
extern void  serialClose     (int fd) ;
extern void  serialFlush     (int fd) ;
extern int   serialPutchar   (int fd, unsigned char c) ;
extern int   serialPuts      (int fd, char *s) ;
extern int   serialPutBuffer (int fd, char *s, int len);
extern void  serialPrintf    (int fd, char *message, ...) ;
extern int   serialDataAvail (int fd) ;
extern int	 serialGetchar 	 (int fd, char *data) ;

#ifdef __cplusplus
}
#endif
