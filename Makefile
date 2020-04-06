# ar       - compacta os objetos criados em um arquivo .a
# ranlib   - indexa a biblioteca para agilizar o processo de linker com bibliotecas estáticas
# size     - apresenta a quantidade de memória ocupada pela biblioteca
# -g       - adds debugging information to the executable file
# -Ox      - defines compiler optimization level
# -Wall    - disable almost all warning flags
# -pipe	   - use pipes rather than temporary files between compilation stages
# -Winline - informs if the functions with inline directives has not been inlined and why not.


TARGET=libserial.a

#DEBUG	= -g -O0
DEBUG	= -O3
CC	= gcc
INCLUDE	= -I.
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe

LIBS    =

# Should not alter anything below this line
###############################################################################

SRC	=	serial.c

OBJ	=	serial.o

all:		$(TARGET)

$(TARGET):	$(OBJ)
	@echo [AR] $(OBJ)
	@ar rcs $(TARGET) $(OBJ)
	@ranlib $(TARGET)
	@size   $(TARGET)

.c.o:
	@echo [CC] $<
	@$(CC) -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)


install:	$(TARGET)
	@echo [install]
	install -m 0755 -d /usr/local/lib
	install -m 0755 -d /usr/local/include
	install -m 0644 serial.h	/usr/local/include
	install -m 0644 libserial.a	/usr/local/lib

uninstall:
	@echo [uninstall]
	rm -f /usr/local/include/serial.h
	rm -f /usr/local/lib/libserial.a

