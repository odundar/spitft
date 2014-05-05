
bin_PROGRAMS = DrawCircle
DrawCircle_SOURCES = DrawCircle.cpp Print.cpp mux.c UtilTime.cpp font.c fast_gpio_sc.c interrupt.c RingBuffer.cpp Stream.cpp sysfs.c trace.c TTYUART.cpp wiring_digital.c WString.cpp variant.cpp TFTv2.cpp SPI.cpp

EXTRA_DIST = SPI.h TFTv2.h AnalogIO.h Arduino.h pgmspace.h binary.h  BitsAndBytes.h fast_gpio_sc.h HardwareSerial.h interrupt.h Mux.h OSAbstract.h pins_arduino.h Print.h Printable.h RingBuffer.h Stream.h trace.h TTYUART.h UtilTime.h variant.h WCharacter.h wiring_digital.h WMath.h WString.h Makefile Makefile.am Makefile.in

AM_CXXFLAGS = @DrawCircle_CFLAGS@ -I../include/
AM_LDFLAGS = @DrawCircle_LIBS@ -lpthread 

CLEANFILES = *~
