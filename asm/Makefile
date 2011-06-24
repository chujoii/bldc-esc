#ASM=/usr/bin/avra
ASM=/usr/bin/avr-as
ISP=/usr/bin/avrdude 
TARGET=bcr-01
FUSE_BITS=-U lfuse:w:0x2f:m -U hfuse:w:0xC9:m
LOCK_BIS=-U lock:w:0x00:m

program : $(TARGET).hex
	$(ISP) -p m8 -P /dev/parport0 -c blaster -U flash:w:$(TARGET).hex:i $(FUSE_BITS) $(LOCK_BIS)

$(TARGET).hex : $(TARGET).asm
	$(ASM) -l list -m map $(TARGET).asm

.PHONY : clean
clean :
	rm -f *.hex *.obj *.cof *.lst

