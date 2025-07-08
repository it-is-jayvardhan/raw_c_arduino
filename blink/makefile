# ==== Config ====
MCU     = atmega328p
F_CPU   = 16000000UL
PORT    = /dev/ttyACM0
BAUD    = 115200

CC      = avr-gcc
OBJCOPY = avr-objcopy
AVRDUDE = avrdude

CFLAGS  = -mmcu=$(MCU) -DF_CPU=$(F_CPU) -Os -Wall

# Allow user to set TARGET from CLI
TARGET  ?= main
SRC     = $(TARGET).c

# ==== Build Rules ====
all: $(TARGET).hex

$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# ==== Flash to Arduino ====
flash: $(TARGET).hex
	$(AVRDUDE) -c arduino -p m328p -P $(PORT) -b $(BAUD) -U flash:w:$<

# ==== Clean Up ====
clean:
	rm -f *.elf *.hex
