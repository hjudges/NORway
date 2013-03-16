################################################################################
# Notice: This Makefile has been edited to support platforms other than Windows.
# It may be overwritten by AVR Studio.
################################################################################

ifdef SYSTEMROOT
    SHELL := cmd.exe
    EXT :=.exe

    ifndef $(AVR_APP_PATH)

      # Windows
      # #########################################################################

      # For Windows users, for example who have installed AVR Studio

      # Put your path to "avr-gcc.exe" here (should end in "AVR ToolChain\bin\")
      # Include the trailing slash.
      # AVR_APP_PATH :=C:/Program Files (x86)/Atmel/AVR Studio 5.0/AVR ToolChain/bin/

    endif
else
    ifndef $(AVR_APP_PATH)

      # Linux / Mac
      # #########################################################################

      # For Linux or Mac users, for example who have installed CrossPack AVR

      # Put your path to "avr-gcc" here (should end in "CrossPack-AVR/bin/")
      # Include the trailing slash.
      # AVR_APP_PATH :=/usr/local/CrossPack-AVR/bin/

      # Otherwise, if avr-gcc is somewhere on your $PATH
      # AVR_APP_PATH := $$(dirname $$(which avr-gcc$(EXT)))/

    endif
endif

RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
LIB_AS_ARGS :=
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
HEX_FLASH_FILE_PATH :=
HEX_FLASH_FILE_PATH_AS_ARGS :=
HEX_EEPROM_FILE_PATH :=
HEX_EEPROM_FILE_PATH_AS_ARGS :=
LSS_FILE_PATH :=
LSS_FILE_PATH_AS_ARGS :=
MAP_FILE_PATH :=
MAP_FILE_PATH_AS_ARGS :=
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../NANDway.c \
../usb_serial.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
NANDway.o \
usb_serial.o


OBJS_AS_ARGS +=  \
NANDway.o \
usb_serial.o


C_DEPS +=  \
NANDway.d \
usb_serial.d


C_DEPS_AS_ARGS +=  \
NANDway.d \
usb_serial.d


OUTPUT_FILE_PATH +=NANDway.elf

OUTPUT_FILE_PATH_AS_ARGS +=NANDway.elf

HEX_FLASH_FILE_PATH +=NANDway.hex

HEX_FLASH_FILE_PATH_AS_ARGS +=NANDway.hex

HEX_EEPROM_FILE_PATH +=NANDway.eep

HEX_EEPROM_FILE_PATH_AS_ARGS +=NANDway.eep

LSS_FILE_PATH +=NANDway.lss

LSS_FILE_PATH_AS_ARGS +=NANDway.lss

MAP_FILE_PATH =NANDway.map

MAP_FILE_PATH_AS_ARGS =NANDway.map

LIB_AS_ARGS +=libNANDway.a

ADDITIONAL_DEPENDENCIES:= $(HEX_FLASH_FILE_PATH) size

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR/GNU C Compiler





./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler
	$(QUOTE)$(AVR_APP_PATH)avr-gcc$(EXT)$(QUOTE) -funsigned-char -funsigned-bitfields -DF_CPU=8000000UL  -O3 -fpack-struct -fshort-enums -Wall -c -gdwarf-2 -std=gnu99   -mmcu=at90usb1286   -MD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo Finished building: $<



# AVR/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

# AVR/GNU C/C++ Linker
$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU C/C++ Linker
	$(QUOTE)$(AVR_APP_PATH)avr-gcc$(EXT)$(QUOTE)  -mmcu=at90usb1286   -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS)
	@echo Finished building target: $@



$(HEX_FLASH_FILE_PATH): $(OUTPUT_FILE_PATH)
	$(QUOTE)$(AVR_APP_PATH)avr-objcopy$(EXT)$(QUOTE) -O ihex -R .eeprom -R .fuse -R .lock -R .signature  $(QUOTE)$<$(QUOTE) $(QUOTE)$@$(QUOTE)

$(HEX_EEPROM_FILE_PATH): $(OUTPUT_FILE_PATH)
	-$(QUOTE)$(AVR_APP_PATH)avr-objcopy$(EXT)$(QUOTE) -j .eeprom --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0 --no-change-warnings -O ihex $(QUOTE)$<$(QUOTE) $(QUOTE)$@$(QUOTE) || exit 0

$(LSS_FILE_PATH): $(OUTPUT_FILE_PATH)
	$(QUOTE)$(AVR_APP_PATH)avr-objdump$(EXT)$(QUOTE) -h -S $(QUOTE)$<$(QUOTE) > $(QUOTE)$@$(QUOTE)

size: $(OUTPUT_FILE_PATH)
	@$(QUOTE)$(AVR_APP_PATH)avr-size$(EXT)$(QUOTE) -C --mcu=at90usb1286 $(OUTPUT_FILE_PATH_AS_ARGS)

# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) $(LIB_AS_ARGS) $(HEX_FLASH_FILE_PATH_AS_ARGS) $(HEX_EEPROM_FILE_PATH_AS_ARGS) $(LSS_FILE_PATH_AS_ARGS) $(MAP_FILE_PATH_AS_ARGS)
