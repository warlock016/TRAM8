################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := /bin/bash
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
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../general_twi.c \
../main.c \
../MAX5825.c \
../midi.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
general_twi.o \
main.o \
MAX5825.o \
midi.o

OBJS_AS_ARGS +=  \
general_twi.o \
main.o \
MAX5825.o\
midi.o

C_DEPS +=  \
general_twi.d \
main.d \
MAX5825.d\
midi.d

C_DEPS_AS_ARGS +=  \
general_twi.d \
main.d \
MAX5825.d\
midi.d

OUTPUT_FILE_PATH +=Tram8_Random.elf

OUTPUT_FILE_PATH_AS_ARGS +=Tram8_Random.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 

# AVR32/GNU C Compiler
%.o: ../%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 5.4.0
	$(QUOTE)/usr/local/Cellar/avr-gcc@9/9.4.0/bin/avr-gcc$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -DDEBUG  -I"/usr/local/Cellar/avr-gcc@9/9.4.0/avr/include" -I"/usr/local/Cellar/avr-gcc@9/9.4.0/avr/lib"  -O1 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -g2 -Wall -mmcu=atmega8a -B "/usr/local/Cellar/avr-gcc@9/9.4.0/avr/lib/usr/local/Cellar/avr-binutils/2.40_2/avr/bin" -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<"
	@echo Finished building: $<



# AVR32/GNU Preprocessing Assembler
# AVR32/GNU Assembler
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 5.4.0
	$(QUOTE)/usr/local/Cellar/avr-gcc@9/9.4.0/bin/avr-gcc$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="Tram8_Random.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group -Wl,--gc-sections -mmcu=atmega8a -B "/usr/local/Cellar/avr-gcc@9/9.4.0/avr/lib/usr/local/Cellar/avr-binutils/2.40_2/bin"  
	@echo Finished building target: $@
	"/usr/local/Cellar/avr-binutils/2.40_2/bin/avr-objcopy" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "Tram8_Random.elf" "Tram8_Random.hex"
	"/usr/local/Cellar/avr-binutils/2.40_2/bin/avr-objcopy" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "Tram8_Random.elf" "Tram8_Random.eep" || exit 0
	"/usr/local/Cellar/avr-binutils/2.40_2/bin/avr-objdump" -h -S "Tram8_Random.elf" > "Tram8_Random.lss"
	"/usr/local/Cellar/avr-binutils/2.40_2/bin/avr-objcopy" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "Tram8_Random.elf" "Tram8_Random.srec"
	"/usr/local/Cellar/avr-binutils/2.40_2/bin/avr-size" "Tram8_Random.elf"
	

# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "Tram8_Random.elf" "Tram8_Random.a" "Tram8_Random.hex" "Tram8_Random.lss" "Tram8_Random.eep" "Tram8_Random.map" "Tram8_Random.srec" "Tram8_Random.usersignatures"
	