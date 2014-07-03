#   Copyright (c) 2014 Thomas Kuehnel
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the authors nor the names of its contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
#   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#   POSSIBILITY OF SUCH DAMAGE.

# === main parameters of the project =========================================
URACOLIDIR = ../uracoli-src-0.4.2/
PROJECT = linux
CURRENT_MAKEFILE = Makefile
BOARD = raspbee
PART = UNDEFINED
OBJDIR = ./obj

BINDIR = bin
LIBDIR = $(URACOLIDIR)/lib

# guessing the OS for a working (g)mkdir
ifdef SystemRoot
    MKDIR=gmkdir -p
else
    MKDIR=mkdir -p
endif

raspbee:
	$(MAKE) -f $(CURRENT_MAKEFILE) BOARD=raspbee MCU=atmega256rfr2 F_CPU=8000000UL BOOTOFFSET=0x3e000 $(TARGETS)

clean:
	rm -rf $(OBJDIR)/*.o $(OBJDIR)/*.lst $(BINDIR)/*.elf $(BINDIR)/*.hex

# === internal rules ===================================================

# temporary output directory
$(OBJDIR):
	$(MKDIR) $@

$(BINDIR):
	$(MKDIR) $@

TARGETS=$(OBJDIR) $(BINDIR) __linux__
SOURCES = $(PROJECT).c
INCDIRS = . $(URACOLIDIR)/inc
LIBDIRS = $(URACOLIDIR)/lib
# DBGFMT=stabs for Linux
# DBGFMT=dwarf-2 for Windows
DBGFMT=
# automatically derived parameters
OBJECTS = $(SOURCES:%.c=$(OBJDIR)/%_$(BOARD).o)
TARGET = $(BINDIR)/$(PROJECT)_$(BOARD)

# === tool parameters ======================================================

CC = avr-gcc
CCFLAGS = -Wall -Wundef -Os -g$(DBGFMT) -mmcu=$(MCU)
CCFLAGS += -Wa,-adhlns=$(<:%.c=$(OBJDIR)/%_$(BOARD).lst)
CCFLAGS += -D$(BOARD) -DF_CPU=$(F_CPU)
ifneq ($(baudrate),)
    CCFLAGS += -DHIF_DEFAULT_BAUDRATE=$(baudrate)
endif
CCFLAGS += -I$(URACOLIDIR)/inc -I.
LDFLAGS = $(patsubst %,-L%,$(LIBDIRS)) -luracoli_$(BOARD)

# === custom settings ======================================================

OC=avr-objcopy
OCFLAGS=-O binary

# === build rules ============================================================
__linux__: $(TARGET).bin

$(TARGET).bin: $(TARGET).elf
	$(OC) $(OCFLAGS) $< $@

$(TARGET).elf: $(OBJECTS)
	$(CC) -o $@ $(CCFLAGS) $^ $(LDFLAGS)

$(OBJDIR)/%_$(BOARD).o: %.c
	$(CC) $(CCFLAGS) -c -o $@ $<
