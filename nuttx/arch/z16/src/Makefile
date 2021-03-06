############################################################################
# arch/z16/src/Makefile
#
#   Copyright (C) 2008, 2011 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs
-include chip/Make.defs

COMPILER	= ${shell basename $(CC)}
ARCHSRCDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(COMPILER),zneocc.exe)
WARCHSRCDIR	:= ${shell cygpath -w $(ARCHSRCDIR)}
USRINCLUDES	= -usrinc:'.;$(WTOPDIR)\sched;$(WARCHSRCDIR);$(WARCHSRCDIR)\common'
else
WARCHSRCDIR	= $(ARCHSRCDIR)
USRINCLUDES	= -I$(TOPDIR)/sched -I$(ARCHSRCDIR) -I$(ARCHSRCDIR)/common
endif
INCLUDES	= $(ARCHSTDINCLUDES) $(USRINCLUDES)
CFLAGS		= $(ARCHWARNINGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) $(INCLUDES) $(ARCHDEFINES) $(EXTRADEFINES)
CPPFLAGS	+= -I$(ARCHSRCDIR)
ifeq ($(COMPILER),zneocc.exe)
LDFLAGS 	= @"${shell cygpath -w $(ARCHSRCDIR)/nuttx.linkcmd}"
endif

HEAD_ASRC	= $(HEAD_SSRC:.S=$(ASMEXT))
HEAD_AOBJ	= $(HEAD_SSRC:.S=$(OBJEXT))

SSRCS		= $(CHIP_SSRCS) $(CMN_SSRCS)
ASRCS		= $(SSRCS:.S=$(ASMEXT))
AOBJS		= $(SSRCS:.S=$(OBJEXT))

CSRCS		= $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

DEPSRCS 	= $(SSRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BOARDDIR	= $(TOPDIR)/arch/$(CONFIG_ARCH)/src/board

VPATH		= chip:common

all: $(HEAD_OBJ) libarch$(LIBEXT)

ifeq ($(COMPILER),zneocc.exe)
$(ASRCS) $(HEAD_ASRC): %$(ASMEXT): %.S
	@$(CPP) $(CPPFLAGS) $< -o $@.tmp
	@cat $@.tmp | sed -e "s/^#/;/g" > $@
	@rm $@.tmp

$(AOBJS) $(HEAD_AOBJ): %$(OBJEXT): %$(ASMEXT)
	$(call ASSEMBLE, $<, $@)
else
$(OBJS) $(HEAD_AOBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)
endif

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

board/libboard$(LIBEXT):
	@$(MAKE) -C board TOPDIR="$(TOPDIR)" libboard$(LIBEXT) EXTRADEFINES=$(EXTRADEFINES)

ifeq ($(COMPILER),zneocc.exe)
nuttx.linkcmd: $(LINKCMDTEMPLATE)
	@cp -f $(LINKCMDTEMPLATE) nuttx.linkcmd
	@echo "\"${shell cygpath -w $(TOPDIR)/nuttx}\"= \\" >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ARCHSRCDIR)/$(HEAD_AOBJ)}\", \\" >>nuttx.linkcmd
	@( for lib in $(LINKLIBS); do \
		echo "  \"`cygpath -w $(TOPDIR)/$${lib}`\", \\" >>nuttx.linkcmd; \
	done ; )
	@echo "  \"${shell cygpath -w $(ARCHSRCDIR)/board/libboard$(LIBEXT)}\", \\"  >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ZDSSTDLIBDIR)/chelpld$(LIBEXT)}\", \\"  >>nuttx.linkcmd
	@echo "  \"${shell cygpath -w $(ZDSSTDLIBDIR)/fpld$(LIBEXT)}\""  >>nuttx.linkcmd
else
nuttx.linkcmd:
endif

nuttx$(EXEEXT): $(HEAD_AOBJ) board/libboard$(LIBEXT) nuttx.linkcmd
	@echo "LD:  nuttx.hex"
	@$(LD) $(LDFLAGS)

.depend: Makefile chip/Make.defs $(DEPSRCS)
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" depend ; \
	fi
	@$(MKDEP) --dep-path chip --dep-path common $(CC) -- $(CFLAGS) -- $(DEPSRCS) >Make.dep
	@touch $@

depend: .depend

clean:
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" clean ; \
	fi
	@rm -f libarch$(LIBEXT) *~ .*.swp
ifeq ($(COMPILER),zneocc.exe)
	@rm -f nuttx.linkcmd *.asm *.tmp *.map
endif
	$(call CLEAN)

distclean: clean
	@if [ -e board/Makefile ]; then \
		$(MAKE) -C board TOPDIR="$(TOPDIR)" distclean ; \
	fi
	@rm -f Make.dep .depend

-include Make.dep
