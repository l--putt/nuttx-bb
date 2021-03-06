############################################################################
# graphics/Makefile
#
#   Copyright (C) 2008-2009, 2011 Gregory Nutt. All rights reserved.
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

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs

DEPPATH		= --dep-path .

ifeq ($(WINTOOL),y)
INCDIROPT	= -w
endif

include nxglib/Make.defs
DEPPATH		+= --dep-path nxglib
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxglib}

include nxbe/Make.defs
DEPPATH		+= --dep-path nxbe
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxbe}

ifeq ($(CONFIG_NX_MULTIUSER),y)
include nxmu/Make.defs
DEPPATH		+= --dep-path nxmu
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxmu}
else
include nxsu/Make.defs
DEPPATH		+= --dep-path nxsu
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxsu}
endif

include nxtk/Make.defs
DEPPATH		+= --dep-path nxtk
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxtk}

include nxfonts/Make.defs
DEPPATH		+= --dep-path nxfonts
CFLAGS		+= ${shell $(TOPDIR)/tools/incdir.sh $(INCDIROPT) "$(CC)" $(TOPDIR)/graphics/nxfonts}

ASRCS		= $(NXGLIB_ASRCS) $(NXBE_ASRCS) $(NX_ASRCS) $(NXTK_ASRCS) $(NXFONTS_ASRCS)
AOBJS		= $(ASRCS:.S=$(OBJEXT))

CSRCS		= $(NXGLIB_CSRCS) $(NXBE_CSRCS) $(NX_CSRCS) $(NXTK_CSRCS) $(NXFONTS_CSRCS)
COBJS		= $(CSRCS:.c=$(OBJEXT))

SRCS		= $(ASRCS) $(CSRCS)
OBJS		= $(AOBJS) $(COBJS)

BIN		= libgraphics$(LIBEXT)

ifeq ($(CONFIG_NX_MULTIUSER),y)
VPATH		= nxglib:nxbe:nxmu:nxtk:nxfonts
else
VPATH		= nxglib:nxbe:nxsu:nxtk:nxfonts
endif

all:	mklibgraphics
.PHONY : context depend clean distclean mklibgraphics gensources gen1bppsources \
	 gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource \
	 gen32bppsources

gen1bppsources:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=1 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=1 EXTRADEFINES=$(EXTRADEFINES)

gen2bppsource:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=2 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=2 EXTRADEFINES=$(EXTRADEFINES)

gen4bppsource:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=4 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=4 EXTRADEFINES=$(EXTRADEFINES)

gen8bppsource:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=8 EXTRADEFINES=$(EXTRADEFINES)

gen16bppsource:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=16 EXTRADEFINES=$(EXTRADEFINES)

gen24bppsource:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=24 EXTRADEFINES=$(EXTRADEFINES)

gen32bppsources:
	@$(MAKE) -C nxglib -f Makefile.sources TOPDIR=$(TOPDIR) NXGLIB_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources TOPDIR=$(TOPDIR) NXFONTS_BITSPERPIXEL=32 EXTRADEFINES=$(EXTRADEFINES)

gensources: gen1bppsources gen2bppsource gen4bppsource gen8bppsource gen16bppsource gen24bppsource gen32bppsources

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	@( for obj in $(OBJS) ; do \
		$(call ARCHIVE, $@, $${obj}); \
	done ; )

mklibgraphics: gensources $(BIN)

.depend: gensources Makefile $(SRCS)
	@$(MKDEP) $(DEPPATH) $(CC) -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

context: gensources

clean:
	@$(MAKE) -C nxglib -f Makefile.sources clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources clean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	@rm -f $(BIN) *~ .*.swp
	$(call CLEAN)

distclean: clean
	@$(MAKE) -C nxglib -f Makefile.sources distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	@$(MAKE) -C nxfonts -f Makefile.sources distclean TOPDIR=$(TOPDIR) EXTRADEFINES=$(EXTRADEFINES)
	@rm -f Make.dep .depend

-include Make.dep

