############################################################################
# insn16/Makefile
#
#   Copyright (C) 2008 Gregory Nutt. All rights reserved.
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
#
# Directories
#
INSNDIR		= ${shell pwd}
PASCAL		= $(INSNDIR)/..

include $(PASCAL)/Make.config
include $(PASCAL)/Make.defs

LIBDIR		= $(PASCAL)/lib
BINDIR		= $(PASCAL)/bin16
LIBINSNDIR	= $(INSNDIR)/libinsn
POPTDIR		= $(INSNDIR)/popt
PRUNDIR		= $(INSNDIR)/prun
PLISTDIR	= $(INSNDIR)/plist

#
# Objects and targets
#
all: libinsn popt prun plist
PHONY: all libinsn popt prun plist clean

$(LIBINSNDIR)/libinsn.a:
	$(MAKE) -C $(LIBINSNDIR)

libinsn.a: $(LIBINSNDIR)/libinsn.a

$(BINDIR)/popt:
	$(MAKE) -C $(POPTDIR)

popt: $(BINDIR)/popt

$(BINDIR)/prun:
	$(MAKE) -C $(PRUNDIR)

prun: $(BINDIR)/prun

$(BINDIR)/plist:
	$(MAKE) -C $(PLISTDIR)

plist: $(BINDIR)/plist

clean:
	$(RM) libinsn.a core *~
	$(MAKE) -C $(LIBINSNDIR) clean
	$(MAKE) -C $(POPTDIR) clean
	$(MAKE) -C $(PRUNDIR) clean
	$(MAKE) -C $(PLISTDIR) clean
