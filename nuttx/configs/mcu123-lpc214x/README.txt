README
^^^^^^

This README discusses issues unique to NuttX configurations for the
MCU-123 LPC2148 development board.

Development Environment
^^^^^^^^^^^^^^^^^^^^^^^

  Either Linux or Cygwin on Windows can be used for the development environment.
  The source has been built only using the GNU toolchain (see below).  Other
  toolchains will likely cause problems.

GNU Toolchain Options
^^^^^^^^^^^^^^^^^^^^^

  The NuttX make system has been modified to support the following different
  toolchain options.

  1. The NuttX buildroot Toolchain (see below).
  2. The CodeSourcery GNU toolchain,
  3. The devkitARM GNU toolchain, or
 
  All testing has been conducted using the NuttX buildroot toolchain.  To use
  the CodeSourcery or devkitARM GNU toolchain, you simply need to build the
  system as follows:

     make                         # Will build for the NuttX buildroot toolchain
     make CROSSDEV=arm-eabi-      # Will build for the devkitARM toolchain
     make CROSSDEV=arm-none-eabi- # Will build for the CodeSourcery toolchain
     make CROSSDEV=arm-elf-       # Will build for the NuttX buildroot toolchain

  Of course, hard coding this CROSS_COMPILE value in Make.defs file will save
  some repetitive typing.

  NOTE: the CodeSourcery and devkitARM toolchains are Windows native toolchains.
  The NuttX buildroot toolchain is a Cygwin toolchain.  There are several limitations
  to using a Windows based toolchain in a Cygwin environment.  The three biggest are:

  1. The Windows toolchain cannot follow Cygwin paths.  Path conversions are
     performed automatically in the Cygwin makefiles using the 'cygpath' utility
     but you might easily find some new path problems.  If so, check out 'cygpath -w'

  2. Windows toolchains cannot follow Cygwin symbolic links.  Many symbolic links
     are used in Nuttx (e.g., include/arch).  The make system works around these
     problems for the Windows tools by copying directories instead of linking them.
     But this can also cause some confusion for you:  For example, you may edit
     a file in a "linked" directory and find that your changes had not effect.
     That is because you are building the copy of the file in the "fake" symbolic
     directory.  If you use a Windows toolchain, you should get in the habit of
     making like this:

       make clean_context; make CROSSDEV=arm-none-eabi-

     An alias in your .bashrc file might make that less painful.

  3. Dependencies are not made when using Windows versions of the GCC.  This is
     because the dependencies are generated using Windows pathes which do not
     work with the Cygwin make.

     Support has been added for making dependencies with the CodeSourcery toolchain.
     That support can be enabled by modifying your Make.defs file as follows:

    -  MKDEP                = $(TOPDIR)/tools/mknulldeps.sh
    +  MKDEP                = $(TOPDIR)/tools/mkdeps.sh --winpaths "$(TOPDIR)"

     If you have problems with the dependency build (for example, if you are not
     building on C:), then you may need to modify tools/mkdeps.sh

  NOTE 1: The CodeSourcery toolchain (2009q1) may not work with default optimization
  level of -Os (See Make.defs).  It will work with -O0, -O1, or -O2, but not with
  -Os.

  NOTE 2: The devkitARM toolchain includes a version of MSYS make.  Make sure that
  the paths to Cygwin's /bin and /usr/bin directories appear BEFORE the devkitARM
  path or will get the wrong version of make.

NuttX buildroot Toolchain
^^^^^^^^^^^^^^^^^^^^^^^^^

  A GNU GCC-based toolchain is assumed.  The files */setenv.sh should
  be modified to point to the correct path to the Cortex-M3 GCC toolchain (if
  different from the default in your PATH variable).

  If you have no Cortex-M3 toolchain, one can be downloaded from the NuttX
  SourceForge download site (https://sourceforge.net/project/showfiles.php?group_id=189573).
  This GNU toolchain builds and executes in the Linux or Cygwin environment.

  1. You must have already configured Nuttx in <some-dir>/nuttx.

     cd tools
     ./configure.sh eagle100/<sub-dir>

  2. Download the latest buildroot package into <some-dir>

  3. unpack the buildroot tarball.  The resulting directory may
     have versioning information on it like buildroot-x.y.z.  If so,
     rename <some-dir>/buildroot-x.y.z to <some-dir>/buildroot.

  4. cd <some-dir>/buildroot

  5. cp configs/cortexm3-defconfig-4.3.3 .config

  6. make oldconfig

  7. make

  8. Edit setenv.h, if necessary, so that the PATH variable includes
     the path to the newly built binaries.

  See the file configs/README.txt in the buildroot source tree.  That has more
  detailed PLUS some special instructions that you will need to follow if you are
  building a Cortex-M3 toolchain for Cygwin under Windows.

Flash Tools
^^^^^^^^^^^

I use the lpc21isp tool to load NuttX into FLASH.  That tool is available
in the files section at http://tech.groups.yahoo.com/group/lpc21isp/.  In
order version 1.60 of lpc21isp for Linux, I had to make several changes.
This changes are shown in lpc21ips-1.60.diff.

I use the script lpc21isp.sh to perform the actual download.  You will
probably have to make some changes to this script in order to use it.
For example, the path to the built lpc21isp binary will most likely
have to change.  Then move this script to the top level NuttX
directory and simply execute it to load NuttX onto the board (after
entering ISP mode).

Here are the detailed steps I use:

1. Make sure you exit minicom (or whatever terminal emulator you are
   using).  It will interfere with the download.

2. On the MCU123 board, I need to put a jumper on JP3-INT. On that board,
   JP3-INT is connected to P0.14 of LPC214x. When P0.14 is low and RTS is
   changed from high to low, the LPC214x will enter ISP (In System Programming) 
   state.

3. start lpc21isp.sh

4. reset the board


ARM/LPC214X-specific Configuration Options
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

	CONFIG_ARCH - Identifies the arch/ subdirectory.  This should
	   be set to:

	   CONFIG_ARCH=arm

	CONFIG_ARCH_family - For use in C code:

	   CONFIG_ARCH_ARM=y

	CONFIG_ARCH_architecture - For use in C code:

	   CONFIG_ARCH_ARM7TDMI=y

	CONFIG_ARCH_CHIP - Identifies the arch/*/chip subdirectory

	   CONFIG_ARCH_CHIP=lpc214x

	CONFIG_ARCH_CHIP_name - For use in C code

	   CONFIG_ARCH_CHIP_LPC214X

	CONFIG_ARCH_BOARD - Identifies the configs subdirectory and
	   hence, the board that supports the particular chip or SoC.

	   CONFIG_ARCH_BOARD=mcu123-lpc214x

	CONFIG_ARCH_BOARD_name - For use in C code

	   CONFIG_ARCH_BOARD_MCU123 (for the Spectrum Digital C5471 EVM)

	CONFIG_ARCH_LOOPSPERMSEC - Must be calibrated for correct operation
	   of delay loops

	CONFIG_ENDIAN_BIG - define if big endian (default is little
	   endian)

	CONFIG_DRAM_SIZE - Describes the installed RAM.

	CONFIG_DRAM_START - The start address of installed RAM

	CONFIG_DRAM_END - Should be (CONFIG_DRAM_START+CONFIG_DRAM_SIZE)

	CONFIG_ARCH_LEDS - Use LEDs to show state. Unique to boards that
	   have LEDs

	CONFIG_ARCH_INTERRUPTSTACK - This architecture supports an interrupt
	   stack. If defined, this symbol is the size of the interrupt
	   stack in bytes.  If not defined, the user task stacks will be
	  used during interrupt handling.

	CONFIG_ARCH_STACKDUMP - Do stack dumps after assertions

	CONFIG_ARCH_CALIBRATION - Enables some build in instrumentation that
	   cause a 100 second delay during boot-up.  This 100 second delay
	   serves no purpose other than it allows you to calibratre
	   CONFIG_ARCH_LOOPSPERMSEC.  You simply use a stop watch to measure
	   the 100 second delay then adjust CONFIG_ARCH_LOOPSPERMSEC until
	   the delay actually is 100 seconds.

  LPC2148 specific chip initialization

    These provide register setup values:
	CONFIG_EXTMEM_MODE, CONFIG_RAM_MODE, CONFIG_CODE_BASE, CONFIG_PLL_SETUP,
	CONFIG_MAM_SETUP, CONFIG_APBDIV_SETUP, CONFIG_EMC_SETUP, CONFIG_BCFG0_SETUP,
	CONFIG_BCFG1_SETUP, CONFIG_BCFG2_SETUP, CONFIG_BCFG3_SETUP, CONFIG_ADC_SETUP

	CONFIG_LPC214x_FIO - Enable fast GPIO (vs. legacy, "old" GPIO).

  LPC214X specific device driver settings

	CONFIG_UARTn_SERIAL_CONSOLE - selects the UARTn for the
	   console and ttys0 (default is the UART0).

	CONFIG_UARTn_RXBUFSIZE - Characters are buffered as received.
	   This specific the size of the receive buffer

	CONFIG_UARTn_TXBUFSIZE - Characters are buffered before
	   being sent.  This specific the size of the transmit buffer

	CONFIG_UARTn_BAUD - The configure BAUD of the UART.

	CONFIG_UARTn_BITS - The number of bits.  Must be either 7 or 8.

	CONFIG_UARTn_PARTIY - 0=no parity, 1=odd parity, 2=even parity, 3=mark 1, 4=space 0

	CONFIG_UARTn_2STOP - Two stop bits

  LPC214X USB Configuration

	CONFIG_LPC214X_USBDEV_FRAME_INTERRUPT
	   Handle USB Start-Of-Frame events. 
	   Enable reading SOF from interrupt handler vs. simply reading on demand.
	   Probably a bad idea... Unless there is some issue with sampling the SOF
	   from hardware asynchronously.

	CONFIG_LPC214X_USBDEV_EPFAST_INTERRUPT
	   Enable high priority interrupts.  I have no idea why you might want to
	   do that

	CONFIG_LPC214X_USBDEV_NDMADESCRIPTORS
	   Number of DMA descriptors to allocate in the 8Kb USB RAM.  This is a
	   tradeoff between the number of DMA channels that can be supported vs
	   the size of the DMA buffers available.

	CONFIG_LPC214X_USBDEV_DMA
	   Enable lpc214x-specific DMA support

Configurations
^^^^^^^^^^^^^^

Each NXP LPC214x configuration is maintained in a sudirectory and
can be selected as follow:

	cd tools
	./configure.sh mcu123-lpc214x/<subdir>
	cd -
	. ./setenv.sh

Where <subdir> is one of the following:

ostest:
  This configuration directory, performs a simple OS test using
  examples/ostest.

nsh:
  Configures the NuttShell (nsh) located at examples/nsh.  The
  Configuration enables only the serial NSH interfaces.

usbserial:
  This configuration directory exercises the USB serial class
  driver at examples/usbserial.  See examples/README.txt for
  more information.

usbstorage:
  This configuration directory exercises the USB mass storage
  class driver at examples/usbstorage.  See examples/README.txt for
  more information.
