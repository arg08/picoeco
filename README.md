# picoeco
Support for Acorn Econet protocols on the Raspberry Pi Pico



How to build
------------

Install the pico-sdk somewhere and set PICO_SDK_PATH to point at it.

git clone -b master https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init
setenv PICO_SDK_PATH `pwd`

Fetch this repository:

git clone https://github.com/arg08/picoeco.git

Build one of the projects within it:

cd picoeco/ecotest
mkdir build
cd build
cmake ..
make

(similarly for picoeco/netmon etc)
This generates a binary ecotest.uf2

How to install
--------------
Get the pico into boot mode by powering up with the button on the pico
held (or hold down the button on the pico and press the reset button).

Either mount the pico as if it were a USB memory stick and copy the .uf2
file onto it, or else build picotool and use:

 picotool load netmon.uf2 -x



Building on FreeBSD notes
-------------------------
Standard Pi documentation covers Linux:
 https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf

To build on a FreeBSD PC host, you need the following packages installed:

cmake
pkgconf
gcc-arm-embedded	(NB. _not_ arm-none-eabi-gcc, it doesn't work!)
git

You then need to set PICO_TOOLCHAIN_PATH=/usr/local/gcc-arm-embedded
as well as PICO_SDK_PATH

I find picotool more useful than the USB mass storage stuff; it
almost builds per the Linux instructions, except the libusb
naming is different so needs a tiny patch to cmake/findLIBUSB.cmake
in the distribution:

git clone -b master https://github.com/raspberrypi/picotool.git

--- a/cmake/FindLIBUSB.cmake
+++ b/cmake/FindLIBUSB.cmake
@@ -20,7 +20,7 @@ else (LIBUSB_INCLUDE_DIR AND LIBUSB_LIBRARIES)
     ENDIF(NOT WIN32)
     FIND_PATH(LIBUSB_INCLUDE_DIR libusb.h
             PATHS ${PC_LIBUSB_INCLUDEDIR} ${PC_LIBUSB_INCLUDE_DIRS})
-    FIND_LIBRARY(LIBUSB_LIBRARIES NAMES usb-1.0
+    FIND_LIBRARY(LIBUSB_LIBRARIES NAMES usb-1.0 usb
             PATHS ${PC_LIBUSB_LIBDIR} ${PC_LIBUSB_LIBRARY_DIRS})
     include(FindPackageHandleStandardArgs)
     FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBUSB DEFAULT_MSG LIBUSB_LIBRARIES LIBUSB_INCLUDE_DIR)

ie. just add "usb" after "usb-1.0" on the FIND_LIBRARY line.

NB. FreeBSD has libusb in the base system rather than a package.