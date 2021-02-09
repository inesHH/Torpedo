
# Clone

```
git clone --recurse-submodules https://github.com/FTOD-LAB/EzSTM32.git
```

If you have an older git, or got ahead of yourself and skipped the 
```
--recurse-submodules
```
you can fix things by running 
```
git submodule update --init
``` 
(This is only needed once)


# Directories
* my-project contains your application
* my-common-code contains something shared.

# Build & Flash
    1. make -C libopencm3 # (Only needed once)
    2. make -C my-project
    3. st-flash write my-project/awesomesauce.bin 0x8000000

# Debug: 
    1.  st-util
    2.  run gdb
    2.  connect gdb to the port with : target remote [port]

# TODO
    1.  Add to the code VSP support to communicate with PC via microUSB port;
    2.  st-util is open-source version of st-util of ST. We may want to consider openocd for flash & debug. However, the st-util seems to be sufficient.
