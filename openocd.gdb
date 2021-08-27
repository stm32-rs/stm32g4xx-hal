target remote :3333

# monitor tpiu config internal /tmp/itm.fifo uart off 2000000
# monitor itm port 0 on

set print asm-demangle on
monitor arm semihosting enable
load
continue
