MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* TODO Adjust these memory regions to match your device memory layout */
  FLASH : ORIGIN = 0x8000000, LENGTH = 128K
  RAM : ORIGIN = 0x20000000, LENGTH = 16K
  RAM2: ORIGIN = 0x20004000, LENGTH = 6K
  CCMRAM: ORIGIN = 0x10000000, LENGTH = 10K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);