#include <avr/wdt.h>

// Causes an immediate reset to happen
#define RESET_HACK() { \
						void(* sw_reset) (void) = 0; \
						sw_reset(); \
					 }


// Return from interrupt to the specified address
// Potentially corrupts stack by pushing the new return address
#define replace_return_rti(return_addr)         \
(__extension__({                                \
      __asm__ __volatile__                      \
      (                                         \
        "ldi r31, %0\n\t"                       \
        "push r31\n\t"                          \
        "ldi r30, %0\n\t"                       \
        "push r30\n\t"                          \
        "reti\n\t"                              \
        :                                       \
        : "i" ((uint8_t)(return_addr))          \
        : "r0"                                  \
      );                                        \
}))
// #define replace_return_rti(return_addr)         \
// (__extension__({                                \
      // __asm__ __volatile__                      \
      // (                                         \
        // "ldi r31, %0\n\t"                       \
        // "push r31\n\t"                          \
        // "ldi r30, %1\n\t"                       \
        // "push r30\n\t"                          \
        // "reti\n\t"                              \
        // :                                       \
        // : "i" ((uint8_t)(return_addr)),         \
          // "i" ((uint8_t)(return_addr))          \
        // : "r0"                                  \
      // );                                        \
// }))

// #define __boot_lock_bits_set(lock_bits)                    \
// (__extension__({                                           \
    // uint8_t value = (uint8_t)(~(lock_bits));               \
    // __asm__ __volatile__                                   \
    // (                                                      \
        // "ldi r30, 1\n\t"                                   \
        // "ldi r31, 0\n\t"                                   \
        // "mov r0, %2\n\t"                                   \
        // "sts %0, %1\n\t"                                   \
        // "spm\n\t"                                          \
        // :                                                  \
        // : "i" (_SFR_MEM_ADDR(__SPM_REG)),                  \
          // "r" ((uint8_t)(__BOOT_LOCK_BITS_SET)),           \
          // "r" (value)                                      \
        // : "r0", "r30", "r31"                               \
    // );                                                     \
// }))


// @@@ WARNING: THIS WILL PREVENT THE WATCHDOG TIMER FROM INTERRUPTING IF THE 1xDELAY IS LESS THAN THE TIMEOUT @@@
void loopDelay(int n, bool resetWDT) { 
  while (n > 0) {
    if (resetWDT) {
      wdt_reset();
    }
    
    for (uint32_t i = 0; i < 400000; i++) { asm ("nop \n"); }
    
    n--;
  }
}

#define OUTPUT0 3
void debugFlashHexCode(uint8_t code, bool resetWDT) {
  noInterrupts();
  pinMode(OUTPUT0, OUTPUT);
  
  while(1) { // INFINITE LOOP
    digitalWrite(OUTPUT0, LOW);
    loopDelay(8, resetWDT);

    for (int i = (code & 0xF0) >> 4; i > 0; i--) {
      digitalWrite(OUTPUT0, HIGH);
      loopDelay(2, resetWDT);
      digitalWrite(OUTPUT0, LOW);
      loopDelay(1, resetWDT);
    }
  
    loopDelay(5, resetWDT);
  
    for (int i = code & 0x0F; i > 0; i--) {
      digitalWrite(OUTPUT0, HIGH);
      loopDelay(2, resetWDT);
      digitalWrite(OUTPUT0, LOW);
      loopDelay(1, resetWDT);
    }

    loopDelay(10, resetWDT);
  }
}

// void testFunc2();

// void testFunc1() {
  // testFunc2();
// }

// void testFunc2() {
// uint8_t *s;
  
  // Serial.print("testFunc2 -> 0x"), Serial.println((uint16_t)testFunc2, HEX);
  // Serial.print("testFunc1 -> 0x"), Serial.println((uint16_t)testFunc1), HEX;
  // Serial.println();
  
  // Serial.print("SP -> "), Serial.println((uint16_t)SP , HEX);
  // Serial.println();

  // // Show 'i' bytes of the stack, top to bottom (low address to high address)
  // s = SP;
  // for (int i = 0; i < 31; i++) {
	// Serial.print("[0x"); Serial.print((uint16_t)s , HEX);Serial.print("] *(SP+"); Serial.print(i); Serial.print(") -> 0x"), Serial.println( *s , HEX );
	// s++;
  // }

  // Serial.print("SP -> "), Serial.println((uint16_t)SP , HEX);

  // while(1);

  // // void(** return_func) (void) = SP;
  // // Serial.println((uint16_t)return_func, HEX);

  // // *return_func = printMCP2515Registers;
  // // Serial.println((uint16_t)return_func, HEX);

  // // while(1);
// }
