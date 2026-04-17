# Test Plan (prizm_firmware)

1. Build check: `pio run`.
2. SPI link bring-up:
   - verify HELLO/HELLO_ACK exchange.
   - verify CAPABILITIES and STATUS reads.
3. Startup lock:
   - force compute offline => `FAULT_LOCK`, motors stopped.
   - restore compute health => transition to READY.
4. Safety check:
   - ensure wheel + M1/M2 commands are zero while run lock active.
5. Stage 2 smoke:
   - CH10+CH3 combo toggles mode manager state only after lock released.
