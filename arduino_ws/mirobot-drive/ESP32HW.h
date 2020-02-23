#pragma once

/* DC motors signals */
/* Left Motor */
#define IN1 2         // ESP pin RX2 is connected to MDDS30 pin IN1.
#define AN1 16        // ESP pin D4  is connected to MDDS30 pin AN1.
/* Right MOtor */
#define AN2 4         // ESP pin D2  is connected to MDDS30 pin AN2.
#define IN2 15        // ESP pin D15 is connected to MDDS30 pin IN2.

/* Encoder signals */
/* Left wheel encoder */
#define LW_ENCODER_A 13  // ESP pin RX2 is connected to ENCODER wire YELLOW.
#define LW_ENCODER_B 12  // ESP pin RX2 is connected to ENCODER wire WHITE.
/* Right wheel encoder */
#define RW_ENCODER_A 14  // ESP pin RX2 is connected to ENCODER wire YELLOW.
#define RW_ENCODER_B 27  // ESP pin RX2 is connected to ENCODER wire WHITE.

#define ENCODER_2PI_STEPS (544)  // steps per one revolution
