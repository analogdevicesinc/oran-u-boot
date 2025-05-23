/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * (C) Copyright 2025 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#ifndef __ADI_PERIPHERAL_TWI__
#define __ADI_PERIPHERAL_TWI__

#define CLKLOW(x) ((x) & 0xFF)          // Periods Clock Is Held Low
#define CLKHI(y) (((y) & 0xFF) << 0x8)  // Periods Clock Is High

#define PRESCALE        0x007F          // SCLKs Per Internal Time Reference (10MHz)
#define TWI_ENA         0x0080          // TWI Enable
#define SCCB            0x0200          // SCCB Compatibility Enable

#define SEN             0x0001          // Slave Enable
#define SADD_LEN        0x0002          // Slave Address Length
#define STDVAL          0x0004          // Slave Transmit Data Valid
#define TSC_NAK         0x0008          // NAK Generated At Conclusion Of Transfer
#define GEN             0x0010          // General Call Adrress Matching Enabled

#define SDIR            0x0001          // Slave Transfer Direction
#define GCALL           0x0002          // General Call Indicator

#define MEN             0x0001          // Master Mode Enable
#define MADD_LEN        0x0002          // Master Address Length
#define MDIR            0x0004          // Master Transmit Direction (RX/TX*)
#define FAST            0x0008          // Use Fast Mode Timing Specs
#define STOP            0x0010          // Issue Stop Condition
#define RSTART          0x0020          // Repeat Start or Stop* At End Of Transfer
#define DCNT            0x3FC0          // Data Bytes To Transfer
#define SDAOVR          0x4000          // Serial Data Override
#define SCLOVR          0x8000          // Serial Clock Override

#define MPROG           0x0001          // Master Transfer In Progress
#define LOSTARB         0x0002          // Lost Arbitration Indicator (Xfer Aborted)
#define ANAK            0x0004          // Address Not Acknowledged
#define DNAK            0x0008          // Data Not Acknowledged
#define BUFRDERR        0x0010          // Buffer Read Error
#define BUFWRERR        0x0020          // Buffer Write Error
#define SDASEN          0x0040          // Serial Data Sense
#define SCLSEN          0x0080          // Serial Clock Sense
#define BUSBUSY         0x0100          // Bus Busy Indicator

#define SINIT           0x0001          // Slave Transfer Initiated
#define SCOMP           0x0002          // Slave Transfer Complete
#define SERR            0x0004          // Slave Transfer Error
#define SOVF            0x0008          // Slave Overflow
#define MCOMP           0x0010          // Master Transfer Complete
#define MERR            0x0020          // Master Transfer Error
#define XMTSERV         0x0040          // Transmit FIFO Service
#define RCVSERV         0x0080          // Receive FIFO Service

#define XMTFLUSH        0x0001          // Transmit Buffer Flush
#define RCVFLUSH        0x0002          // Receive Buffer Flush
#define XMTINTLEN       0x0004          // Transmit Buffer Interrupt Length
#define RCVINTLEN       0x0008          // Receive Buffer Interrupt Length

#define XMTSTAT         0x0003          // Transmit FIFO Status
#define XMT_EMPTY       0x0000          // Transmit FIFO Empty
#define XMT_HALF        0x0001          // Transmit FIFO Has 1 Byte To Write
#define XMT_FULL        0x0003          // Transmit FIFO Full (2 Bytes To Write)

#define RCVSTAT         0x000C          // Receive FIFO Status
#define RCV_EMPTY       0x0000          // Receive FIFO Empty
#define RCV_HALF        0x0004          // Receive FIFO Has 1 Byte To Read
#define RCV_FULL        0x000C          // Receive FIFO Full (2 Bytes To Read)

#endif
