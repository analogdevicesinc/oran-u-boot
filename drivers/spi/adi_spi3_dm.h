// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Converted to driver model by Nathan Barrett-Morrison
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#ifndef _SPI_CHANNEL_H_
#define _SPI_CHANNEL_H_

#include <linux/types.h>

#define MAX_CTRL_CS 7
#define gpio_cs(cs)         ((cs) - MAX_CTRL_CS)
#define is_gpio_cs(cs)      ((cs) >= MAX_CTRL_CS)

/* SPI_CONTROL */
#define SPI_CTL_EN          0x00000001          /* Enable */
#define SPI_CTL_MSTR        0x00000002          /* Master/Slave */
#define SPI_CTL_PSSE        0x00000004          /* controls modf error in master mode */
#define SPI_CTL_ODM         0x00000008          /* Open Drain Mode */
#define SPI_CTL_CPHA        0x00000010          /* Clock Phase */
#define SPI_CTL_CPOL        0x00000020          /* Clock Polarity */
#define SPI_CTL_ASSEL       0x00000040          /* Slave Select Pin Control */
#define SPI_CTL_SELST       0x00000080          /* Slave Select Polarity in transfers */
#define SPI_CTL_EMISO       0x00000100          /*Enable MISO */
#define SPI_CTL_SIZE        0x00000600          /*Word Transfer Size */
#define SPI_CTL_SIZE08      0x00000000          /*SIZE: 8 bits */
#define SPI_CTL_SIZE16      0x00000200          /*SIZE: 16 bits */
#define SPI_CTL_SIZE32      0x00000400          /*SIZE: 32 bits */
#define SPI_CTL_LSBF        0x00001000          /*LSB First */
#define SPI_CTL_FCEN        0x00002000          /*Flow-Control Enable */
#define SPI_CTL_FCCH        0x00004000          /*Flow-Control Channel Selection */
#define SPI_CTL_FCPL        0x00008000          /*Flow-Control Polarity */
#define SPI_CTL_FCWM        0x00030000          /*Flow-Control Water-Mark */
#define SPI_CTL_FIFO0       0x00000000          /*FCWM: Tx empty or Rx Full */
#define SPI_CTL_FIFO1       0x00010000          /*FCWM: Tx empty or Rx full (>=75%) */
#define SPI_CTL_FIFO2       0x00020000          /*FCWM: Tx empty or Rx full (>=50%) */
#define SPI_CTL_FMODE       0x00040000          /*Fast-mode Enable */
#define SPI_CTL_MIOM        0x00300000          /*Multiple I/O Mode */
#define SPI_CTL_MIO_DIS     0x00000000          /*MIOM: Disable */
#define SPI_CTL_MIO_DUAL    0x00100000          /*MIOM: Enable DIOM (Dual I/O Mode) */
#define SPI_CTL_MIO_QUAD    0x00200000          /*MIOM: Enable QUAD (Quad SPI Mode) */
#define SPI_CTL_SOSI        0x00400000          /*Start on MOSI */
#define SPI_CTL_MMWEM       0x40000000          /*Start on MMWEM */
#define SPI_CTL_MMSE        0x80000000          /*Start on MMSE */
/* SPI_RX_CONTROL */
#define SPI_RXCTL_REN       0x00000001          /*Receive Channel Enable */
#define SPI_RXCTL_RTI       0x00000004          /*Receive Transfer Initiate */
#define SPI_RXCTL_RWCEN     0x00000008          /*Receive Word Counter Enable */
#define SPI_RXCTL_RDR       0x00000070          /*Receive Data Request */
#define SPI_RXCTL_RDR_DIS   0x00000000          /*RDR: Disabled */
#define SPI_RXCTL_RDR_NE    0x00000010          /*RDR: RFIFO not empty */
#define SPI_RXCTL_RDR_25    0x00000020          /*RDR: RFIFO 25% full */
#define SPI_RXCTL_RDR_50    0x00000030          /*RDR: RFIFO 50% full */
#define SPI_RXCTL_RDR_75    0x00000040          /*RDR: RFIFO 75% full */
#define SPI_RXCTL_RDR_FULL  0x00000050          /*RDR: RFIFO full */
#define SPI_RXCTL_RDO       0x00000100          /*Receive Data Over-Run */
#define SPI_RXCTL_RRWM      0x00003000          /*FIFO Regular Water-Mark */
#define SPI_RXCTL_RWM_0     0x00000000          /*RRWM: RFIFO Empty */
#define SPI_RXCTL_RWM_25    0x00001000          /*RRWM: RFIFO 25% full */
#define SPI_RXCTL_RWM_50    0x00002000          /*RRWM: RFIFO 50% full */
#define SPI_RXCTL_RWM_75    0x00003000          /*RRWM: RFIFO 75% full */
#define SPI_RXCTL_RUWM      0x00070000          /*FIFO Urgent Water-Mark */
#define SPI_RXCTL_UWM_DIS   0x00000000          /*RUWM: Disabled */
#define SPI_RXCTL_UWM_25    0x00010000          /*RUWM: RFIFO 25% full */
#define SPI_RXCTL_UWM_50    0x00020000          /*RUWM: RFIFO 50% full */
#define SPI_RXCTL_UWM_75    0x00030000          /*RUWM: RFIFO 75% full */
#define SPI_RXCTL_UWM_FULL  0x00040000          /*RUWM: RFIFO full */
/* SPI_TX_CONTROL */
#define SPI_TXCTL_TEN       0x00000001          /*Transmit Channel Enable */
#define SPI_TXCTL_TTI       0x00000004          /*Transmit Transfer Initiate */
#define SPI_TXCTL_TWCEN     0x00000008          /*Transmit Word Counter Enable */
#define SPI_TXCTL_TDR       0x00000070          /*Transmit Data Request */
#define SPI_TXCTL_TDR_DIS   0x00000000          /*TDR: Disabled */
#define SPI_TXCTL_TDR_NF    0x00000010          /*TDR: TFIFO not full */
#define SPI_TXCTL_TDR_25    0x00000020          /*TDR: TFIFO 25% empty */
#define SPI_TXCTL_TDR_50    0x00000030          /*TDR: TFIFO 50% empty */
#define SPI_TXCTL_TDR_75    0x00000040          /*TDR: TFIFO 75% empty */
#define SPI_TXCTL_TDR_EMPTY 0x00000050          /*TDR: TFIFO empty */
#define SPI_TXCTL_TDU       0x00000100          /*Transmit Data Under-Run */
#define SPI_TXCTL_TRWM      0x00003000          /*FIFO Regular Water-Mark */
#define SPI_TXCTL_RWM_FULL  0x00000000          /*TRWM: TFIFO full */
#define SPI_TXCTL_RWM_25    0x00001000          /*TRWM: TFIFO 25% empty */
#define SPI_TXCTL_RWM_50    0x00002000          /*TRWM: TFIFO 50% empty */
#define SPI_TXCTL_RWM_75    0x00003000          /*TRWM: TFIFO 75% empty */
#define SPI_TXCTL_TUWM      0x00070000          /*FIFO Urgent Water-Mark */
#define SPI_TXCTL_UWM_DIS   0x00000000          /*TUWM: Disabled */
#define SPI_TXCTL_UWM_25    0x00010000          /*TUWM: TFIFO 25% empty */
#define SPI_TXCTL_UWM_50    0x00020000          /*TUWM: TFIFO 50% empty */
#define SPI_TXCTL_UWM_75    0x00030000          /*TUWM: TFIFO 75% empty */
#define SPI_TXCTL_UWM_EMPTY 0x00040000          /*TUWM: TFIFO empty */
/* SPI_CLOCK */
#define SPI_CLK_BAUD        0x0000FFFF          /*Baud Rate */
/* SPI_DELAY */
#define SPI_DLY_STOP        0x000000FF          /*Transfer delay time */
#define SPI_DLY_LEADX       0x00000100          /*Extended (1 SCK) LEAD Control */
#define SPI_DLY_LAGX        0x00000200          /*Extended (1 SCK) LAG control */
/* SPI_SSEL */
#define SPI_SLVSEL_SSE1     0x00000002          /*SPISSEL1 Enable */
#define SPI_SLVSEL_SSE2     0x00000004          /*SPISSEL2 Enable */
#define SPI_SLVSEL_SSE3     0x00000008          /*SPISSEL3 Enable */
#define SPI_SLVSEL_SSE4     0x00000010          /*SPISSEL4 Enable */
#define SPI_SLVSEL_SSE5     0x00000020          /*SPISSEL5 Enable */
#define SPI_SLVSEL_SSE6     0x00000040          /*SPISSEL6 Enable */
#define SPI_SLVSEL_SSE7     0x00000080          /*SPISSEL7 Enable */
#define SPI_SLVSEL_SSEL1    0x00000200          /*SPISSEL1 Value */
#define SPI_SLVSEL_SSEL2    0x00000400          /*SPISSEL2 Value */
#define SPI_SLVSEL_SSEL3    0x00000800          /*SPISSEL3 Value */
#define SPI_SLVSEL_SSEL4    0x00001000          /*SPISSEL4 Value */
#define SPI_SLVSEL_SSEL5    0x00002000          /*SPISSEL5 Value */
#define SPI_SLVSEL_SSEL6    0x00004000          /*SPISSEL6 Value */
#define SPI_SLVSEL_SSEL7    0x00008000          /*SPISSEL7 Value */
/* SPI_RWC */
#define SPI_RWC_VALUE       0x0000FFFF          /*Received Word-Count */
/* SPI_RWCR */
#define SPI_RWCR_VALUE      0x0000FFFF          /*Received Word-Count Reload */
/* SPI_TWC */
#define SPI_TWC_VALUE       0x0000FFFF          /*Transmitted Word-Count */
/* SPI_TWCR */
#define SPI_TWCR_VALUE      0x0000FFFF          /*Transmitted Word-Count Reload */
/* SPI_IMASK */
#define SPI_IMSK_RUWM       0x00000002          /*Receive Water-Mark Interrupt Mask */
#define SPI_IMSK_TUWM       0x00000004          /*Transmit Water-Mark Interrupt Mask */
#define SPI_IMSK_ROM        0x00000010          /*Receive Over-Run Interrupt Mask */
#define SPI_IMSK_TUM        0x00000020          /*Transmit Under-Run Interrupt Mask */
#define SPI_IMSK_TCM        0x00000040          /*Transmit Collision Interrupt Mask */
#define SPI_IMSK_MFM        0x00000080          /*Mode Fault Interrupt Mask */
#define SPI_IMSK_RSM        0x00000100          /*Receive Start Interrupt Mask */
#define SPI_IMSK_TSM        0x00000200          /*Transmit Start Interrupt Mask */
#define SPI_IMSK_RFM        0x00000400          /*Receive Finish Interrupt Mask */
#define SPI_IMSK_TFM        0x00000800          /*Transmit Finish Interrupt Mask */
/* SPI_IMASKCL */
#define SPI_IMSK_CLR_RUW    0x00000002          /*Receive Water-Mark Interrupt Mask */
#define SPI_IMSK_CLR_TUWM   0x00000004          /*Transmit Water-Mark Interrupt Mask */
#define SPI_IMSK_CLR_ROM    0x00000010          /*Receive Over-Run Interrupt Mask */
#define SPI_IMSK_CLR_TUM    0x00000020          /*Transmit Under-Run Interrupt Mask */
#define SPI_IMSK_CLR_TCM    0x00000040          /*Transmit Collision Interrupt Mask */
#define SPI_IMSK_CLR_MFM    0x00000080          /*Mode Fault Interrupt Mask */
#define SPI_IMSK_CLR_RSM    0x00000100          /*Receive Start Interrupt Mask */
#define SPI_IMSK_CLR_TSM    0x00000200          /*Transmit Start Interrupt Mask */
#define SPI_IMSK_CLR_RFM    0x00000400          /*Receive Finish Interrupt Mask */
#define SPI_IMSK_CLR_TFM    0x00000800          /*Transmit Finish Interrupt Mask */
/* SPI_IMASKST */
#define SPI_IMSK_SET_RUWM   0x00000002          /*Receive Water-Mark Interrupt Mask */
#define SPI_IMSK_SET_TUWM   0x00000004          /*Transmit Water-Mark Interrupt Mask */
#define SPI_IMSK_SET_ROM    0x00000010          /*Receive Over-Run Interrupt Mask */
#define SPI_IMSK_SET_TUM    0x00000020          /*Transmit Under-Run Interrupt Mask */
#define SPI_IMSK_SET_TCM    0x00000040          /*Transmit Collision Interrupt Mask */
#define SPI_IMSK_SET_MFM    0x00000080          /*Mode Fault Interrupt Mask */
#define SPI_IMSK_SET_RSM    0x00000100          /*Receive Start Interrupt Mask */
#define SPI_IMSK_SET_TSM    0x00000200          /*Transmit Start Interrupt Mask */
#define SPI_IMSK_SET_RFM    0x00000400          /*Receive Finish Interrupt Mask */
#define SPI_IMSK_SET_TFM    0x00000800          /*Transmit Finish Interrupt Mask */
/* SPI_STATUS */
#define SPI_STAT_SPIF       0x00000001          /*SPI Finished */
#define SPI_STAT_RUWM       0x00000002          /*Receive Water-Mark Breached */
#define SPI_STAT_TUWM       0x00000004          /*Transmit Water-Mark Breached */
#define SPI_STAT_ROE        0x00000010          /*Receive Over-Run Indication */
#define SPI_STAT_TUE        0x00000020          /*Transmit Under-Run Indication */
#define SPI_STAT_TCE        0x00000040          /*Transmit Collision Indication */
#define SPI_STAT_MODF       0x00000080          /*Mode Fault Indication */
#define SPI_STAT_RS         0x00000100          /*Receive Start Indication */
#define SPI_STAT_TS         0x00000200          /*Transmit Start Indication */
#define SPI_STAT_RF         0x00000400          /*Receive Finish Indication */
#define SPI_STAT_TF         0x00000800          /*Transmit Finish Indication */
#define SPI_STAT_RFS        0x00007000          /*SPI_RFIFO status */
#define SPI_STAT_RFIFO_EMPTY 0x00000000         /*RFS: RFIFO Empty */
#define SPI_STAT_RFIFO_25   0x00001000          /*RFS: RFIFO 25% Full */
#define SPI_STAT_RFIFO_50   0x00002000          /*RFS: RFIFO 50% Full */
#define SPI_STAT_RFIFO_75   0x00003000          /*RFS: RFIFO 75% Full */
#define SPI_STAT_RFIFO_FULL 0x00004000          /*RFS: RFIFO Full */
#define SPI_STAT_TFS        0x00070000          /*SPI_TFIFO status */
#define SPI_STAT_TFIFO_FULL 0x00000000          /*TFS: TFIFO full */
#define SPI_STAT_TFIFO_25   0x00010000          /*TFS: TFIFO 25% empty */
#define SPI_STAT_TFIFO_50   0x00020000          /*TFS: TFIFO 50% empty */
#define SPI_STAT_TFIFO_75   0x00030000          /*TFS: TFIFO 75% empty */
#define SPI_STAT_TFIFO_EMPTY 0x00040000         /*TFS: TFIFO empty */
#define SPI_STAT_FCS        0x00100000          /*Flow-Control Stall Indication */
#define SPI_STAT_RFE        0x00400000          /*SPI_RFIFO Empty */
#define SPI_STAT_TFF        0x00800000          /*SPI_TFIFO Full */
/* SPI_ILAT */
#define SPI_ILAT_RUWMI      0x00000002          /*Receive Water Mark Interrupt */
#define SPI_ILAT_TUWMI      0x00000004          /*Transmit Water Mark Interrupt */
#define SPI_ILAT_ROI        0x00000010          /*Receive Over-Run Indication */
#define SPI_ILAT_TUI        0x00000020          /*Transmit Under-Run Indication */
#define SPI_ILAT_TCI        0x00000040          /*Transmit Collision Indication */
#define SPI_ILAT_MFI        0x00000080          /*Mode Fault Indication */
#define SPI_ILAT_RSI        0x00000100          /*Receive Start Indication */
#define SPI_ILAT_TSI        0x00000200          /*Transmit Start Indication */
#define SPI_ILAT_RFI        0x00000400          /*Receive Finish Indication */
#define SPI_ILAT_TFI        0x00000800          /*Transmit Finish Indication */
/* SPI_ILATCL */
#define SPI_ILAT_CLR_RUWMI  0x00000002          /*Receive Water Mark Interrupt */
#define SPI_ILAT_CLR_TUWMI  0x00000004          /*Transmit Water Mark Interrupt */
#define SPI_ILAT_CLR_ROI    0x00000010          /*Receive Over-Run Indication */
#define SPI_ILAT_CLR_TUI    0x00000020          /*Transmit Under-Run Indication */
#define SPI_ILAT_CLR_TCI    0x00000040          /*Transmit Collision Indication */
#define SPI_ILAT_CLR_MFI    0x00000080          /*Mode Fault Indication */
#define SPI_ILAT_CLR_RSI    0x00000100          /*Receive Start Indication */
#define SPI_ILAT_CLR_TSI    0x00000200          /*Transmit Start Indication */
#define SPI_ILAT_CLR_RFI    0x00000400          /*Receive Finish Indication */
#define SPI_ILAT_CLR_TFI    0x00000800          /*Transmit Finish Indication */
/* SPI_MMRDH */
#define SPI_MMRDH_MERGE     0x04000000          /*Merge Enable */
#define SPI_MMRDH_DMYSIZE   0x00007000          /*Bytes of Dummy */
#define SPI_MMRDH_ADRPINS   0x00000800          /*Pins used for Address */
#define SPI_MMRDH_ADRSIZE   0x00000700          /*Bytes of Read Address */
#define SPI_MMRDH_OPCODE    0x000000FF          /*Read Opcode */

#define SPI_MMRDH_TRIDMY_OFF  24                /*Bytes of Dummy offset */
#define SPI_MMRDH_DMYSIZE_OFF 12                /*Bytes of Dummy offset */
#define SPI_MMRDH_ADRSIZE_OFF 8                 /*Bytes of Read Addres offset */

#define BIT_SSEL_VAL(x) ((1 << 8) << (x + 1))   /* Slave Select input value bit */
#define BIT_SSEL_EN(x)  (1 << (x + 1))          /* Slave Select enable bit*/

// DDE

/* DDE_CFG */
#define DDE_CFG_EN          0x00000001  /* DMA channel enable */
#define DDE_CFG_WNR         0x00000002  /* Write/!read of memory */
#define DDE_CFG_SYNC        0x00000004  /* Synchronyze work units */
#define DDE_CFG_CADDR       0x00000008  /* Use current address */
#define DDE_CFG_PSIZE       0x00000070  /* Peripheral word size */
#define DDE_CFG_MSIZE       0x00000700  /* Memory transfer size */
#define DDE_CFG_FLOW        0x00007000  /* Flow control (next operation) */
#define DDE_CFG_TWAIT       0x00008000  /* Wait for trigger */
#define DDE_CFG_NDSIZE      0x00070000  /* Next descriptot set size */
#define DDE_CFG_INT         0x00300000  /* Generate interrupt */
#define DDE_CFG_TRIG        0x00C00000  /* Generate trigger */
#define DDE_CFG_TOVEN       0x01000000  /* Trigger overrun error enable */
#define DDE_CFG_DESCIDCPY   0x02000000  /* Descriptor ID copy control */
#define DDE_CFG_TWOD        0x04000000  /* Two dimension addressing enable */
#define DDE_CFG_PDRF        0x10000000  /* Peripheral data request forward */

#define DDE_CFG_MSIZE_OFFSET  8         /* MSIZE bit offset */
#define DDE_CFG_PSIZE_OFFSET  4         /* PSIZE bit offset */

/* DDE STAT */
#define DDE_STAT_IRQDONE    0x00000001  /* Work unit done  */
#define DDE_STAT_IRQERR     0x00000002  /* Error interrupt  */
#define DDE_STAT_PIRQ       0x00000004  /* Peripheral interrupt request */
#define DDE_STAT_ERRC       0x00000070  /* Error cause */
#define DDE_STAT_RUN        0x00000700  /* Operational state of DDE */
#define DDE_STAT_RUN_STOPPED 0x00000000 /* Idle/stop state */
#define DDE_STAT_PBWID      0x00003000  /* Peripheral bus width */
#define DDE_STAT_MBWID      0x0000C000  /* Memory bus width */
#define DDE_STAT_FIFOFILL   0x00070000  /* FIFO fill status */
#define DDE_STAT_TWAIT      0x00100000  /* Trigger wait status */

#define DDE_STAT_RUN_OFFSET   8         /* STAT run bits offset */

struct adi_spi_regs {
	u32 revid;
	u32 control;
	u32 rx_control;
	u32 tx_control;
	u32 clock;
	u32 delay;
	u32 ssel;
	u32 rwc;
	u32 rwcr;
	u32 twc;
	u32 twcr;
	u32 reserved0;
	u32 emask;
	u32 emaskcl;
	u32 emaskst;
	u32 reserved1;
	u32 status;
	u32 elat;
	u32 elatcl;
	u32 reserved2;
	u32 rfifo;
	u32 reserved3;
	u32 tfifo;
	u32 reserved4;
	u32 mmrdh;
	u32 mmtop;
};

struct adi_spi_dde_regs {
	u32 descp_next;
	u32 addr_start;
	u32 config;
	u32 x_count;
	u32 x_modify;
	u32 y_count;
	u32 y_modify;
	u32 reserved1;
	u32 reserved2;
	u32 descp_curr;
	u32 descp_prev;
	u32 addr_curr;
	u32 status;
	u32 x_count_curr;
	u32 y_count_curr;
	u32 reserved3;
	u32 bwl_count;
	u32 bwl_count_cur;
	u32 bwm_count;
	u32 bwm_count_cur;
};

struct adi_spi_platdata {
	u32 max_hz;
	u32 bus_num;
	int cs_num;
	struct adi_spi_regs *regs;
	struct adi_spi_dde_regs *tx_dde_regs;
	struct adi_spi_dde_regs *rx_dde_regs;
};

struct adi_spi_priv {
	u32 control;
	u32 clock;
	u32 bus_num;
	struct adi_spi_regs *regs;
	struct adi_spi_dde_regs *tx_dde_regs;
	struct adi_spi_dde_regs *rx_dde_regs;
	unsigned short *pins;
	int cs_pol;
	int cs_num;
	void *memory_map;
	bool use_dma;
};

#ifdef NEVER
extern int adi_spi_cs_valid(unsigned int bus, unsigned int cs);
extern struct adi_spi_slave *adi_spi_setup(unsigned int bus, unsigned int cs);
#endif

extern void adi_spi_setup_reg(struct adi_spi_platdata *platdata, int bus);
extern void adi_spi_setup_cs(struct adi_spi_priv *, int bus, unsigned int cs);

#endif /* _SPI_CHANNEL_H_ */
