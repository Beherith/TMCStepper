#pragma once

// Register addresses
cfg.REG_DRVCTRL					= 0b000;
cfg.REG_CHOPCONF				= 0b100;
cfg.REG_SMARTEN					= 0b101;
cfg.REG_SGCSCONF				= 0b110;
cfg.REG_DRVCONF					= 0b111;
// DRVCTRL (SPI)
cfg.PHA_bp							= 17;
cfg.CA_bp								= 9;
cfg.PHB_bp							= 8;
cfg.CB_bp								= 0;
cfg.PHA_bm							= 0x20000UL;
cfg.CA_bm								= 0x1FE00UL;
cfg.PHB_bm							= 0x100UL;
cfg.CB_bm								= 0xFFUL;
// DRVCTRL (STEP/DIR)
cfg.INTPOL_bp 					= 9;
cfg.DEDGE_bp 						= 8;
cfg.MRES_bp 						= 0;
cfg.INTPOL_bm 					= 0x200UL;
cfg.DEDGE_bm 						= 0x100UL;
cfg.MRES_bm 						= 0xFUL;
// CHOPCONF
cfg.TBL_bp 							= 15;
cfg.CHM_bp							= 14;
cfg.RNDTF_bp 						= 13;
cfg.HDEC_bp 						= 11;
cfg.HEND_bp 						= 7;
cfg.HSTRT_bp 						= 4;
cfg.TOFF_bp 						= 0;
cfg.TBL_bm 							= 0x18000UL;
cfg.CHM_bm							= 0x4000UL;
cfg.RNDTF_bm 						= 0x2000UL;
cfg.HDEC_bm 						= 0x1800UL;
cfg.HEND_bm 						= 0x780UL;
cfg.HSTRT_bm 						= 0x70UL;
cfg.TOFF_bm 						= 0xFUL;
// SMARTEN
cfg.SEIMIN_bp 					= 15;
cfg.SEDN_bp 						= 13;
cfg.SEMAX_bp 						= 8;
cfg.SEUP_bp 						= 5;
cfg.SEMIN_bp 						= 0;
cfg.SEIMIN_bm 					= 0x8000UL;
cfg.SEDN_bm 						= 0x6000UL;
cfg.SEMAX_bm 						= 0xF00UL;
cfg.SEUP_bm 						= 0x60UL;
cfg.SEMIN_bm 						= 0xFUL;
// SGCSCONF
cfg.SFILT_bp 						= 16;
cfg.SGT_bp 							= 8;
cfg.CS_bp 							= 0;
cfg.SFILT_bm 						= 0x10000UL;
cfg.SGT_bm 							= 0x7F00UL;
cfg.CS_bm 							= 0x1FUL;
// DRVCONF
cfg.TST_bp 							= 16;
cfg.SLPH_bp 						= 14;
cfg.SLPL_bp 						= 12;
cfg.DISS2G_bp 					= 10;
cfg.TS2G_bp							= 8;
cfg.SDOFF_bp						= 7;
cfg.VSENSE_bp 					= 6;
cfg.RDSEL_bp						= 4;
cfg.TST_bm 							= 0x10000UL;
cfg.SLPH_bm 						= 0xC000UL;
cfg.SLPL_bm 						= 0x3000UL;
cfg.DISS2G_bm 					= 0x400UL;
cfg.TS2G_bm							= 0x300UL;
cfg.SDOFF_bm						= 0x80UL;
cfg.VSENSE_bm 					= 0x40UL;
cfg.RDSEL_bm						= 0x30UL;
// DRVSTATUS
cfg.MSTEP_bp						= 10;
cfg.SG_RESULT_10b_bp		= 10;
cfg.SG_RESULT_5b_bp			= 15;
cfg.SE_bp 							= 10;
cfg.STST_bp 						= 7;
cfg.OLB_bp 							= 6;
cfg.OLA_bp 							= 5;
cfg.S2GB_bp 						= 4;
cfg.S2GA_bp 						= 3;
cfg.OTPW_bp 						= 2;
cfg.OT_bp 							= 1;
cfg.SG_bp 							= 0;
cfg.MSTEP_bm						= 0xFFC00UL;
cfg.SG_RESULT_10b_bm		= 0xFFC00UL;
cfg.SG_RESULT_5b_bm			= 0xF8000UL;
cfg.SE_bm 							= 0x7C00UL;
cfg.STST_bm 						= 0x80UL;
cfg.OLB_bm 							= 0x40UL;
cfg.OLA_bm 							= 0x20UL;
cfg.S2GB_bm 						= 0x10UL;
cfg.S2GA_bm 						= 0x8UL;
cfg.OTPW_bm 						= 0x4UL;
cfg.OT_bm 							= 0x2UL;
cfg.SG_bm 							= 0x1UL;