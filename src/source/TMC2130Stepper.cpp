#include "TMCStepper.h"
#include "TMC_MACROS.h"

uint32_t TMC2130Stepper::spi_speed = 16000000/8;

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS) :
  TMCStepper(RS),
  _pinCS(pinCS)
  {}

TMC2130Stepper::TMC2130Stepper(uint16_t pinCS, float RS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK) :
  TMCStepper(RS),
  _pinCS(pinCS)
  {
    SW_SPIClass *SW_SPI_Obj = new SW_SPIClass(pinMOSI, pinMISO, pinSCK);
    TMC_SW_SPI = SW_SPI_Obj;
  }

void TMC2130Stepper::setSPISpeed(uint32_t speed) {
  spi_speed = speed;
}

void TMC2130Stepper::switchCSpin(bool state) {
  // Allows for overriding in child class to make use of fast io
  digitalWrite(_pinCS, state);
}

uint32_t TMC2130Stepper::read(uint8_t addressByte) {
  uint32_t out = 0UL;
  if (TMC_SW_SPI != NULL) {
    switchCSpin(LOW);
    TMC_SW_SPI->transfer(addressByte & 0xFF);
    TMC_SW_SPI->transfer16(0x0000); // Clear SPI
    TMC_SW_SPI->transfer16(0x0000);

    switchCSpin(HIGH);
    switchCSpin(LOW);

    status_response = TMC_SW_SPI->transfer(addressByte & 0xFF); // Send the address byte again
    out  = TMC_SW_SPI->transfer(0x00);
    out <<= 8;
    out |= TMC_SW_SPI->transfer(0x00);
    out <<= 8;
    out |= TMC_SW_SPI->transfer(0x00);
    out <<= 8;
    out |= TMC_SW_SPI->transfer(0x00);

  } else {
    SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
    switchCSpin(LOW);
    SPI.transfer(addressByte & 0xFF);
    SPI.transfer16(0x0000); // Clear SPI
    SPI.transfer16(0x0000);

    switchCSpin(HIGH);
    switchCSpin(LOW);

    status_response = SPI.transfer(addressByte & 0xFF); // Send the address byte again
    out  = SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);
    out <<= 8;
    out |= SPI.transfer(0x00);

    SPI.endTransaction();
  }
  switchCSpin(HIGH);
  return out;
}

void TMC2130Stepper::write(uint8_t addressByte, uint32_t config) {
  addressByte |= TMC_WRITE;
  if (TMC_SW_SPI != NULL) {
    switchCSpin(LOW);
    status_response = TMC_SW_SPI->transfer(addressByte & 0xFF);
    TMC_SW_SPI->transfer16((config>>16) & 0xFFFF);
    TMC_SW_SPI->transfer16(config & 0xFFFF);
  } else {
    SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));
    switchCSpin(LOW);
    status_response = SPI.transfer(addressByte & 0xFF);
    SPI.transfer16((config>>16) & 0xFFFF);
    SPI.transfer16(config & 0xFFFF);
    SPI.endTransaction();
  }
  switchCSpin(HIGH);
}

void TMC2130Stepper::begin() {
  //set pins
  pinMode(_pinCS, OUTPUT);
  switchCSpin(HIGH);

  if (TMC_SW_SPI != NULL) TMC_SW_SPI->init();

  GCONF(GCONF_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  PWMCONF(PWMCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);

  toff(8); //off_time(8);
  tbl(1); //blank_time(24);
}

/**
 *  Helper functions
 */

bool TMC2130Stepper::isEnabled() { return !drv_enn_cfg6() && toff(); }

void TMC2130Stepper::push() {
  GCONF(GCONF_register.sr);
  IHOLD_IRUN(IHOLD_IRUN_register.sr);
  TPOWERDOWN(TPOWERDOWN_register.sr);
  TPWMTHRS(TPWMTHRS_register.sr);
  TCOOLTHRS(TCOOLTHRS_register.sr);
  THIGH(THIGH_register.sr);
  XDIRECT(XDIRECT_register.sr);
  VDCMIN(VDCMIN_register.sr);
  CHOPCONF(CHOPCONF_register.sr);
  COOLCONF(COOLCONF_register.sr);
  PWMCONF(PWMCONF_register.sr);
  ENCM_CTRL(ENCM_CTRL_register.sr);
}

///////////////////////////////////////////////////////////////////////////////////////
// R: IOIN
uint32_t  TMC2130Stepper::IOIN()    { return read(IOIN_t::address); }
bool TMC2130Stepper::step()         { IOIN_t r{0}; r.sr = IOIN(); return r.step; }
bool TMC2130Stepper::dir()          { IOIN_t r{0}; r.sr = IOIN(); return r.dir; }
bool TMC2130Stepper::dcen_cfg4()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcen_cfg4; }
bool TMC2130Stepper::dcin_cfg5()    { IOIN_t r{0}; r.sr = IOIN(); return r.dcin_cfg5; }
bool TMC2130Stepper::drv_enn_cfg6() { IOIN_t r{0}; r.sr = IOIN(); return r.drv_enn_cfg6; }
bool TMC2130Stepper::dco()          { IOIN_t r{0}; r.sr = IOIN(); return r.dco; }
uint8_t TMC2130Stepper::version()   { IOIN_t r{0}; r.sr = IOIN(); return r.version; }
///////////////////////////////////////////////////////////////////////////////////////
// W: TCOOLTHRS
uint32_t TMC2130Stepper::TCOOLTHRS() { return TCOOLTHRS_register.sr; }
void TMC2130Stepper::TCOOLTHRS(uint32_t input) {
  TCOOLTHRS_register.sr = input;
  write(TCOOLTHRS_register.address, TCOOLTHRS_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// W: THIGH
uint32_t TMC2130Stepper::THIGH() { return THIGH_register.sr; }
void TMC2130Stepper::THIGH(uint32_t input) {
  THIGH_register.sr = input;
  write(THIGH_register.address, THIGH_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// RW: XDIRECT
uint32_t TMC2130Stepper::XDIRECT() {
  XDIRECT_register.sr = read(XDIRECT_register.address);
  return XDIRECT_register.sr;
}
void TMC2130Stepper::XDIRECT(uint32_t input) {
  XDIRECT_register.sr = input;
  write(XDIRECT_register.address, XDIRECT_register.sr);
}
void TMC2130Stepper::coil_A(int16_t B)  { XDIRECT_register.coil_A = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
void TMC2130Stepper::coil_B(int16_t B)  { XDIRECT_register.coil_B = B; write(XDIRECT_register.address, XDIRECT_register.sr); }
int16_t TMC2130Stepper::coil_A()        { XDIRECT(); return XDIRECT_register.coil_A; }
int16_t TMC2130Stepper::coil_B()        { XDIRECT(); return XDIRECT_register.coil_B; }
///////////////////////////////////////////////////////////////////////////////////////
// W: VDCMIN
uint32_t TMC2130Stepper::VDCMIN() { return VDCMIN_register.sr; }
void TMC2130Stepper::VDCMIN(uint32_t input) {
  VDCMIN_register.sr = input;
  write(VDCMIN_register.address, VDCMIN_register.sr);
}
///////////////////////////////////////////////////////////////////////////////////////
// R: PWM_SCALE
uint8_t TMC2130Stepper::PWM_SCALE() { return read(PWM_SCALE_t::address); }
///////////////////////////////////////////////////////////////////////////////////////
// W: ENCM_CTRL
uint8_t TMC2130Stepper::ENCM_CTRL() { return ENCM_CTRL_register.sr; }
void TMC2130Stepper::ENCM_CTRL(uint8_t input) {
  ENCM_CTRL_register.sr = input;
  write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr);
}
void TMC2130Stepper::inv(bool B)      { ENCM_CTRL_register.inv = B;       write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
void TMC2130Stepper::maxspeed(bool B) { ENCM_CTRL_register.maxspeed  = B; write(ENCM_CTRL_register.address, ENCM_CTRL_register.sr); }
bool TMC2130Stepper::inv()            { return ENCM_CTRL_register.inv; }
bool TMC2130Stepper::maxspeed()       { return ENCM_CTRL_register.maxspeed; }
///////////////////////////////////////////////////////////////////////////////////////
// R: LOST_STEPS
uint32_t TMC2130Stepper::LOST_STEPS() { return read(LOST_STEPS_t::address); }

void TMC2130Stepper::sg_current_decrease(uint8_t value) {
  switch(value) {
    case 32: sedn(0b00); break;
    case  8: sedn(0b01); break;
    case  2: sedn(0b10); break;
    case  1: sedn(0b11); break;
  }
}
uint8_t TMC2130Stepper::sg_current_decrease() {
  switch(sedn()) {
    case 0b00: return 32;
    case 0b01: return  8;
    case 0b10: return  2;
    case 0b11: return  1;
  }
  return 0;
}


#define TMC2130_WAVE_FAC200_MIN 180
#define TMC2130_WAVE_FAC200_MAX 250
#define TMC2130_WAVE_FAC200_STP 1

#define TMC2130_REG_MSLUT0 0x60
#define TMC2130_REG_MSLUTSEL 0x68
#define TMC2130_REG_MSLUTSTART 0x69



uint32_t TMC2130Stepper::tmc2130_wr_MSLUTSTART(uint8_t start_sin, uint8_t start_sin90)
{
	uint32_t val = 0;
	val |= (uint32_t)start_sin;
	val |= ((uint32_t)start_sin90) << 16;
	//tmc2130_wr(axis, TMC2130_REG_MSLUTSTART, val);
  	write(TMC2130_REG_MSLUTSTART, val);
  	return val;
	//printf_P(PSTR("MSLUTSTART=%08lx (start_sin=%d start_sin90=%d)\n"), val, start_sin, start_sin90);
}

uint32_t TMC2130Stepper::tmc2130_wr_MSLUTSEL(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t w0, uint8_t w1, uint8_t w2, uint8_t w3)
{
	uint32_t val = 0;
	val |= ((uint32_t)w0);
	val |= ((uint32_t)w1) << 2;
	val |= ((uint32_t)w2) << 4;
	val |= ((uint32_t)w3) << 6;
	val |= ((uint32_t)x1) << 8;
	val |= ((uint32_t)x2) << 16;
	val |= ((uint32_t)x3) << 24;
	write(TMC2130_REG_MSLUTSEL, val);
  	return val;
	//printf_P(PSTR("MSLUTSEL=%08lx (x1=%d x2=%d x3=%d w0=%d w1=%d w2=%d w3=%d)\n"), val, x1, x2, x3, w0, w1, w2, w3);
}

uint32_t TMC2130Stepper::tmc2130_wr_MSLUT(uint8_t i, uint32_t val)
{
	write(TMC2130_REG_MSLUT0 + (i & 7), val);
  	return val;
	//printf_P(PSTR("MSLUT[%d]=%08lx\n"), i, val);
}

void TMC2130Stepper::tmc2130_set_wave(uint8_t amp, uint8_t fac200)
// TMC2130 wave compression algorithm
// amp defaults to 247!, fac200 defaults to like 0?
// optimized for minimal memory requirements
	//printf_P(PSTR("tmc2130_set_wave %d %d\n"), axis, fac200);
	if (fac200 < TMC2130_WAVE_FAC200_MIN) fac200 = 0;
	if (fac200 > TMC2130_WAVE_FAC200_MAX) fac200 = TMC2130_WAVE_FAC200_MAX;
	float fac = (float)fac200/200; //correction factor
	uint8_t vA = 0;                //value of currentA
	uint8_t va = 0;                //previous vA
	uint8_t d0 = 0;                //delta0
	uint8_t d1 = 1;                //delta1
	uint8_t w[4] = {1,1,1,1};      //W bits (MSLUTSEL)
	uint8_t x[3] = {255,255,255};  //X segment bounds (MSLUTSEL)
	uint8_t s = 0;                 //current segment
	int8_t b;                      //encoded bit value
	uint8_t dA;                    //delta value
	int i;                         //microstep index
	uint32_t reg;                  //tmc2130 register
	tmc2130_wr_MSLUTSTART(0, amp);
	for (i = 0; i < 256; i++)
	{
		if ((i & 31) == 0)
			reg = 0;
		// calculate value
		if (fac == 0) // default TMC wave
			vA = (uint8_t)((amp+1) * sin((2*PI*i + PI)/1024) + 0.5) - 1;
		else // corrected wave
			vA = (uint8_t)(amp * pow(sin(2*PI*i/1024), fac) + 0.5);
		dA = vA - va; // calculate delta
		va = vA;
		b = -1;
		if (dA == d0) b = 0;      //delta == delta0 => bit=0
		else if (dA == d1) b = 1; //delta == delta1 => bit=1
		else
		{
			if (dA < d0) // delta < delta0 => switch wbit down
			{
				//printf("dn\n");
				b = 0;
				switch (dA)
				{
				case -1: d0 = -1; d1 = 0; w[s+1] = 0; break;
				case  0: d0 =  0; d1 = 1; w[s+1] = 1; break;
				case  1: d0 =  1; d1 = 2; w[s+1] = 2; break;
				default: b = -1; break;
				}
				if (b >= 0) { x[s] = i; s++; }
			}
			else if (dA > d1) // delta > delta0 => switch wbit up
			{
				//printf("up\n");
				b = 1;
				switch (dA)
				{
				case  1: d0 =  0; d1 = 1; w[s+1] = 1; break;
				case  2: d0 =  1; d1 = 2; w[s+1] = 2; break;
				case  3: d0 =  2; d1 = 3; w[s+1] = 3; break;
				default: b = -1; break;
				}
			    if (b >= 0) { x[s] = i; s++; }
			}
		}
		if (b < 0) break; // delta out of range (<-1 or >3)
		if (s > 3) break; // segment out of range (> 3)
		//printf("%d\n", vA);
		if (b == 1) reg |= 0x80000000;
		if ((i & 31) == 31)
			tmc2130_wr_MSLUT((uint8_t)(i >> 5), reg);
		else
			reg >>= 1;
//		printf("%3d\t%3d\t%2d\t%2d\t%2d\t%2d    %08x\n", i, vA, dA, b, w[s], s, reg);
	}
	tmc2130_wr_MSLUTSEL(x[0], x[1], x[2], w[0], w[1], w[2], w[3]);

/*
//	printf_P(PSTR(" tmc2130_set_wave %d %d\n"), axis, fac200);
	switch (fac200)
	{
	case 0: //default TMC wave 247/0
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0xaaaab556);
		tmc2130_wr_MSLUT(axis, 1, 0x4a9554aa);
		tmc2130_wr_MSLUT(axis, 2, 0x24492929);
		tmc2130_wr_MSLUT(axis, 3, 0x10104222);
		tmc2130_wr_MSLUT(axis, 4, 0xf8000000);
		tmc2130_wr_MSLUT(axis, 5, 0xb5bb777d);
		tmc2130_wr_MSLUT(axis, 6, 0x49295556);
		tmc2130_wr_MSLUT(axis, 7, 0x00404222);
		tmc2130_wr_MSLUTSEL(axis, 2, 154, 255, 1, 2, 1, 1);
		break;
	case 210: //calculated wave 247/1.050
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x55294a4e);
		tmc2130_wr_MSLUT(axis, 1, 0xa52a552a);
		tmc2130_wr_MSLUT(axis, 2, 0x48949294);
		tmc2130_wr_MSLUT(axis, 3, 0x81042222);
		tmc2130_wr_MSLUT(axis, 4, 0x00000000);
		tmc2130_wr_MSLUT(axis, 5, 0xdb6eef7e);
		tmc2130_wr_MSLUT(axis, 6, 0x9295555a);
		tmc2130_wr_MSLUT(axis, 7, 0x00408444);
		tmc2130_wr_MSLUTSEL(axis, 3, 160, 255, 1, 2, 1, 1);
		break;
	case 212: //calculated wave 247/1.060
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x4a94948e);
		tmc2130_wr_MSLUT(axis, 1, 0x94a952a5);
		tmc2130_wr_MSLUT(axis, 2, 0x24925252);
		tmc2130_wr_MSLUT(axis, 3, 0x10421112);
		tmc2130_wr_MSLUT(axis, 4, 0xc0000020);
		tmc2130_wr_MSLUT(axis, 5, 0xdb7777df);
		tmc2130_wr_MSLUT(axis, 6, 0x9295556a);
		tmc2130_wr_MSLUT(axis, 7, 0x00408444);
		tmc2130_wr_MSLUTSEL(axis, 3, 157, 255, 1, 2, 1, 1);
		break;
	case 214: //calculated wave 247/1.070
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0xa949489e);
		tmc2130_wr_MSLUT(axis, 1, 0x52a54a54);
		tmc2130_wr_MSLUT(axis, 2, 0x224a494a);
		tmc2130_wr_MSLUT(axis, 3, 0x04108889);
		tmc2130_wr_MSLUT(axis, 4, 0xffc08002);
		tmc2130_wr_MSLUT(axis, 5, 0x6dbbbdfb);
		tmc2130_wr_MSLUT(axis, 6, 0x94a555ab);
		tmc2130_wr_MSLUT(axis, 7, 0x00408444);
		tmc2130_wr_MSLUTSEL(axis, 4, 149, 255, 1, 2, 1, 1);
		break;
	case 215: //calculated wave 247/1.075
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x4a52491e);
		tmc2130_wr_MSLUT(axis, 1, 0xa54a54a9);
		tmc2130_wr_MSLUT(axis, 2, 0x49249494);
		tmc2130_wr_MSLUT(axis, 3, 0x10421122);
		tmc2130_wr_MSLUT(axis, 4, 0x00000008);
		tmc2130_wr_MSLUT(axis, 5, 0x6ddbdefc);
		tmc2130_wr_MSLUT(axis, 6, 0x94a555ad);
		tmc2130_wr_MSLUT(axis, 7, 0x00408444);
		tmc2130_wr_MSLUTSEL(axis, 4, 161, 255, 1, 2, 1, 1);
		break;
	case 216: //calculated wave 247/1.080
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x9494911e);
		tmc2130_wr_MSLUT(axis, 1, 0x4a94a94a);
		tmc2130_wr_MSLUT(axis, 2, 0x92492929);
		tmc2130_wr_MSLUT(axis, 3, 0x41044444);
		tmc2130_wr_MSLUT(axis, 4, 0x00000040);
		tmc2130_wr_MSLUT(axis, 5, 0xaedddf7f);
		tmc2130_wr_MSLUT(axis, 6, 0x94a956ad);
		tmc2130_wr_MSLUT(axis, 7, 0x00808448);
		tmc2130_wr_MSLUTSEL(axis, 4, 159, 255, 1, 2, 1, 1);
		break;
	case 218: //calculated wave 247/1.090
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x4a49223e);
		tmc2130_wr_MSLUT(axis, 1, 0x4a52a529);
		tmc2130_wr_MSLUT(axis, 2, 0x49252529);
		tmc2130_wr_MSLUT(axis, 3, 0x08422224);
		tmc2130_wr_MSLUT(axis, 4, 0xfc008004);
		tmc2130_wr_MSLUT(axis, 5, 0xb6eef7df);
		tmc2130_wr_MSLUT(axis, 6, 0xa4aaaab5);
		tmc2130_wr_MSLUT(axis, 7, 0x00808448);
		tmc2130_wr_MSLUTSEL(axis, 5, 153, 255, 1, 2, 1, 1);
		break;
	case 220: //calculated wave 247/1.100
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0xa492487e);
		tmc2130_wr_MSLUT(axis, 1, 0x294a52a4);
		tmc2130_wr_MSLUT(axis, 2, 0x492494a5);
		tmc2130_wr_MSLUT(axis, 3, 0x82110912);
		tmc2130_wr_MSLUT(axis, 4, 0x00000080);
		tmc2130_wr_MSLUT(axis, 5, 0xdb777df8);
		tmc2130_wr_MSLUT(axis, 6, 0x252aaad6);
		tmc2130_wr_MSLUT(axis, 7, 0x00808449);
		tmc2130_wr_MSLUTSEL(axis, 6, 162, 255, 1, 2, 1, 1);
		break;
	case 222: //calculated wave 247/1.110
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x524910fe);
		tmc2130_wr_MSLUT(axis, 1, 0xa5294a52);
		tmc2130_wr_MSLUT(axis, 2, 0x24929294);
		tmc2130_wr_MSLUT(axis, 3, 0x20844489);
		tmc2130_wr_MSLUT(axis, 4, 0xc0004008);
		tmc2130_wr_MSLUT(axis, 5, 0xdbbbdf7f);
		tmc2130_wr_MSLUT(axis, 6, 0x252aab5a);
		tmc2130_wr_MSLUT(axis, 7, 0x00808449);
		tmc2130_wr_MSLUTSEL(axis, 7, 157, 255, 1, 2, 1, 1);
		break;
	case 224: //calculated wave 247/1.120
		tmc2130_wr_MSLUTSTART(axis, 0, 247);
		tmc2130_wr_MSLUT(axis, 0, 0x292223fe);
		tmc2130_wr_MSLUT(axis, 1, 0x94a52949);
		tmc2130_wr_MSLUT(axis, 2, 0x92524a52);
		tmc2130_wr_MSLUT(axis, 3, 0x04222244);
		tmc2130_wr_MSLUT(axis, 4, 0x00000101);
		tmc2130_wr_MSLUT(axis, 5, 0x6dddefe0);
		tmc2130_wr_MSLUT(axis, 6, 0x254aad5b);
		tmc2130_wr_MSLUT(axis, 7, 0x00810889);
		tmc2130_wr_MSLUTSEL(axis, 9, 164, 255, 1, 2, 1, 1);
		break;
	}*/
}


