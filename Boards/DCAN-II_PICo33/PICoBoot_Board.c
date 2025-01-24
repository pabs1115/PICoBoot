/*
    This file is part of PICoBoot.

    Copyright (C) 2021 ReimuNotMoe <reimu@sudomaker.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "PICoBoot_Board.h"
#include <PICoBoot/PICoBoot.h>

/*
// CONFIG3
#pragma config WPFP = WPFP511    //Write Protection Flash Page Segment Boundary->Highest Page (same as page 170)
#pragma config WPDIS = WPDIS    //Segment Write Protection Disable bit->Segmented code protection disabled
#pragma config WPCFG = WPCFGDIS    //Configuration Word Code Page Protection Select bit->Last page(at the top of program memory) and Flash configuration words are not protected
#pragma config WPEND = WPENDMEM    //Segment Write Protection End Page Select bit->Write Protect from WPFP to the last page of memory

// CONFIG2
#pragma config POSCMOD = XT		// Primary Oscillator Select->XT oscillator mode selected
#pragma config DISUVREG = OFF		// Internal USB 3.3V Regulator Disable bit->Regulator is disabled
#pragma config IOL1WAY = OFF		// IOLOCK One-Way Set Enable bit->Write RP Registers Once
#pragma config OSCIOFNC = ON		// Primary Oscillator Output Function->OSCO functions as port I/O (RC15)
#pragma config FCKSM = CSECMD		// Clock Switching and Monitor->Clock switching is enabled, Fail-safe Clock Monitor is disabled
#pragma config FNOSC = FRC		// Oscillator Select->FRC
#pragma config PLL_96MHZ = ON		// 96MHz PLL Disable->Enabled
#pragma config PLLDIV = DIV2		// USB 96 MHz PLL Prescaler Select bits->Oscillator input divided by 2 (8MHz input)
#pragma config IESO = OFF		// Internal External Switch Over Mode->IESO mode (Two-speed start-up)disabled

// CONFIG1
#pragma config WDTPS = PS32768		// Watchdog Timer Postscaler->1:32768
#pragma config FWPSA = PR128		// WDT Prescaler->Prescaler ratio of 1:128
#pragma config WINDIS = OFF		// Watchdog Timer Window->Standard Watchdog Timer enabled,(Windowed-mode is disabled)
#pragma config FWDTEN = OFF		// Watchdog Timer Enable->Watchdog Timer is disabled
#pragma config ICS = PGx2		// Comm Channel Select->Emulator functions are shared with PGEC1/PGED1
#pragma config BKBUG = OFF		// Background Debug->Device resets into Operational mode
#pragma config GWRP = OFF		// General Code Segment Write Protect->Writes to program memory are allowed
#pragma config GCP = ON			// General Code Segment Code Protect->Code protection is **enabled**
#pragma config JTAGEN = OFF		// JTAG Port Enable->JTAG port is disabled

*/
// FGS
#pragma config GWRP = OFF    //General Segment Write-Protect bit->General Segment may be written
#pragma config GSS = OFF    //General Segment Code-Protect bit->General Segment Code protect is disabled
#pragma config GSSK = OFF    //General Segment Key bits->General Segment Write Protection and Code Protection is Disabled

// FOSCSEL
#pragma config FNOSC = FRC    //Initial Oscillator Source Selection bits->FRC
#pragma config IESO = OFF    //Two-speed Oscillator Start-up Enable bit->Start up with user-selected oscillator source

// FOSC
#pragma config POSCMD = HS    //Primary Oscillator Mode Select bits->HS Crystal Oscillator Mode
#pragma config OSCIOFNC = ON    //OSC2 Pin Function bit->OSC2 is general purpose digital I/O pin
#pragma config IOL1WAY = ON    //Peripheral pin select configuration->Allow only one reconfiguration
#pragma config FCKSM = CSECMD    //Clock Switching Mode bits->Clock switching is enabled,Fail-safe Clock Monitor is disabled

// FWDT
#pragma config WDTPOST = PS32768    //Watchdog Timer Postscaler bits->1:32768
#pragma config WDTPRE = PR128    //Watchdog Timer Prescaler bit->1:128
#pragma config PLLKEN = ON    //PLL Lock Wait Enable bit->Clock switch to PLL source will wait until the PLL lock signal is valid.
#pragma config WINDIS = OFF    //Watchdog Timer Window Enable bit->Watchdog Timer in Non-Window mode
#pragma config FWDTEN = OFF    //Watchdog Timer Enable bit->Watchdog timer enabled/disabled by user software

// FPOR
#pragma config FPWRT = PWR128    //Power-on Reset Timer Value Select bits->128ms
#pragma config BOREN = ON    //Brown-out Reset (BOR) Detection Enable bit->BOR is enabled
#pragma config ALTI2C1 = ON    //Alternate I2C pins for I2C1->ASDA1/ASCK1 pins are selected as the I/O pins for I2C1

// FICD
#pragma config ICS = PGD1    //ICD Communication Channel Select bits->Communicate on PGEC1 and PGED1
#pragma config RSTPRI = PF    //Reset Target Vector Select bit->Device will obtain reset instruction from Primary flash
#pragma config JTAGEN = OFF    //JTAG Enable bit->JTAG is disabled

// FAS
#pragma config AWRP = OFF    //Auxiliary Segment Write-protect bit->Aux Flash may be written
#pragma config APL = OFF    //Auxiliary Segment Code-protect bit->Aux Flash Code protect is disabled
#pragma config APLK = OFF    //Auxiliary Segment Key bits->Aux Flash Write Protection and Code Protection is Disabled


const uint32_t XTAL_FREQ = 80000000UL;
const uint32_t FCY = 80000000UL / 2;

const char PICoBoot_BoardManufacturer[] = "SudoMaker";
const char PICoBoot_Board[] = "CartBoy RW v2";
const char PICoBoot_ChipManufacturer[] = "Microchip";
const char PICoBoot_Chip[] = "33EP256MU806";

#define StaticEnvironment_Address	0x0200

const uint32_t PICoBoot_Bootloader_Address = 0x0400;
const uint32_t PICoBoot_App_Address = 0x4000;
const uint32_t PICoBoot_App_Size = 0x26000;
const uint32_t PICoBoot_StaticEnvironment_Address = StaticEnvironment_Address;

const uint16_t PICoBoot_Flash_PageSize = 1024;

const uint32_t PICoBoot_FlashProhibitedRanges[] = {
        0x0400, 0x3fff,		// BL itself
        0x2a800, 0x2afff	// Config bits
};

volatile const uint8_t __attribute__((space(prog), address(StaticEnvironment_Address))) PICoBoot_StaticEnvironment_DefaultValues[32] = {
        PICoBoot_ENVAR_UNINIT, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, PICoBoot_ENVAR_UNINIT
};

void PICoBoot_Board_PinInitialize() {
    /****************************************************************************
     * Setting the Output Latch SFR(s)
     ***************************************************************************/
    LATB = 0x0004;
    LATC = 0x0000;
    LATD = 0x0000;
    LATE = 0x0000;
    LATF = 0x0000;
    LATG = 0x0000;

    /****************************************************************************
     * Setting the GPIO Direction SFR(s)
     ***************************************************************************/
    TRISB = 0xF9FB;
    TRISC = 0x7000;
    TRISD = 0x0FD9;
    TRISE = 0x00FF;
    TRISF = 0x003B;
    TRISG = 0x03C0;

    /****************************************************************************
     * Setting the Weak Pull Up and Weak Pull Down SFR(s)
     ***************************************************************************/
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPDD = 0x0000;
    CNPDE = 0x0000;
    CNPDF = 0x0000;
    CNPDG = 0x0000;
    CNPUB = 0x0000;
    CNPUC = 0x0000;
    CNPUD = 0x0000;
    CNPUE = 0x0000;
    CNPUF = 0x0000;
    CNPUG = 0x0000;

    /****************************************************************************
     * Setting the Open Drain SFR(s)
     ***************************************************************************/
    ODCD = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;

    /****************************************************************************
     * Setting the Analog/Digital Configuration SFR(s)
     ***************************************************************************/
    ANSELB = 0xF93B;
    ANSELC = 0x6000;
    ANSELD = 0x00C0;
    ANSELE = 0x00FF;
    ANSELG = 0x0340;


    /////////////////////////////////////////////////////////////////////////////////////////


}

void PICoBoot_Board_SystemInitialize() {
    // FRCDIV FRC/1; PLLPRE 4; DOZE 1:8; PLLPOST 1:2; DOZEN disabled; ROI disabled; 
    CLKDIV = 0x3002;
    // TUN Center frequency; 
    OSCTUN = 0x00;
    // ROON enabled; ROSEL FOSC; RODIV 3; ROSSLP disabled; 
    REFOCON = 0x8300;
    // PLLDIV 38; 
    PLLFBD = 0x26;
    // ENAPLL enabled; APLLPOST 1:2; FRCSEL ASRCSEL determines input clock source; SELACLK Auxiliary Oscillators; ASRCSEL Primary Oscillator; AOSCMD AUX; APLLPRE 1:4;  
    ACLKCON3 = 0xA4C3;
    // APLLDIV 24; 
    ACLKDIV3 = 0x07;
    // AD1MD enabled; PWMMD enabled; T3MD enabled; T4MD enabled; T1MD enabled; U2MD enabled; T2MD enabled; U1MD enabled; QEI1MD enabled; SPI2MD enabled; SPI1MD enabled; C2MD enabled; C1MD enabled; DCIMD enabled; T5MD enabled; I2C1MD enabled; 
    PMD1 = 0x00;
    // OC5MD enabled; OC6MD enabled; OC7MD enabled; OC8MD enabled; OC1MD enabled; IC2MD enabled; OC2MD enabled; IC1MD enabled; OC3MD enabled; OC4MD enabled; IC6MD enabled; IC7MD enabled; IC5MD enabled; IC8MD enabled; IC4MD enabled; IC3MD enabled; 
    PMD2 = 0x00;
    // AD2MD enabled; PMPMD enabled; CMPMD enabled; U3MD enabled; QEI2MD enabled; RTCCMD enabled; T9MD enabled; T8MD enabled; CRCMD enabled; T7MD enabled; I2C2MD enabled; DAC1MD enabled; T6MD enabled; 
    PMD3 = 0x00;
    // U4MD enabled; USB1MD enabled; REFOMD enabled; 
    PMD4 = 0x00;
    // OC9MD enabled; OC16MD enabled; IC10MD enabled; IC11MD enabled; IC12MD enabled; IC13MD enabled; IC14MD enabled; IC15MD enabled; IC16MD enabled; IC9MD enabled; OC14MD enabled; OC15MD enabled; OC12MD enabled; OC13MD enabled; OC10MD enabled; OC11MD enabled; 
    PMD5 = 0x00;
    // PWM2MD enabled; PWM1MD enabled; SPI4MD enabled; PWM4MD enabled; SPI3MD enabled; PWM3MD enabled; 
    PMD6 = 0x00;
    // DMA8MD enabled; DMA4MD enabled; DMA12MD enabled; DMA0MD enabled; 
    PMD7 = 0x00;
    // CF no clock failure; NOSC PRIPLL; LPOSCEN disabled; CLKLOCK unlocked; OSWEN Switch is Complete; IOLOCK not-active; 
    __builtin_write_OSCCONH((uint8_t) (0x03));
    __builtin_write_OSCCONL((uint8_t) (0x01));

    // Important: Make use of the DefaultValues so it will not be LTO'd
    for (size_t i=0; i<sizeof(PICoBoot_StaticEnvironment_DefaultValues); i++) {
        PICoBoot_LED_1 = PICoBoot_StaticEnvironment_DefaultValues[i];
        PICoBoot_LED_1 = 0;
    }

    // Wait for Clock switch to occur
    while (OSCCONbits.OSWEN != 0);
    while (OSCCONbits.LOCK != 1);
}

int PICoBoot_Board_Reset_Action() {
    Delay_Milliseconds(10);

    if (picoboot_runtime_env.reboot_target == RebootTarget_Bootloader) {
        picoboot_runtime_env.reboot_target = RebootTarget_Default;
        return ResetAction_StayInBL;
    }

    if (picoboot_static_env.watchdog_fails_to_enter_bl) {
        if (picoboot_runtime_env.watchdog_failed_count >= picoboot_static_env.watchdog_fails_to_enter_bl) {
            return ResetAction_StayInBL;
        }
    }

    if (PORTBbits.RB0 == 1) {
        return ResetAction_RunUserApp;
    } else {
        return ResetAction_StayInBL;
    }
}

uint32_t PICoBoot_Board_Flash_ReadInstruction(uint32_t addr) {
    uint32_t ret;

    uint16_t tblpag_save = TBLPAG;

    TBLPAG = (addr >> 16) & 0xff;		// Load TBLPAG register with read address <23:16>
    uint16_t addrOffset = addr & 0xffff;	// Load offset with read address <15:0>

    // Read data from program memory
    ret = __builtin_tblrdl(addrOffset);   // readDataL contains lower word data
    ret |= (uint32_t)__builtin_tblrdh(addrOffset) << 16;

    TBLPAG = tblpag_save;

    return ret;
}

void PICoBoot_Board_Flash_ErasePage(uint32_t addr) {
    NVMCON = 0x4042;			// Set WREN, ERASE, NVMOP<3:0> = 0010 (Memory page erase operation)

    uint16_t tblpag_save = TBLPAG;

    TBLPAG = (addr >> 16) & 0xff;		// Load TBLPAG register with read address <23:16>
    uint16_t addrOffset = addr & 0xffff;	// Load offset with read address <15:0>

    __builtin_tblwtl(addrOffset, 0);

    __builtin_disable_interrupts();
    __builtin_write_NVM();

    while (NVMCONbits.WR == 1);

    TBLPAG = tblpag_save;

    NVMCONbits.WREN = 0;
}

void PICoBoot_Board_Flash_WriteInstruction(uint32_t addr, uint32_t value) {
    NVMCON = 0x4003;			// Set WREN, NVMOP<3:0> = 0011 (Memory word program operation)

    uint16_t tblpag_save = TBLPAG;

    TBLPAG = (addr >> 16) & 0xff;		// Load TBLPAG register with read address <23:16>
    uint16_t addrOffset = addr & 0xffff;	// Load offset with read address <15:0>

    __builtin_tblwtl(addrOffset, value & 0xffff);
    __builtin_tblwth(addrOffset, (value >> 16) & 0xff);

    __builtin_disable_interrupts();
    __builtin_write_NVM();

    while (NVMCONbits.WR == 1);

    TBLPAG = tblpag_save;

    NVMCONbits.WREN = 0;
}

__attribute__((persistent, address(0x4600))) uint8_t __runtime_env[sizeof(PicoBootRuntimeEnvironment)];

void PICoBoot_RuntimeEnvironment_Load() {
    uint8_t *penv = (uint8_t *) &picoboot_runtime_env;

    for (size_t i=0; i<sizeof(PicoBootRuntimeEnvironment); i++) {
        penv[i] = __runtime_env[i];
    }
}

void PICoBoot_RuntimeEnvironment_Save() {
    uint8_t *penv = (uint8_t *) &picoboot_runtime_env;

    for (size_t i=0; i<sizeof(PicoBootRuntimeEnvironment); i++) {
        __runtime_env[i] = penv[i];
    }
}