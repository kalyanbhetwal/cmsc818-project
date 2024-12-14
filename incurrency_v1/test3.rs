#![feature(prelude_import)]
//! examples/spawn_loop.rs
#![no_main]
#![no_std]
#![allow(warnings)]
#[prelude_import]
use core::prelude::rust_2021::*;
#[macro_use]
extern crate core;
extern crate compiler_builtins as _;
use test_app as _;
use stm32f3xx_hal_v2::pac::Interrupt;
use cortex_m::peripheral::NVIC;
use cortex_m_semihosting::{debug, hprintln};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
mod checkpoint {
    #![allow(unsafe_code, non_upper_case_globals)]
    pub mod my_flash {
        #![allow(unsafe_code, unused, non_upper_case_globals)]
        #![no_main]
        #![no_std]
        use core::mem;
        use core::ptr;
        use cortex_m::asm::{nop, self};
        use cortex_m_rt::entry;
        use ::core::arch::asm;
        use cortex_m_semihosting::{debug, hprintln};
        use stm32f3xx_hal_v2::{
            self as hal, pac, prelude::*, flash::ACR, pac::Peripherals, pac::FLASH,
        };
        use volatile::Volatile;
        use stm32f3xx_hal_v2::hal::blocking::rng::Read;
        const UNLOCK_KEY1: u32 = 0x4567_0123;
        const UNLOCK_KEY2: u32 = 0xCDEF_89AB;
        pub fn unlock(flash: &mut FLASH) -> bool {
            if flash.cr.read().lock().bit_is_clear() {
                return true;
            }
            flash.keyr.write(|w| unsafe { w.bits(UNLOCK_KEY1) });
            flash.keyr.write(|w| unsafe { w.bits(UNLOCK_KEY2) });
            if flash.cr.read().lock().bit_is_clear() {
                return true;
            } else {
                return false;
            }
        }
        pub fn wait_ready(flash: &FLASH) {
            while flash.sr.read().bsy().bit() {}
        }
        pub fn clear_error_flags(flash: &FLASH) {
            if flash.sr.read().wrprterr().bit_is_set() {
                flash.sr.modify(|_, w| w.wrprterr().set_bit());
            }
            if flash.sr.read().pgerr().bit_is_set() {
                flash.sr.modify(|_, w| w.pgerr().set_bit());
            }
        }
        pub fn erase_page(flash: &mut FLASH, page: u32) {
            if flash.sr.read().bsy().bit_is_set() {
                ::cortex_m_semihosting::export::hstdout_str("Flash is busy.\n");
            }
            clear_error_flags(&flash);
            flash.cr.modify(|_, w| w.per().set_bit());
            flash.ar.write(|w| unsafe { w.bits(page as u32) });
            flash.cr.modify(|_, w| w.strt().set_bit());
            while flash.sr.read().bsy().bit_is_set() {}
            while flash.sr.read().bsy().bit_is_set() {}
            flash.cr.modify(|_, w| w.lock().set_bit());
        }
        pub fn write_to_flash(flash: &mut FLASH, addr: u32, data: u32) {
            unlock(flash);
            if flash.sr.read().bsy().bit_is_set() {
                ::cortex_m_semihosting::export::hstdout_str("Flash is busy.\n");
            }
            clear_error_flags(&flash);
            flash.cr.modify(|_, w| w.pg().set_bit());
            unsafe {
                ptr::write_volatile(addr as *mut u16, data as u16);
                ptr::write_volatile(
                    (addr as usize + 2) as *mut u16,
                    (data.wrapping_shr(16)) as u16,
                );
            }
            while flash.sr.read().bsy().bit_is_set() {}
            flash.cr.modify(|_, w| w.lock().set_bit());
            if flash.sr.read().eop().bit_is_set() {
                flash.sr.modify(|_, w| w.eop().set_bit());
            }
            flash.cr.modify(|_, w| w.pg().clear_bit());
        }
    }
    use my_flash::{unlock, wait_ready, clear_error_flags, erase_page, write_to_flash};
    use core::mem;
    use core::ptr;
    use cortex_m::asm::{nop, self};
    use cortex_m_semihosting::hprintln;
    use ::core::arch::asm;
    use stm32f3xx_hal_v2::{pac::Peripherals, pac::FLASH};
    use volatile::Volatile;
    pub static mut transcation_log: u32 = 0x60004000;
    pub static mut execution_mode: bool = true;
    pub static mut counter: *mut u8 = 0x60003002 as *mut u8;
    pub fn initialization() {
        unsafe {
            let dp = Peripherals::steal();
            dp.RCC.cr.write(|w| w.hsion().set_bit());
            while dp.RCC.cr.read().hsirdy().bit_is_clear() {}
            dp.RCC.cr.modify(|_r, w| w.pllon().clear_bit());
            while dp.RCC.cr.read().pllrdy().bit_is_set() {}
            dp.RCC.cfgr.modify(|_, w| w.pllsrc().hsi_div_prediv());
            dp.RCC.cfgr2.modify(|_, w| w.prediv().div1());
            dp.RCC.cfgr.modify(|_, w| w.pllmul().mul9());
            dp.RCC.cr.modify(|_, w| w.pllon().on());
            while dp.RCC.cr.read().pllrdy().bit_is_clear() {}
            dp.RCC.cfgr.modify(|_, w| { w.hpre().div1().ppre1().div2().ppre2().div1() });
            dp.FLASH.acr.modify(|_, w| w.prftbe().enabled().latency().ws1());
            dp.RCC.cfgr.modify(|_, w| w.sw().pll());
            while dp.RCC.cfgr.read().sw().bits() != 0b10 {}
            while dp.RCC.cfgr.read().sws().bits() != 0b10 {}
            dp.RCC.ahbenr.modify(|_, w| w.iopden().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.iopeen().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.iopfen().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.iopgen().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.iophen().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.sramen().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.flitfen().set_bit());
            dp.RCC.ahbenr.modify(|_, w| w.fmcen().set_bit());
            dp.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
            dp.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());
            let mut gpiod = dp.GPIOD;
            let mut gpioe = dp.GPIOE;
            let mut gpiof = dp.GPIOF;
            let mut gpiog = dp.GPIOG;
            let mut gpioh = dp.GPIOH;
            gpioh.moder.modify(|_, w| { w.moder0().alternate() });
            gpioh.afrl.modify(|_, w| { w.afrl0().af12() });
            gpioh.ospeedr.modify(|_, w| w.ospeedr0().very_high_speed());
            gpioh.moder.modify(|_, w| { w.moder1().alternate() });
            gpioh.afrl.modify(|_, w| { w.afrl1().af12() });
            gpioh.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder2().alternate() });
            gpiof.afrl.modify(|_, w| { w.afrl2().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder3().alternate() });
            gpiof.afrl.modify(|_, w| { w.afrl3().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr3().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder4().alternate() });
            gpiof.afrl.modify(|_, w| { w.afrl4().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder5().alternate() });
            gpiof.afrl.modify(|_, w| { w.afrl5().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder12().alternate() });
            gpiof.afrh.modify(|_, w| { w.afrh12().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr12().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder13().alternate() });
            gpiof.afrh.modify(|_, w| { w.afrh13().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr13().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder14().alternate() });
            gpiof.afrh.modify(|_, w| { w.afrh14().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr14().very_high_speed());
            gpiof.moder.modify(|_, w| { w.moder15().alternate() });
            gpiof.afrh.modify(|_, w| { w.afrh15().af12() });
            gpiof.ospeedr.modify(|_, w| w.ospeedr15().very_high_speed());
            gpiog.moder.modify(|_, w| { w.moder0().alternate() });
            gpiog.afrl.modify(|_, w| { w.afrl0().af12() });
            gpiog.ospeedr.modify(|_, w| w.ospeedr0().very_high_speed());
            gpiog.moder.modify(|_, w| { w.moder1().alternate() });
            gpiog.afrl.modify(|_, w| { w.afrl1().af12() });
            gpiog.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());
            gpiog.moder.modify(|_, w| { w.moder2().alternate() });
            gpiog.afrl.modify(|_, w| { w.afrl2().af12() });
            gpiog.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());
            gpiog.moder.modify(|_, w| { w.moder3().alternate() });
            gpiog.afrl.modify(|_, w| { w.afrl3().af12() });
            gpiog.ospeedr.modify(|_, w| w.ospeedr3().very_high_speed());
            gpiog.moder.modify(|_, w| { w.moder4().alternate() });
            gpiog.afrl.modify(|_, w| { w.afrl4().af12() });
            gpiog.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
            gpiog.moder.modify(|_, w| { w.moder5().alternate() });
            gpiog.afrl.modify(|_, w| { w.afrl5().af12() });
            gpiog.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder14().alternate() });
            gpiod.afrh.modify(|_, w| { w.afrh14().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr14().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder15().alternate() });
            gpiod.afrh.modify(|_, w| { w.afrh15().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr15().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder0().alternate() });
            gpiod.afrl.modify(|_, w| { w.afrl0().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr0().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder1().alternate() });
            gpiod.afrl.modify(|_, w| { w.afrl1().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr1().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder7().alternate() });
            gpioe.afrl.modify(|_, w| { w.afrl7().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder8().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh8().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder9().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh9().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder10().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh10().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder11().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh11().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr11().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder12().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh12().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr12().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder13().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh13().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr13().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder14().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh14().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr14().very_high_speed());
            gpioe.moder.modify(|_, w| { w.moder15().alternate() });
            gpioe.afrh.modify(|_, w| { w.afrh15().af12() });
            gpioe.ospeedr.modify(|_, w| w.ospeedr15().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder8().alternate() });
            gpiod.afrh.modify(|_, w| { w.afrh8().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr8().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder9().alternate() });
            gpiod.afrh.modify(|_, w| { w.afrh9().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder10().alternate() });
            gpiod.afrh.modify(|_, w| { w.afrh10().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder7().alternate() });
            gpiod.afrl.modify(|_, w| { w.afrl7().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder4().alternate() });
            gpiod.afrl.modify(|_, w| { w.afrl4().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
            gpiod.moder.modify(|_, w| { w.moder5().alternate() });
            gpiod.afrl.modify(|_, w| { w.afrl5().af12() });
            gpiod.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
            dp.FMC
                .bcr1
                .modify(|_, w| {
                    w.mbken().set_bit();
                    w.mtyp().bits(0b00);
                    w.mwid().bits(0b01);
                    w.bursten().clear_bit();
                    w.wren().clear_bit();
                    w.muxen().clear_bit();
                    w.extmod().clear_bit();
                    w.asyncwait().clear_bit();
                    w
                });
            dp.FMC
                .btr1
                .modify(|_, w| {
                    w.addset().bits(0x1);
                    w.datast().bits(0x5);
                    w.addhld().bits(0x1);
                    w.busturn().bits(0x0);
                    w.clkdiv().bits(0x0);
                    w.datlat().bits(0x0);
                    w.accmod().bits(0x0);
                    w
                });
        }
    }
    pub fn save_variables<T>(mem_loc: *const T, size: usize) {
        unsafe {
            let mem_loc_u16 = mem_loc as *const u16;
            for i in 0..2 {
                let byte = (mem_loc_u16 as u32 >> (i * 16)) as u16;
                ptr::write((transcation_log + (2 * i)) as *mut u16, byte);
                let a = ptr::read_volatile((transcation_log + (2 * i)) as *mut u16);
            }
            transcation_log += 4;
            ptr::write(transcation_log as *mut u16, size as u16);
            transcation_log += 2 * 1;
            for i in 0..size / 2 {
                let byte = *mem_loc_u16.add(i);
                ptr::write((transcation_log + (2 * i as u32)) as *mut u16, byte);
                let a = ptr::read_volatile(
                    (transcation_log + (2 * i as u32)) as *mut u16,
                );
            }
            transcation_log = transcation_log + 2 * size as u32;
            *counter += 1;
        }
    }
    pub fn start_atomic() {
        checkpoint(true);
        unsafe {
            execution_mode = false;
        }
    }
    pub fn end_atomic() {
        unsafe {
            transcation_log = 0x60004000;
        }
        unsafe {
            execution_mode = true;
        }
    }
    #[no_mangle]
    pub fn checkpoint(c_type: bool) {
        let r0_value: u32;
        let r1_value: u32;
        let r2_value: u32;
        let r3_value: u32;
        let r4_value: u32;
        let r5_value: u32;
        let r6_value: u32;
        let r7_value: u32;
        let r8_value: u32;
        let r9_value: u32;
        let r10_value: u32;
        let r11_value: u32;
        let r12_value: u32;
        let r13_sp: u32;
        let r14_lr: u32;
        let r15_pc: u32;
        unsafe {
            asm!("MOV {0}, r0", out(reg) r0_value);
        }
        unsafe {
            asm!("MOV {0}, r1", out(reg) r1_value);
        }
        unsafe {
            asm!("MOV {0}, r2", out(reg) r2_value);
        }
        unsafe {
            asm!("MOV {0}, r3", out(reg) r3_value);
        }
        unsafe {
            asm!("MOV {0}, r4", out(reg) r4_value);
        }
        unsafe {
            asm!("MOV {0}, r5", out(reg) r5_value);
        }
        unsafe {
            asm!("MOV {0}, r6", out(reg) r6_value);
        }
        unsafe {
            asm!("MOV {0}, r7", out(reg) r7_value);
        }
        unsafe {
            asm!("MOV {0}, r8", out(reg) r8_value);
        }
        unsafe {
            asm!("MOV {0}, r9", out(reg) r9_value);
        }
        unsafe {
            asm!("MOV {0}, r10", out(reg) r10_value);
        }
        unsafe {
            asm!("MOV {0}, r11", out(reg) r11_value);
        }
        unsafe {
            asm!("MOV {0}, r12", out(reg) r12_value);
        }
        unsafe {
            asm!("MOV {0}, r14", out(reg) r14_lr);
        }
        unsafe {
            asm!("MOV {0}, r15", out(reg) r15_pc);
        }
        unsafe {
            asm!("MOV r0, sp");
        }
        unsafe {
            asm!("add r0, #112");
        }
        unsafe {
            asm!("MOV {0}, r0", out(reg) r13_sp);
        }
        unsafe {
            let dp = Peripherals::steal();
            let mut flash = dp.FLASH;
            unlock(&mut flash);
            wait_ready(&flash);
            let mut start_address: u32;
            let end_address = r13_sp;
            asm!("movw r0, 0xFFFC\n             movt r0, 0x2000");
            asm!("MOV {0}, r0", out(reg) start_address);
            let stack_size = (start_address - end_address) + 4;
            let mut flash_start_address = Volatile::new(0x0803_0000);
            let mut flash_end_address = Volatile::new(0x0807_FFFF);
            let mut checkpoint_size = Volatile::new(0u32);
            asm::dmb();
            checkpoint_size.write(stack_size + 4 + 16 * 4 + 4 + 4);
            asm::dmb();
            loop {
                let mut offset = ptr::read_volatile(
                    flash_start_address.read() as *const u32,
                );
                if offset == 0xffff_ffff {
                    break;
                }
                flash_start_address.write(flash_start_address.read() + offset);
                if flash_start_address.read() + checkpoint_size.read()
                    >= flash_end_address.read()
                {
                    erase_all(&mut flash);
                    flash_start_address = Volatile::new(0x0803_0000);
                    break;
                }
            }
            asm::dmb();
            write_to_flash(
                &mut flash,
                (flash_start_address.read()) as u32,
                checkpoint_size.read() as u32,
            );
            flash_start_address.write(flash_start_address.read() + 4);
            asm::dmb();
            asm::dmb();
            if c_type {
                write_to_flash(
                    &mut flash,
                    flash_start_address.read() as u32,
                    0xDEADBEEF as u32,
                );
            } else {
                write_to_flash(
                    &mut flash,
                    flash_start_address.read() as u32,
                    0x0000_0001 as u32,
                );
            }
            while start_address >= end_address {
                let mut data = Volatile::new(0u32);
                data.write(core::ptr::read_volatile(start_address as *const u32));
                write_to_flash(
                    &mut flash,
                    flash_start_address.read() as u32,
                    data.read() as u32,
                );
                flash_start_address.write(flash_start_address.read() + 1 * 4);
                start_address = start_address - 4;
            }
            asm::dmb();
            asm::dmb();
            write_to_flash(
                &mut flash,
                (flash_start_address.read()) as u32,
                0xf1f1_f1f1 as u32,
            );
            flash_start_address.write(flash_start_address.read() + 4);
            asm::dmb();
        }
    }
    pub fn erase_all(flash: &mut FLASH) {
        let start_address = 0x0803_0000;
        for i in 0..100 {
            let page = start_address + i * 2 * 1024;
            erase_page(flash, page);
        }
    }
    pub fn restore_globals() {
        unsafe {
            let mut restore_ctr: u8 = 0;
            loop {
                if *counter == restore_ctr {
                    break;
                }
                let mut combined: u32 = 0;
                for i in 0..2 {
                    combined
                        |= (ptr::read((transcation_log + i) as *const u32) << (i * 16));
                }
                let mut size: u16 = ptr::read(transcation_log as *const u16);
                for i in 0..size {
                    ptr::write(
                        (combined + i as u32) as *mut u16,
                        *((transcation_log + i as u32) as *const u16),
                    );
                }
                combined = combined + size as u32;
                let end = ptr::read(combined as *const u16);
                restore_ctr += 1;
            }
        }
    }
    pub fn restore() -> bool {
        unsafe {
            let mut flash_start_address = 0x0803_0000;
            let packet_size = ptr::read_volatile(0x0803_0000 as *const u32);
            if packet_size == 0xffff_ffff {
                return false;
            }
            if ptr::read_volatile((flash_start_address + packet_size) as *const u32)
                == 0xffff_ffff
            {
                return false;
            }
            let mut offset: u32 = 0;
            loop {
                offset = ptr::read_volatile(flash_start_address as *const u32);
                if ptr::read_volatile((flash_start_address + offset) as *const u32)
                    == 0xffff_ffff
                {
                    break;
                }
                flash_start_address += offset;
            }
            flash_start_address += 4;
            if ptr::read_volatile(flash_start_address as *const u32) == 0xDEAD_BEEF {
                restore_globals();
                *counter = 0;
            }
            flash_start_address += 4;
            asm!("mov r0, {0}", in (reg) flash_start_address);
            asm!("movw r1, 0xfff8\n        movt r1, 0x02000");
            asm!("msr msp, r1");
            asm!("movw r3, 0xf1f1\n        movt r3, 0xf1f1");
            asm!(
                "1:\n            ldr r1, [r0, #4]\n            cmp r1, r3\n            beq 2f\n            push {{r1}}\n            adds r0, r0, #4\n            b 1b\n            2:"
            );
            asm!("adds r0, r0, #4");
            asm!("adds r0, r0, #4");
            asm!("LDR r1, [r0]");
            asm!("Push {{r1}}");
            asm!("adds r0, r0, #4");
            asm!("LDR r1, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r2, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r3, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r4, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r5, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r6, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r7, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r8, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r9, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r10, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r11, [r0]");
            asm!("adds r0, r0, #4");
            asm!("LDR r12, [r0]");
            asm!("adds r0, r0, #4");
            asm!("adds r0, r0, #4");
            asm!("LDR r14, [r0]");
            asm!("POP {{r0}}");
            asm!("cpsie i");
            asm!("mov r15, r14");
            asm!("adds sp, sp, #56");
            asm!("adds sp, sp, #8");
            asm!("POP {{r0, r1, r2, r3}}");
            asm!("adds sp, sp, #4");
            asm!("POP {{r4}}");
            asm!("adds sp, sp, #16");
            asm!("adds sp, sp, #64");
            asm!("mov pc, r4");
        }
        return true;
    }
    pub fn delete_pg(page: u32) {
        unsafe {
            let mut dp = Peripherals::steal();
            let mut flash = &mut dp.FLASH;
            unlock(&mut flash);
            wait_ready(&flash);
            erase_page(&mut flash, page);
        }
    }
    pub fn delete_all_pg() {
        let start_address = 0x0803_0000;
        unsafe {
            let mut dp = Peripherals::steal();
            let mut flash = &mut dp.FLASH;
            for i in 0..25 {
                let page = start_address + i * 2 * 1024;
                unlock(&mut flash);
                wait_ready(&flash);
                erase_page(&mut flash, page);
            }
        }
    }
    #[no_mangle]
    pub fn c_checkpoint(c_type: bool) {
        unsafe {
            asm!("push {{r1}}");
        }
        unsafe {
            asm!("add sp, #80");
        }
        unsafe { asm!("pop\t{{r8, r9, sl, fp}}") };
        unsafe { asm!("pop\t{{r4, r5, r6, r7, lr}}") };
        unsafe { asm!("push\t{{r4, r5, r6, r7, lr}}") };
        unsafe { asm!("push\t{{r8, r9, sl, fp}}") };
        unsafe {
            asm!("sub sp, #80");
        }
        unsafe {
            asm!("pop {{r1}}");
        }
        let r0_value: u32;
        let r1_value: u32;
        let r2_value: u32;
        let r3_value: u32;
        let r4_value: u32;
        let r5_value: u32;
        let r6_value: u32;
        let r7_value: u32;
        let r8_value: u32;
        let r9_value: u32;
        let r10_value: u32;
        let r11_value: u32;
        let r12_value: u32;
        let r13_sp: u32;
        let r14_lr: u32;
        let r15_pc: u32;
        unsafe {
            asm!("MOV {0}, r0", out(reg) r0_value);
        }
        unsafe {
            asm!("MOV {0}, r1", out(reg) r1_value);
        }
        unsafe {
            asm!("MOV {0}, r2", out(reg) r2_value);
        }
        unsafe {
            asm!("MOV {0}, r3", out(reg) r3_value);
        }
        unsafe {
            asm!("MOV {0}, r4", out(reg) r4_value);
        }
        unsafe {
            asm!("MOV {0}, r5", out(reg) r5_value);
        }
        unsafe {
            asm!("MOV {0}, r14", out(reg) r14_lr);
        }
        unsafe {
            asm!("MOV {0}, r7", out(reg) r7_value);
        }
        unsafe {
            asm!("MOV {0}, r8", out(reg) r8_value);
        }
        unsafe {
            asm!("MOV {0}, r9", out(reg) r9_value);
        }
        unsafe {
            asm!("MOV {0}, r10", out(reg) r10_value);
        }
        unsafe {
            asm!("MOV {0}, r11", out(reg) r11_value);
        }
        unsafe {
            asm!("MOV {0}, r12", out(reg) r12_value);
        }
        unsafe {
            asm!("MOV {0}, r6", out(reg) r6_value);
        }
        unsafe {
            asm!("MOV {0}, r15", out(reg) r15_pc);
        }
        unsafe {
            asm!("MOV r0, sp");
        }
        unsafe {
            asm!("add r0, #108");
        }
        unsafe {
            asm!("MOV {0}, r0", out(reg) r13_sp);
        }
        unsafe {
            let dp = Peripherals::steal();
            let mut flash = dp.FLASH;
            unlock(&mut flash);
            wait_ready(&flash);
            let mut start_address: u32;
            let end_address = r13_sp;
            asm!("movw r0, 0xFFFC\n         movt r0, 0x2000");
            asm!("MOV {0}, r0", out(reg) start_address);
            let stack_size = (start_address - end_address) + 4;
            let mut flash_start_address = Volatile::new(0x0803_0000);
            let mut flash_end_address = Volatile::new(0x0807_FFFF);
            let mut checkpoint_size = Volatile::new(0u32);
            checkpoint_size.write(stack_size + 4 + 16 * 4 + 4 + 4);
            asm::dmb();
            loop {
                let mut offset = ptr::read_volatile(
                    flash_start_address.read() as *const u32,
                );
                if offset == 0xffff_ffff {
                    break;
                }
                flash_start_address.write(flash_start_address.read() + offset);
                if flash_start_address.read() + checkpoint_size.read()
                    >= flash_end_address.read()
                {
                    erase_all(&mut flash);
                    flash_start_address = Volatile::new(0x0803_0000);
                    break;
                }
            }
            asm::dmb();
            write_to_flash(
                &mut flash,
                (flash_start_address.read()) as u32,
                checkpoint_size.read() as u32,
            );
            flash_start_address.write(flash_start_address.read() + 4);
            asm::dmb();
            asm::dmb();
            if c_type {
                write_to_flash(
                    &mut flash,
                    flash_start_address.read() as u32,
                    0xDEADBEEF as u32,
                );
            } else {
                write_to_flash(
                    &mut flash,
                    flash_start_address.read() as u32,
                    0x0000_0001 as u32,
                );
            }
            while start_address >= end_address {
                let mut data = Volatile::new(0u32);
                data.write(core::ptr::read_volatile(start_address as *const u32));
                write_to_flash(
                    &mut flash,
                    flash_start_address.read() as u32,
                    data.read() as u32,
                );
                flash_start_address.write(flash_start_address.read() + 1 * 4);
                start_address = start_address - 4;
            }
            asm::dmb();
            asm::dmb();
            write_to_flash(
                &mut flash,
                (flash_start_address.read()) as u32,
                0xf1f1_f1f1 as u32,
            );
            flash_start_address.write(flash_start_address.read() + 4);
            asm::dmb();
            write_to_flash(
                &mut flash,
                flash_start_address.read() as u32,
                r0_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 4 as u32,
                r1_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 8 as u32,
                r2_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 12 as u32,
                r3_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 16 as u32,
                r4_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 20 as u32,
                r5_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 24 as u32,
                r6_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 28 as u32,
                r7_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 32 as u32,
                r8_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 36 as u32,
                r9_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 40 as u32,
                r10_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 44 as u32,
                r11_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 48 as u32,
                r12_value as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 52 as u32,
                r13_sp as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 56 as u32,
                r14_lr as u32,
            );
            write_to_flash(
                &mut flash,
                flash_start_address.read() + 60 as u32,
                r15_pc as u32,
            );
        }
    }
}
use checkpoint::{
    checkpoint, restore, delete_pg, delete_all_pg, transcation_log, execution_mode,
    counter, start_atomic, end_atomic,
};
use volatile::Volatile;
use checkpoint::my_flash::{
    unlock, wait_ready, clear_error_flags, erase_page, write_to_flash,
};
/// The RTIC application module
pub mod app {
    /// Always include the device crate which contains the vector table
    use stm32f3xx_hal_v2::pac as you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml;
    /// Holds the maximum priority level for use by async HAL drivers.
    #[no_mangle]
    static RTIC_ASYNC_MAX_LOGICAL_PRIO: u8 = 1 << stm32f3xx_hal_v2::pac::NVIC_PRIO_BITS;
    use cortex_m::peripheral::DWT;
    use core::arch::asm;
    use core::mem;
    use core::ptr;
    use cortex_m::asm::{nop, self};
    use crate::checkpoint::end_atomic;
    use crate::checkpoint::start_atomic;
    use crate::checkpoint::save_variables;
    use crate::checkpoint::{
        self, delete_all_pg, restore, checkpoint, c_checkpoint, erase_all,
    };
    use crate::checkpoint::my_flash::{
        unlock, wait_ready, clear_error_flags, erase_page, write_to_flash,
    };
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt::exception;
    use stm32f3xx_hal_v2::{
        pac::{self, NVIC},
        pac::Peripherals, pac::FLASH, pac::Interrupt, gpio::{gpioa::PA0, Input, PullUp},
    };
    use volatile::Volatile;
    use cortex_m_semihosting::{debug, hprintln};
    use crate::checkpoint::initialization;
    const NUM_WARMUP_SAMPLES: u16 = 3;
    const ACCEL_WINDOW_SIZE: usize = 3;
    const MODEL_SIZE: usize = 16;
    const SAMPLE_NOISE_FLOOR: u8 = 10;
    const SAMPLES_TO_COLLECT: u16 = 128;
    struct threeAxis_t_8 {
        x: u8,
        y: u8,
        z: u8,
    }
    type accelReading = threeAxis_t_8;
    #[allow(non_camel_case_types)]
    type accelWindow = [accelReading; ACCEL_WINDOW_SIZE as usize];
    struct features_t {
        meanmag: u16,
        stddevmag: u16,
    }
    enum class_t {
        CLASS_STATIONARY,
        CLASS_MOVING,
    }
    struct model_t {
        stationary: [features_t; MODEL_SIZE as usize],
        moving: [features_t; MODEL_SIZE as usize],
    }
    enum run_mode_t {
        MODE_IDLE = 3,
        MODE_TRAIN_STATIONARY = 2,
        MODE_TRAIN_MOVING = 1,
        MODE_RECOGNIZE = 0,
    }
    struct stats_t {
        totalCount: u16,
        movingCount: u16,
        stationaryCount: u16,
    }
    struct input {
        xs: u8,
        ys: u8,
        zs: u8,
    }
    static mut input1: input = input { xs: 0, ys: 0, zs: 0 };
    #[no_mangle]
    pub static IO_NAME: fn(u16) -> u8 = readSensor;
    #[no_mangle]
    fn readSensor(input: u16) -> u8 {
        return (input % 85) as u8;
    }
    fn accel_sample(nv_seed: &mut u16) -> threeAxis_t_8 {
        let mut seed: u16 = *nv_seed;
        async_task1::spawn(seed);
        let xs = unsafe { input1.xs };
        let tmp: u32 = (seed as u32 * 17 * 17 * 17) % 85;
        let ys = tmp as u8;
        let tmp: u32 = (seed as u32 * 17 * 17 * 17) % 85;
        let zs = tmp as u8;
        let result: threeAxis_t_8 = threeAxis_t_8 {
            x: xs,
            y: ys,
            z: zs,
        };
        seed = seed + 1;
        *nv_seed = seed;
        return result;
    }
    fn acquire_window(window: &mut accelWindow, seed: &mut u16) -> () {
        let mut samplesInWindow: usize = 0;
        while samplesInWindow < ACCEL_WINDOW_SIZE {
            let sample: accelReading = accel_sample(seed);
            window[samplesInWindow] = sample;
            samplesInWindow += 1;
        }
    }
    fn transform(window: &mut accelWindow) -> () {
        for i in 0..ACCEL_WINDOW_SIZE {
            let sample: &mut accelReading = &mut window[i];
            if sample.x < SAMPLE_NOISE_FLOOR || sample.y < SAMPLE_NOISE_FLOOR
                || sample.z < SAMPLE_NOISE_FLOOR
            {
                (*sample).x = if sample.x > SAMPLE_NOISE_FLOOR { sample.x } else { 0 };
                (*sample).y = if sample.y > SAMPLE_NOISE_FLOOR { sample.y } else { 0 };
                (*sample).z = if sample.z > SAMPLE_NOISE_FLOOR { sample.z } else { 0 };
            }
        }
    }
    fn featurize(features: &mut features_t, aWin: &accelWindow) -> () {
        let mut mean = accelReading { x: 0, y: 0, z: 0 };
        let mut stddev = accelReading { x: 0, y: 0, z: 0 };
        for i in 0..ACCEL_WINDOW_SIZE {
            mean.x += aWin[i].x;
            mean.y += aWin[i].y;
            mean.z += aWin[i].z;
        }
        mean.x >>= 2;
        mean.y >>= 2;
        mean.z >>= 2;
        for i in 0..ACCEL_WINDOW_SIZE {
            stddev.x
                += if aWin[i].x > mean.x {
                    aWin[i].x - mean.x
                } else {
                    mean.x - aWin[i].x
                };
            stddev.y
                += if aWin[i].y > mean.y {
                    aWin[i].y - mean.y
                } else {
                    mean.y - aWin[i].y
                };
            stddev.z
                += if aWin[i].z > mean.z {
                    aWin[i].z - mean.z
                } else {
                    mean.z - aWin[i].z
                };
        }
        stddev.x >>= 2;
        stddev.y >>= 2;
        stddev.z >>= 2;
        let meanmag: u32 = mean.x as u32 * mean.x as u32 + mean.y as u32 * mean.y as u32
            + mean.z as u32 * mean.z as u32;
        let stddevmag: u32 = stddev.x as u32 * stddev.x as u32
            + stddev.y as u32 * stddev.y as u32 + stddev.z as u32 * stddev.z as u32;
        features.meanmag = libm::sqrtf(meanmag as f32) as u16;
        features.stddevmag = libm::sqrtf(stddevmag as f32) as u16;
    }
    fn classify(features: &features_t, model: &model_t) -> class_t {
        let mut move_less_error: i16 = 0;
        let mut stat_less_error: i16 = 0;
        let mut model_features: features_t = features_t {
            meanmag: 0,
            stddevmag: 0,
        };
        for i in 0..MODEL_SIZE {
            model_features.meanmag = model.stationary[i].meanmag;
            model_features.stddevmag = model.stationary[i].stddevmag;
            let stat_mean_err: i32 = if model_features.meanmag > features.meanmag {
                (model_features.meanmag - features.meanmag) as i32
            } else {
                (features.meanmag - model_features.meanmag) as i32
            };
            let stat_sd_err: i32 = if model_features.stddevmag > features.stddevmag {
                (model_features.stddevmag - features.stddevmag) as i32
            } else {
                (features.stddevmag - model_features.stddevmag) as i32
            };
            model_features.meanmag = model.moving[i].meanmag;
            model_features.stddevmag = model.moving[i].stddevmag;
            let move_mean_err: i32 = if model_features.meanmag > features.meanmag {
                (model_features.meanmag - features.meanmag) as i32
            } else {
                (features.meanmag - model_features.meanmag) as i32
            };
            let move_sd_err: i32 = if model_features.stddevmag > features.stddevmag {
                (model_features.stddevmag - features.stddevmag) as i32
            } else {
                (features.stddevmag - model_features.stddevmag) as i32
            };
            if move_mean_err < stat_mean_err {
                move_less_error += 1;
            } else {
                stat_less_error += 1;
            }
            if move_sd_err < stat_sd_err {
                move_less_error += 1;
            } else {
                stat_less_error += 1;
            }
        }
        let class: class_t = if move_less_error > stat_less_error {
            class_t::CLASS_MOVING
        } else {
            class_t::CLASS_STATIONARY
        };
        return class;
    }
    fn record_stats(stats: &mut stats_t, class: class_t) -> () {
        stats.totalCount += 1;
        match class {
            class_t::CLASS_MOVING => stats.movingCount += 1,
            class_t::CLASS_STATIONARY => stats.stationaryCount += 1,
        }
    }
    fn print_stats(stats: &stats_t) -> () {
        let resultStationaryPct = stats.stationaryCount * 100 / stats.totalCount;
        let resultMovingPct = stats.movingCount * 100 / stats.totalCount;
        let sum = stats.stationaryCount + stats.movingCount;
        unsafe {
            ::cortex_m_semihosting::export::hstdout_fmt(
                format_args!(
                    "stats: s {0} ({1}) m {2} ({3}) sum/tot {4}/{5}: {6}\n",
                    stats.stationaryCount as u32,
                    resultStationaryPct as u32,
                    stats.movingCount as u32,
                    resultMovingPct as u32,
                    stats.totalCount as u32,
                    sum as u32,
                    if sum == stats.totalCount && sum == SAMPLES_TO_COLLECT {
                        'V'
                    } else {
                        'X'
                    },
                ),
            );
        }
    }
    fn warmup_sensor(seed: &mut u16) -> () {
        let mut discardedSamplesCount: u16 = 0;
        let mut _sample: accelReading;
        while discardedSamplesCount < NUM_WARMUP_SAMPLES {
            _sample = accel_sample(seed);
            discardedSamplesCount += 1;
        }
    }
    fn train(
        classModel: &mut [features_t; MODEL_SIZE],
        seed: &mut u16,
        id: &idle::Context,
    ) -> () {
        let mut sampleWindow: accelWindow = [
            accelReading { x: 0, y: 0, z: 0 },
            accelReading { x: 0, y: 0, z: 0 },
            accelReading { x: 0, y: 0, z: 0 },
        ];
        let mut features: features_t = features_t {
            meanmag: 0,
            stddevmag: 0,
        };
        warmup_sensor(seed);
        for i in 0..MODEL_SIZE {
            acquire_window(&mut sampleWindow, seed);
            transform(&mut sampleWindow);
            featurize(&mut features, &sampleWindow);
            classModel[i].meanmag = features.meanmag;
            classModel[i].stddevmag = features.stddevmag;
        }
    }
    fn recognize(model: &model_t, seed: &mut u16) -> () {
        let mut stats: stats_t = stats_t {
            totalCount: 0,
            movingCount: 0,
            stationaryCount: 0,
        };
        let mut sampleWindow: accelWindow = [
            accelReading { x: 0, y: 0, z: 0 },
            accelReading { x: 0, y: 0, z: 0 },
            accelReading { x: 0, y: 0, z: 0 },
        ];
        let mut features: features_t = features_t {
            meanmag: 0,
            stddevmag: 0,
        };
        let mut class: class_t;
        for _i in 0..SAMPLES_TO_COLLECT {
            acquire_window(&mut sampleWindow, seed);
            transform(&mut sampleWindow);
            featurize(&mut features, &sampleWindow);
            class = classify(&features, model);
            record_stats(&mut stats, class);
        }
        print_stats(&stats);
    }
    fn end_of_benchmark() -> () {}
    fn count_error(count: &u16) -> () {
        unsafe {
            ::cortex_m_semihosting::export::hstdout_fmt(
                format_args!("An error occured during count, count = {0}\n", count),
            );
        }
    }
    fn select_mode(prev_pin_state: &mut u8, count: &mut u16) -> u8 {
        let mut pin_state: u8 = run_mode_t::MODE_IDLE as u8;
        *count = *count + 1;
        match *count {
            1 | 2 => pin_state = run_mode_t::MODE_TRAIN_MOVING as u8,
            3 | 4 => pin_state = run_mode_t::MODE_TRAIN_STATIONARY as u8,
            5 | 6 => pin_state = run_mode_t::MODE_RECOGNIZE as u8,
            7 => end_of_benchmark(),
            _ => {
                pin_state = run_mode_t::MODE_IDLE as u8;
                count_error(count)
            }
        }
        if *count == 7 {
            return run_mode_t::MODE_IDLE as u8;
        }
        if (pin_state == run_mode_t::MODE_TRAIN_STATIONARY as u8
            || pin_state == run_mode_t::MODE_TRAIN_MOVING as u8)
            && pin_state == *prev_pin_state
        {
            pin_state = run_mode_t::MODE_IDLE as u8;
        } else {
            *prev_pin_state = pin_state;
        }
        return pin_state;
    }
    /// User code end
    ///Shared resources
    struct Shared {
        a: u8,
    }
    ///Local resources
    struct Local {}
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_init_Context<'a> {
        #[doc(hidden)]
        __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
        /// The space used to allocate async executors in bytes.
        pub executors_size: usize,
        /// Core peripherals
        pub core: rtic::export::Peripherals,
        /// Device peripherals (PAC)
        pub device: stm32f3xx_hal_v2::pac::Peripherals,
        /// Critical section token for init
        pub cs: rtic::export::CriticalSection<'a>,
    }
    impl<'a> __rtic_internal_init_Context<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new(
            core: rtic::export::Peripherals,
            executors_size: usize,
        ) -> Self {
            __rtic_internal_init_Context {
                __rtic_internal_p: ::core::marker::PhantomData,
                core: core,
                device: stm32f3xx_hal_v2::pac::Peripherals::steal(),
                cs: rtic::export::CriticalSection::new(),
                executors_size,
            }
        }
    }
    #[allow(non_snake_case)]
    ///Initialization function
    pub mod init {
        #[doc(inline)]
        pub use super::__rtic_internal_init_Context as Context;
    }
    #[inline(always)]
    #[allow(non_snake_case)]
    fn init(cx: init::Context) -> (Shared, Local) {
        ::cortex_m_semihosting::export::hstdout_str("init\n");
        let mut cp = cx.core;
        cp.DWT.enable_cycle_counter();
        (Shared { a: 0 }, Local {})
    }
    impl<'a> __rtic_internal_idleSharedResources<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_idleSharedResources {
                a: shared_resources::a_that_needs_to_be_locked::new(),
                __rtic_internal_marker: core::marker::PhantomData,
            }
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    ///Shared resources `idle` has access to
    pub struct __rtic_internal_idleSharedResources<'a> {
        #[allow(missing_docs)]
        pub a: shared_resources::a_that_needs_to_be_locked<'a>,
        #[doc(hidden)]
        pub __rtic_internal_marker: core::marker::PhantomData<&'a ()>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_idle_Context<'a> {
        #[doc(hidden)]
        __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
        /// Shared Resources this task has access to
        pub shared: idle::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_idle_Context<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_idle_Context {
                __rtic_internal_p: ::core::marker::PhantomData,
                shared: idle::SharedResources::new(),
            }
        }
    }
    #[allow(non_snake_case)]
    ///Idle loop
    pub mod idle {
        #[doc(inline)]
        pub use super::__rtic_internal_idleSharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_idle_Context as Context;
    }
    #[allow(non_snake_case)]
    fn idle(mut id: idle::Context) -> ! {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        let mut prev_pin_state: u8 = run_mode_t::MODE_IDLE as u8;
        let count: &'static mut u16 = unsafe {
            static mut COUNT_NV: u16 = 1;
            &mut COUNT_NV
        };
        let model: &'static mut model_t = unsafe {
            static mut MODEL_NV: model_t = model_t {
                moving: [
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                ],
                stationary: [
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                    features_t {
                        meanmag: 0,
                        stddevmag: 0,
                    },
                ],
            };
            &mut MODEL_NV
        };
        let _v_seed: &'static mut u16 = unsafe {
            static mut SEED_NV: u16 = 1;
            &mut SEED_NV
        };
        loop {
            let mut localSeed = *_v_seed;
            let mode: u8 = select_mode(&mut prev_pin_state, count);
            if mode == 2 {
                train(&mut model.stationary, &mut localSeed, &id);
            } else if mode == 1 {
                train(&mut model.moving, &mut localSeed, &id);
            } else if mode == 0 {
                recognize(&model, &mut localSeed);
            } else if mode == 3 && *count == 7 {
                localSeed = 1;
                *count = 1;
                prev_pin_state = run_mode_t::MODE_IDLE as u8;
                unsafe {}
            }
            *_v_seed = localSeed;
        }
    }
    impl<'a> __rtic_internal_async_task1SharedResources<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_async_task1SharedResources {
                a: shared_resources::a_that_needs_to_be_locked::new(),
                __rtic_internal_marker: core::marker::PhantomData,
            }
        }
    }
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    ///Shared resources `async_task1` has access to
    pub struct __rtic_internal_async_task1SharedResources<'a> {
        #[allow(missing_docs)]
        pub a: shared_resources::a_that_needs_to_be_locked<'a>,
        #[doc(hidden)]
        pub __rtic_internal_marker: core::marker::PhantomData<&'a ()>,
    }
    /// Execution context
    #[allow(non_snake_case)]
    #[allow(non_camel_case_types)]
    pub struct __rtic_internal_async_task1_Context<'a> {
        #[doc(hidden)]
        __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
        /// Shared Resources this task has access to
        pub shared: async_task1::SharedResources<'a>,
    }
    impl<'a> __rtic_internal_async_task1_Context<'a> {
        #[inline(always)]
        #[allow(missing_docs)]
        pub unsafe fn new() -> Self {
            __rtic_internal_async_task1_Context {
                __rtic_internal_p: ::core::marker::PhantomData,
                shared: async_task1::SharedResources::new(),
            }
        }
    }
    /// Spawns the task directly
    #[allow(non_snake_case)]
    #[doc(hidden)]
    pub fn __rtic_internal_async_task1_spawn(_0: u16) -> Result<(), u16> {
        unsafe {
            let exec = rtic::export::executor::AsyncTaskExecutor::from_ptr_2_args(
                async_task1,
                &__rtic_internal_async_task1_EXEC,
            );
            if exec.try_allocate() {
                exec.spawn(async_task1(unsafe { async_task1::Context::new() }, _0));
                rtic::export::pend(stm32f3xx_hal_v2::pac::interrupt::TIM4);
                Ok(())
            } else {
                Err(_0)
            }
        }
    }
    #[allow(non_snake_case)]
    ///Software task
    pub mod async_task1 {
        #[doc(inline)]
        pub use super::__rtic_internal_async_task1SharedResources as SharedResources;
        #[doc(inline)]
        pub use super::__rtic_internal_async_task1_Context as Context;
        #[doc(inline)]
        pub use super::__rtic_internal_async_task1_spawn as spawn;
    }
    #[allow(non_snake_case)]
    async fn async_task1<'a>(mut cx: async_task1::Context<'a>, seed: u16) {
        use rtic::Mutex as _;
        use rtic::mutex::prelude::*;
        let start = DWT::get_cycle_count();
        unsafe {
            input1.xs = readSensor(seed * 17);
        }
        let end = DWT::get_cycle_count();
        let cycles = end.wrapping_sub(start);
        ::cortex_m_semihosting::export::hstdout_fmt(
            format_args!("async_task1 execution time: {0}\n", cycles),
        );
    }
    #[allow(non_camel_case_types)]
    #[allow(non_upper_case_globals)]
    #[doc(hidden)]
    #[link_section = ".uninit.rtic0"]
    static __rtic_internal_shared_resource_a: rtic::RacyCell<
        core::mem::MaybeUninit<u8>,
    > = rtic::RacyCell::new(core::mem::MaybeUninit::uninit());
    impl<'a> rtic::Mutex for shared_resources::a_that_needs_to_be_locked<'a> {
        type T = u8;
        #[inline(always)]
        fn lock<RTIC_INTERNAL_R>(
            &mut self,
            f: impl FnOnce(&mut u8) -> RTIC_INTERNAL_R,
        ) -> RTIC_INTERNAL_R {
            /// Priority ceiling
            const CEILING: u8 = 1u8;
            unsafe {
                rtic::export::lock(
                    __rtic_internal_shared_resource_a.get_mut() as *mut _,
                    CEILING,
                    stm32f3xx_hal_v2::pac::NVIC_PRIO_BITS,
                    f,
                )
            }
        }
    }
    mod shared_resources {
        #[doc(hidden)]
        #[allow(non_camel_case_types)]
        pub struct a_that_needs_to_be_locked<'a> {
            __rtic_internal_p: ::core::marker::PhantomData<&'a ()>,
        }
        impl<'a> a_that_needs_to_be_locked<'a> {
            #[inline(always)]
            pub unsafe fn new() -> Self {
                a_that_needs_to_be_locked {
                    __rtic_internal_p: ::core::marker::PhantomData,
                }
            }
        }
    }
    #[allow(non_upper_case_globals)]
    static __rtic_internal_async_task1_EXEC: rtic::export::executor::AsyncTaskExecutorPtr = rtic::export::executor::AsyncTaskExecutorPtr::new();
    #[allow(non_snake_case)]
    ///Interrupt handler to dispatch async tasks at priority 1
    #[no_mangle]
    unsafe fn TIM4() {
        /// The priority of this interrupt handler
        const PRIORITY: u8 = 1u8;
        rtic::export::run(
            PRIORITY,
            || {
                let exec = rtic::export::executor::AsyncTaskExecutor::from_ptr_2_args(
                    async_task1,
                    &__rtic_internal_async_task1_EXEC,
                );
                exec.poll(|| {
                    let exec = rtic::export::executor::AsyncTaskExecutor::from_ptr_2_args(
                        async_task1,
                        &__rtic_internal_async_task1_EXEC,
                    );
                    exec.set_pending();
                    rtic::export::pend(stm32f3xx_hal_v2::pac::interrupt::TIM4);
                });
            },
        );
    }
    #[doc(hidden)]
    #[no_mangle]
    unsafe extern "C" fn main() -> ! {
        rtic::export::assert_send::<u8>();
        rtic::export::assert_send::<u16>();
        rtic::export::interrupt::disable();
        let mut core: rtic::export::Peripherals = rtic::export::Peripherals::steal()
            .into();
        let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::TIM2;
        let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::TIM3;
        let _ = you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::TIM4;
        const _: () = if (1 << stm32f3xx_hal_v2::pac::NVIC_PRIO_BITS) < 1u8 as usize {
            {
                ::core::panicking::panic_fmt(
                    format_args!(
                        "Maximum priority used by interrupt vector \'TIM4\' is more than supported by hardware",
                    ),
                );
            };
        };
        core.NVIC
            .set_priority(
                you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::TIM4,
                rtic::export::cortex_logical2hw(
                    1u8,
                    stm32f3xx_hal_v2::pac::NVIC_PRIO_BITS,
                ),
            );
        rtic::export::NVIC::unmask(
            you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::interrupt::TIM4,
        );
        #[inline(never)]
        fn __rtic_init_resources<F>(f: F)
        where
            F: FnOnce(),
        {
            f();
        }
        let mut executors_size = 0;
        let executor = ::core::mem::ManuallyDrop::new(
            rtic::export::executor::AsyncTaskExecutor::new_2_args(async_task1),
        );
        executors_size += ::core::mem::size_of_val(&executor);
        __rtic_internal_async_task1_EXEC.set_in_main(&executor);
        extern "C" {
            pub static _stack_start: u32;
            pub static __ebss: u32;
        }
        let stack_start = &_stack_start as *const _ as u32;
        let ebss = &__ebss as *const _ as u32;
        if stack_start > ebss {
            if rtic::export::msp::read() <= ebss {
                {
                    ::core::panicking::panic_fmt(
                        format_args!("Stack overflow after allocating executors"),
                    );
                };
            }
        }
        __rtic_init_resources(|| {
            let (shared_resources, local_resources) = init(
                init::Context::new(core.into(), executors_size),
            );
            __rtic_internal_shared_resource_a
                .get_mut()
                .write(core::mem::MaybeUninit::new(shared_resources.a));
            rtic::export::interrupt::enable();
        });
        idle(idle::Context::new())
    }
}
