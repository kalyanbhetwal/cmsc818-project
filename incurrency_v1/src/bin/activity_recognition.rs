//! examples/spawn_loop.rs

#![no_main]
#![no_std]
#![allow(warnings)]
// #![deny(unsafe_code)]
//#![deny(missing_docs)]

use test_app as _;
use stm32f3xx_hal_v2::pac::Interrupt;
use cortex_m::peripheral::NVIC;

use cortex_m_semihosting::{debug, hprintln};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};

mod checkpoint;
use checkpoint::{checkpoint, restore, delete_pg, delete_all_pg, transcation_log, execution_mode, counter,start_atomic, end_atomic};
use volatile::Volatile;
use checkpoint::my_flash::{unlock, wait_ready, clear_error_flags, erase_page, write_to_flash};

// #[macro_use]
// mod intermittent;


/*
#define TASK_CHECKPOINT(...)

#include "ftest_util.h"
void mspconsole_init();
 */

#[rtic::app(device = stm32f3xx_hal_v2::pac, dispatchers = [TIM2, TIM3, TIM4])]
mod app {
    
    use cortex_m::peripheral::DWT;

    use core::arch::asm;
    use core::mem;
    use core::ptr;
    use cortex_m::asm::{nop, self};
    use crate::checkpoint::end_atomic;
    use crate::checkpoint::start_atomic;
    use crate::checkpoint::save_variables;
    use crate::checkpoint::{self, delete_all_pg, restore, checkpoint,c_checkpoint, erase_all};
    use crate::checkpoint::my_flash::{unlock, wait_ready, clear_error_flags, erase_page, write_to_flash};
    use cortex_m::peripheral::syst::SystClkSource;
    use cortex_m_rt:: exception;

    use stm32f3xx_hal_v2::{pac::{self, NVIC},pac::Peripherals, pac::FLASH, pac::Interrupt, gpio::{gpioa::PA0,Input, PullUp}};
    use volatile::Volatile;
    use cortex_m_semihosting::{debug, hprintln};
    use crate::checkpoint::initialization;


    // Number of samples to discard before recording training set
    const NUM_WARMUP_SAMPLES:u16 = 3;
    const ACCEL_WINDOW_SIZE:usize = 3;
    const  MODEL_SIZE:usize = 16;
    const SAMPLE_NOISE_FLOOR:u8 = 10; // TODO: made up value

    // Number of classifications to complete in one experiment
    const SAMPLES_TO_COLLECT:u16 = 128;

    #[macro_export]
    macro_rules! nv {
        ($name:ident : $ty:ty = $expr:expr) => {
        unsafe {
        // #[link_section = ".fram_section"]
            static mut $name: Option<$ty> = None;

            let used = $name.is_some();
            if used {
            None
            } else {
            $name = Some($expr);
            $name.as_mut()

            }
        }
        };
    }

    #[macro_export]
    macro_rules! big_nv {
        ($name:ident : $ty:ty = $expr:expr) => {
        unsafe {
        // #[link_section = ".fram_section"]
            static mut $name:$ty = $expr;
            & mut $name

            }
        };
    }

    struct threeAxis_t_8{
        x:u8,
        y:u8,
        z:u8,
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
        MODE_RECOGNIZE = 0, // default
    }
    
    struct stats_t {
        totalCount:u16,
        movingCount:u16,
        stationaryCount: u16,
    }
    struct  input{
        xs: u8,
        ys: u8,
        zs : u8
    }
    static  mut input1: input = input{xs:0, ys:0 , zs:0};
    
    /* Globals */
    #[no_mangle]
    pub static IO_NAME:fn(u16) ->u8 = readSensor;
    
    //simulated input function
    #[no_mangle]
    fn readSensor(input:u16) -> u8 
    {
        return (input % 85) as u8;
    }
    /*Will need more paras to account for no globals*/
    fn accel_sample(nv_seed:& mut u16) -> threeAxis_t_8
    { 
        let mut seed:u16 = *nv_seed;
    
       // async_task1(seed);
        async_task1::spawn(seed);
    
    
        let xs =   unsafe{input1.xs}; //readSensor(seed*17);
        
        let tmp:u32 = (seed as u32 * 17 * 17 *17 )% 85;
    
        let ys = tmp as u8;//readSensor(seed*17*17);
        let tmp:u32 = (seed as u32 * 17 * 17 *17)%85;
        //hprintln!("the tmp val for zs {}", tmp);
    
        let zs =  tmp as u8; //readSensor(seed*17*17*17 as u16);
        let result: threeAxis_t_8 = threeAxis_t_8{x:xs, y:ys, z:zs};
        
        seed = seed +1;
        *nv_seed = seed;
        return result;
    }
    
    
    //#define accel_sample ACCEL_singleSample
    
    fn acquire_window(window:&mut accelWindow, seed:& mut u16) -> ()
    {
        //accelReading sample;
        let mut samplesInWindow:usize = 0;
    
        //TASK_CHECKPOINT();
    
        while samplesInWindow < ACCEL_WINDOW_SIZE {
            let sample: accelReading = accel_sample(seed);
            //LOG("acquire: sample %u %u %u\r\n", sample.x, sample.y, sample.z);
    
            window[samplesInWindow] = sample;
        samplesInWindow += 1;
        }
    }
    
    fn transform(window:&mut accelWindow) -> ()
    {
    
        //LOG("transform\r\n");
    
        for i in 0..ACCEL_WINDOW_SIZE {
            let sample:&mut accelReading = &mut window[i];
    
            if sample.x < SAMPLE_NOISE_FLOOR ||
                sample.y < SAMPLE_NOISE_FLOOR ||
                sample.z < SAMPLE_NOISE_FLOOR {
    
                //LOG("transform: sample %u %u %u\r\n",
                //    sample->x, sample->y, sample->z);
    
                (*sample).x = if sample.x > SAMPLE_NOISE_FLOOR {sample.x} else {0};
                (*sample).y = if sample.y > SAMPLE_NOISE_FLOOR {sample.y} else {0};
                (*sample).z = if sample.z > SAMPLE_NOISE_FLOOR {sample.z} else {0};
            }
        }
    }
    
    fn featurize(features:&mut features_t, aWin:&accelWindow) -> ()
    {
        //TASK_CHECKPOINT();
    
        let mut mean = accelReading {x:0,y:0,z:0};
        let mut stddev = accelReading {x:0,y:0,z:0};
        
        
        for i in 0..ACCEL_WINDOW_SIZE {
            mean.x += aWin[i].x;  // x
            mean.y += aWin[i].y;  // y
            mean.z += aWin[i].z;  // z
        }
        
        mean.x >>= 2;
        mean.y >>= 2;
        mean.z >>= 2;
    
        for i in 0..ACCEL_WINDOW_SIZE {
            stddev.x += if aWin[i].x > mean.x {aWin[i].x - mean.x} else
            {mean.x - aWin[i].x};  // x
            stddev.y += if aWin[i].y > mean.y { aWin[i].y - mean.y } else {
                mean.y - aWin[i].y};  // y
            stddev.z += if aWin[i].z > mean.z { aWin[i].z - mean.z } else
                { mean.z - aWin[i].z};  // z
        }
        
        stddev.x >>= 2;
        stddev.y >>= 2;
        stddev.z >>= 2;
    
        let meanmag:u32 = mean.x as u32*mean.x as u32 + mean.y as u32 *mean.y as u32+
        mean.z as u32*mean.z as u32 ;
        let stddevmag:u32 = stddev.x as u32*stddev.x as u32 + stddev.y as u32*stddev.y as u32
        + stddev.z as u32 *stddev.z as u32;
    
        features.meanmag   =  libm::sqrtf(meanmag as f32) as u16; //1;//unsafe{sqrt16(meanmag)};
        features.stddevmag =   libm::sqrtf(stddevmag as f32) as u16; //2;//unsafe{sqrt16(stddevmag)};
    
        //LOG("featurize: mean %u sd %u\r\n", features->meanmag, features->stddevmag);
    }
    fn classify(features:&features_t, model:&model_t) -> class_t
    {
        let mut move_less_error:i16 = 0;
        let mut stat_less_error:i16 = 0;
        let mut model_features:features_t = features_t{meanmag:0, stddevmag:0};
    
        //TASK_CHECKPOINT();
    
        for i in 0..MODEL_SIZE {
            model_features.meanmag = model.stationary[i].meanmag;
        model_features.stddevmag = model.stationary[i].stddevmag;
    
            let stat_mean_err:i32 = if model_features.meanmag > features.meanmag
            { (model_features.meanmag  - features.meanmag) as i32} else
            { (features.meanmag - model_features.meanmag) as i32};
    
            let stat_sd_err:i32 = if model_features.stddevmag > features.stddevmag
            {(model_features.stddevmag - features.stddevmag) as i32}
            else {(features.stddevmag - model_features.stddevmag) as i32};
    
            model_features.meanmag = model.moving[i].meanmag;
        model_features.stddevmag = model.moving[i].stddevmag;
    
            let move_mean_err:i32 = if model_features.meanmag > features.meanmag
            {(model_features.meanmag - features.meanmag) as i32}
            else  {(features.meanmag - model_features.meanmag) as i32};
    
            let move_sd_err:i32 = if model_features.stddevmag > features.stddevmag
            {(model_features.stddevmag - features.stddevmag) as i32} else
            {(features.stddevmag - model_features.stddevmag) as i32};
    
            if move_mean_err < stat_mean_err {
                move_less_error+=1;
            } else {
                stat_less_error+=1;
            }
    
            if move_sd_err < stat_sd_err {
                move_less_error+=1;
            } else {
                stat_less_error+=1;
            }
        }
    
        let class:class_t = if move_less_error > stat_less_error
        {class_t::CLASS_MOVING} else {class_t::CLASS_STATIONARY };
        //LOG("classify: class %u\r\n", class);
    
        return class;
    }
    
    fn record_stats(stats:&mut stats_t, class:class_t) -> ()
    {
        //TASK_CHECKPOINT();
    
        /* stats->totalCount, stats->movingCount, and stats->stationaryCount have an
         * nv-internal consistency requirement.  This code should be atomic. */
    
        stats.totalCount+=1;
    
        match class {
            class_t::CLASS_MOVING => stats.movingCount+=1,
                
            class_t::CLASS_STATIONARY => stats.stationaryCount+=1,
        }
    
    /*    unsafe{printf(b"stats: s %u\0".as_ptr(),
              stats.stationaryCount as u32);
        printf(b" m %u\0".as_ptr(),
           stats.movingCount as u32);
        printf(b" t %u\r\n\0".as_ptr(),
              stats.totalCount as u32);}*/
    }
    
    fn print_stats(stats:&stats_t) -> ()
    {
        let resultStationaryPct = stats.stationaryCount * 100 / stats.totalCount;
        let resultMovingPct = stats.movingCount * 100 / stats.totalCount;
    
        let sum = stats.stationaryCount + stats.movingCount;
    
        unsafe {
        //if not u32, rust throws errors. 
        //manually guard output
        // output_guard_start();
        hprintln!("stats: s {} ({}) m {} ({}) sum/tot {}/{}: {}",
               stats.stationaryCount as u32, resultStationaryPct as u32,
               stats.movingCount as u32, resultMovingPct as u32,
               stats.totalCount as u32, sum as u32,
                   if sum == stats.totalCount && sum == SAMPLES_TO_COLLECT { 'V'} else {'X'});
        // output_guard_end();
        }
    }
    
    
    fn warmup_sensor(seed:&mut u16) -> ()
    {
        let mut discardedSamplesCount:u16 = 0;
        let mut _sample:accelReading;
    
        //TASK_CHECKPOINT();
    
        //LOG("warmup\r\n");
    
        while discardedSamplesCount < NUM_WARMUP_SAMPLES {
        _sample = accel_sample(seed);
        discardedSamplesCount += 1;
        }
    }
    
    fn train(classModel:&mut [features_t; MODEL_SIZE], seed:& mut u16, id : &idle::Context) -> ()
    {
        let mut sampleWindow:accelWindow= [accelReading{x:0,y:0, z:0},
                     accelReading{x:0,y:0, z:0},accelReading{x:0,y:0, z:0}];
        let mut features:features_t = features_t{meanmag:0, stddevmag:0};
        warmup_sensor(seed);
    
        for i in 0..MODEL_SIZE {
            acquire_window(&mut sampleWindow, seed);
            transform(&mut sampleWindow);
            featurize(&mut features, &sampleWindow);
    
            //TASK_CHECKPOINT();
    
            classModel[i].meanmag = features.meanmag;
        classModel[i].stddevmag = features.stddevmag;
        }
    
    //    unsafe {
    //	printf(b"train: done: mn %l sd %l\r\n\0".as_ptr(), features.meanmag as u32, features.stddevmag as u32);
        //}
        
    }
    
    fn recognize(model:&model_t, seed:&mut u16) -> ()
    {
        let mut stats:stats_t = stats_t{totalCount:0,movingCount:0,stationaryCount:0};
        let mut sampleWindow:accelWindow = [accelReading{x:0,y:0, z:0},
                     accelReading{x:0,y:0, z:0},accelReading{x:0,y:0, z:0}];
        let mut features:features_t = features_t{meanmag:0, stddevmag:0};
        let mut class:class_t;
        
        for _i in 0..SAMPLES_TO_COLLECT {
            acquire_window(&mut sampleWindow, seed);
            transform(&mut sampleWindow);
            featurize(&mut features, &sampleWindow);
            class = classify(&features, model);
            record_stats(&mut stats, class);
        }
    
        print_stats(&stats);
    }
    fn end_of_benchmark() -> (){}
    
    fn count_error(count:&u16) -> ()
    {
        unsafe {
         hprintln!("An error occured during count, count = {}", count);
        }
    }
    
    fn select_mode(prev_pin_state:&mut u8, count: &mut u16) -> u8 
    {
        let mut pin_state:u8 = run_mode_t::MODE_IDLE as u8;
    
        //TASK_CHECKPOINT();
    
        *count = *count + 1;
    
        /* The InK order
         *  rounds:
         *      1,2 = MODE_TRAIN_MOVING
         *      3,4 = MODE_TRAIN_STATIONARY
         *      5,6 = MODE_RECOGNIZE
         *      7   = END OF BENCHMARK
         */
        match *count {
            1|2 => pin_state = run_mode_t::MODE_TRAIN_MOVING as u8,
            3|4 => pin_state = run_mode_t::MODE_TRAIN_STATIONARY as u8,
            5|6  => pin_state = run_mode_t::MODE_RECOGNIZE as u8,
            7 => end_of_benchmark(),
            _ => {
                pin_state = run_mode_t::MODE_IDLE as u8;
                count_error(count)
        },
        }
    
        //loop benchmark
        if *count == 7 {
        return run_mode_t::MODE_IDLE as u8;
        }
    
        //pin_state = GPIO(PORT_AUX, IN) & (BIT(PIN_AUX_1) | BIT(PIN_AUX_2));
    
        // Don't re-launch training after finishing training
        // Vito: could have done this while assigning pin_state. But keep is the same as the original
        if (pin_state == run_mode_t::MODE_TRAIN_STATIONARY as u8  ||
            pin_state == run_mode_t::MODE_TRAIN_MOVING as u8) &&
            pin_state == *prev_pin_state  {
            pin_state = run_mode_t::MODE_IDLE as u8;
        } else {
            *prev_pin_state = pin_state;
        }
    
        //LOG("selectMode: pins %04x\r\n", pin_state);
    
        return pin_state;
    }
    
    #[shared]
    struct Shared {
        a: u8
    }

    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        
        hprintln!("init");
        
        let mut cp = cx.core;
        cp.DWT.enable_cycle_counter();

        //initialization();
        //async_task1::spawn(12).ok();
        //async_task2::spawn().ok();


        (Shared {a: 0,}, Local {})
    }

    #[idle(shared = [a])]
    fn idle(mut id: idle::Context) -> ! {

        let mut prev_pin_state:u8 = run_mode_t::MODE_IDLE as u8;
        let count: &'static mut u16 = big_nv!(COUNT_NV: u16 = 1);
        let model: &'static mut model_t =
        big_nv!(MODEL_NV:model_t = model_t{
            moving:[features_t{meanmag:0,stddevmag:0}, features_t{meanmag:0,stddevmag:0}, features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0}],
            stationary:[features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},features_t{meanmag:0,stddevmag:0},
                features_t{meanmag:0,stddevmag:0}]
                       });
            
        let _v_seed: &'static mut u16 = big_nv!(SEED_NV: u16 = 1);
        loop {
            let mut localSeed = *_v_seed;
            let mode:u8 = select_mode(&mut prev_pin_state, count);
            if mode == 2 {
                train(&mut model.stationary, &mut localSeed, &id);
            } else if mode == 1 {
                train(&mut model.moving, &mut localSeed, &id);
            } else if mode == 0 {
                recognize(&model, &mut localSeed);
            
            } else if mode == 3 && *count == 7{
                //idle, for restarting the bench
                localSeed = 1;
                *count = 1;
                prev_pin_state = run_mode_t::MODE_IDLE as u8;
                unsafe{
              //  gpioTwiddle();
              //  gpioTwiddle();
                }
            }
            *_v_seed = localSeed;
        }
    }
    
    #[task(priority = 1, shared = [a] )]
    async fn async_task1(mut cx: async_task1::Context, seed: u16) {
        //hprintln!("I am getting sensor data");

        let start = DWT::get_cycle_count();
      // start_atomic();
            unsafe {input1.xs = readSensor(seed*17);}
       // end_atomic();
        let end = DWT::get_cycle_count();

        let cycles = end.wrapping_sub(start);

        hprintln!("async_task1 execution time: {}", cycles);

        // cx.shared.a.lock(|a| {
        //       *a = readSensor(seed*17);
        //     });

    }

}
