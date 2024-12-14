//! examples/spawn_loop.rs

#![no_main]
#![no_std]
#![allow(warnings)]
// #![deny(unsafe_code)]

use test_app as _;
use stm32f3xx_hal_v2::pac::Interrupt;
use cortex_m::peripheral::NVIC;

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};

mod checkpoint;
use checkpoint::{checkpoint, restore, delete_pg, delete_all_pg, transcation_log, execution_mode, counter,start_atomic, end_atomic};
use volatile::Volatile;
use checkpoint::my_flash::{unlock, wait_ready, clear_error_flags, erase_page, write_to_flash};

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


    #[cfg(target_arch = "arm")]
    type Numeric = i32;
    #[cfg(not(target_arch = "arm"))]
    type Numeric = i16;

    #[derive(Debug)]
    pub struct Tensor2D<const H: usize, const W: usize> {
        tensor: [[Numeric; W]; H],
    }
    
    impl<const H: usize, const W: usize> Tensor2D<H, W> {
        pub const fn new(tensor: [[Numeric; W]; H]) -> Self {
            Self { tensor }
        }
    
        #[inline(always)]
        pub fn at(&self, rol: usize, col: usize) -> &Numeric {
            &self.tensor[rol][col]
        }
    
        #[inline(always)]
        pub fn mut_at(&mut self, rol: usize, col: usize) -> &mut Numeric {
            &mut self.tensor[rol][col]
        }
    }

    #[derive(Debug)]
    pub struct Tensor1D<const W: usize> {
        tensor: [Numeric; W],
    }
    
    impl<const W: usize> Tensor1D<W> {
        pub const fn new(tensor: [Numeric; W]) -> Self {
            Self { tensor }
        }
    
        #[inline(always)]
        pub fn at(&self, i: usize) -> &Numeric {
            &self.tensor[i]
        }
    
        #[inline(always)]
        pub fn mut_at(&mut self, i: usize) -> &mut Numeric {
            &mut self.tensor[i]
        }
    }
    // Layer 1   input(50, 1)  ----Relu(FC) ---> output(10, 1)
    // Layer 2   input(10, 1)  ----Relu(FC) ---> output(2, 1)

    pub fn fc_layer_impl<const FC_H: usize, const FC_W: usize>(
        param: &Tensor2D<FC_H, FC_W>,
        input1: &Tensor1D<FC_W>,
        output_ref: &mut Tensor1D<FC_H>,
    ) {
        let param_h = FC_H;
        let param_w = FC_W;
        
        for i in 0..param_h {
            let mut sum_i = 0;
            for j in 0..param_w {
                sum_i += *param.at(i, j) * *input.at(j);
            }
            let output_i = if sum_i > 0 { sum_i } else { 0 };
            *output_ref.mut_at(i)  = output_i;
        }
    }

    pub fn fc_layer_impl2<const FC_H: usize, const FC_W: usize>(
        param: &Tensor2D<FC_H, FC_W>,
        input1: &mut Tensor1D<FC_W>,
        output_ref: &mut Tensor1D<FC_H>,
    ) {
        let param_h = FC_H;
        let param_w = FC_W;
        
        for i in 0..param_h {
            let mut sum_i = 0;
            for j in 0..param_w {
                sum_i += *param.at(i, j) * *input.at(j);
            }
            let output_i = if sum_i > 0 { sum_i } else { 0 };
            *output_ref.mut_at(i) = output_i;
        }
    }
        static PARAM_1: Tensor2D<10, 50> = Tensor2D::new([
            [
                7, 0, 2, 5, 4, 4, 5, 7, 9, 2, 9, 4, 9, 3, 0, 8, 4, 0, 2, 9, 3, 8, 1, 6, 6, 6, 5, 3, 3, 2,
                4, 0, 6, 9, 3, 7, 6, 3, 4, 9, 2, 5, 0, 5, 7, 3, 5, 8, 7, 5,
            ],
            [
                8, 0, 6, 0, 3, 6, 0, 6, 0, 0, 6, 3, 3, 0, 0, 0, 5, 4, 5, 9, 8, 4, 5, 8, 8, 5, 5, 9, 1, 7,
                0, 3, 8, 8, 5, 9, 5, 5, 2, 4, 2, 7, 1, 7, 2, 5, 0, 7, 6, 8,
            ],
            [
                2, 0, 6, 9, 4, 9, 8, 7, 0, 6, 4, 8, 1, 5, 5, 3, 6, 8, 4, 8, 8, 4, 7, 8, 4, 2, 4, 8, 0, 7,
                0, 7, 5, 3, 9, 7, 1, 6, 2, 1, 5, 8, 5, 9, 1, 8, 7, 5, 8, 9,
            ],
            [
                9, 1, 9, 7, 4, 1, 8, 3, 2, 5, 3, 9, 2, 8, 3, 1, 8, 8, 1, 4, 1, 3, 2, 4, 0, 5, 9, 5, 3, 9,
                2, 9, 1, 9, 5, 0, 2, 7, 0, 7, 3, 9, 1, 4, 6, 0, 2, 4, 6, 7,
            ],
            [
                4, 9, 0, 4, 7, 8, 3, 4, 4, 2, 2, 0, 5, 7, 0, 2, 7, 2, 3, 5, 0, 3, 2, 0, 3, 0, 4, 8, 1, 9,
                8, 2, 4, 5, 3, 1, 8, 0, 7, 1, 8, 1, 9, 1, 6, 8, 9, 3, 8, 5,
            ],
            [
                4, 4, 0, 3, 5, 7, 1, 9, 2, 2, 6, 6, 5, 0, 6, 5, 0, 3, 0, 9, 2, 6, 0, 0, 6, 6, 2, 5, 4, 8,
                7, 9, 4, 5, 6, 4, 8, 9, 3, 6, 3, 4, 3, 4, 4, 4, 6, 8, 6, 1,
            ],
            [
                5, 7, 8, 4, 6, 2, 0, 7, 9, 1, 3, 6, 0, 6, 8, 3, 4, 8, 9, 1, 9, 0, 3, 4, 6, 6, 7, 4, 5, 1,
                6, 0, 9, 9, 8, 6, 5, 5, 4, 8, 6, 4, 5, 9, 6, 7, 9, 8, 7, 8,
            ],
            [
                5, 0, 8, 2, 6, 3, 0, 1, 9, 9, 4, 9, 6, 0, 6, 6, 5, 8, 3, 4, 5, 5, 7, 9, 0, 8, 2, 8, 9, 4,
                0, 1, 7, 6, 7, 8, 8, 7, 7, 9, 1, 4, 9, 7, 2, 9, 0, 7, 8, 7,
            ],
            [
                3, 0, 0, 1, 0, 4, 7, 2, 9, 5, 6, 8, 6, 4, 3, 6, 2, 1, 5, 4, 5, 1, 4, 8, 6, 3, 5, 8, 0, 8,
                0, 3, 0, 1, 9, 0, 9, 8, 0, 9, 0, 5, 2, 8, 1, 6, 1, 9, 5, 9,
            ],
            [
                3, 7, 8, 5, 9, 8, 7, 4, 6, 9, 9, 1, 4, 1, 6, 2, 3, 4, 8, 9, 8, 0, 5, 6, 5, 3, 8, 2, 1, 4,
                3, 1, 6, 9, 5, 9, 1, 1, 9, 3, 0, 9, 6, 3, 3, 0, 8, 5, 6, 6,
            ],
        ]);
        
        static PARAM_2: Tensor2D<2, 10> = Tensor2D::new([
            [5, 7, 5, 9, 9, 4, 9, 0, 1, 4],
            [2, 9, 2, 3, 2, 2, 8, 0, 8, 4],
        ]);

    const BENCH_ITER: usize = 100;

    // pub fn task_dnn_inference() {
    //     hprintln!("Benchmarking DNN Inference...");
    //     for _ in 0..BENCH_ITER {
    //         dnn_inference();
    //     }
    //     hprintln!("Benchmark complete.");
    // }
    
    // fn dnn_inference() {
    //     let input = Tensor1D::<50>::new([1; 50]);
    //     let mut ob1 = Tensor1D::<10>::new([0; 10]);
    //     let mut ob2 = Tensor1D::<2>::new([0; 2]);
    
    //     fc_layer_impl(&PARAM_1, &input, &mut ob1);
    //     fc_layer_impl2(&PARAM_2, &mut ob1, &mut ob2);
    
    //     hprintln!("Result: {}, {}", ob2.at(0), ob2.at(1));
    // }
        

    #[shared]
    struct Shared {
        a: u16
    }

    #[local]
    struct Local {}

    static input : Tensor1D<50> = Tensor1D::<50>::new([1; 50]);
    static mut ob1 :Tensor1D::<10> = Tensor1D::<10>::new([0; 10]);
    static mut ob2 : Tensor1D::<2> = Tensor1D::<2>::new([0; 2]);
       
    

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {

        let mut cp = cx.core;
        cp.DWT.enable_cycle_counter();
        hprintln!("init");
        //initialization();

        hprintln!("Benchmarking DNN Inference...");
        for _ in 0..BENCH_ITER {
           // dnn_inference();
           //let input = Tensor1D::<50>::new([1; 50]);
           //let mut ob1 = Tensor1D::<10>::new([0; 10]);
           //let mut ob2 = Tensor1D::<2>::new([0; 2]);
       
           //fc_layer_impl(&PARAM_1, &input, &mut ob1);
           //fc_layer_impl2(&PARAM_2, &mut ob1, &mut ob2);
       
           unsafe{async_task1::spawn(&PARAM_1, &mut ob1).ok();}
           unsafe {async_task2::spawn(&PARAM_2, &mut ob1, &mut ob2);}
           async_task3::spawn();
           //unsafe {  async_task2::spawn(&PARAM_1, &mut ob1).ok();  ;}
            
            //fc_layer_impl2(&PARAM_2, &mut ob1, &mut ob2);}

        }
       // hprintln!("Benchmark complete.");

       // let mut ob2 = Tensor1D::<2>::new([0; 2]);
    
        //fc_layer_impl(&PARAM_1, &input, &mut ob1);
        // fc_layer_impl2(&PARAM_2, &mut ob1, &mut ob2);

        (Shared {a: 12}, Local {})
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            hprintln!("idle");
            debug::exit(debug::EXIT_SUCCESS);
        }
    }
    
    #[task(priority = 4, shared = [a] )]
    async fn async_task1(mut cx: async_task1::Context, param: &'static Tensor2D<10, 50>, output_ref: &'static mut Tensor1D<10>) {

        let param_h = 10;
        let param_w = 50;
        
        for i in 0..param_h {
            let mut sum_i = 0;
            for j in 0..param_w {
                sum_i += *param.at(i, j) * *input.at(j);
            }
            let output_i = if sum_i > 0 { sum_i } else { 0 };
            //hprintln!("output_i   {}", output_i);

            let start = DWT::get_cycle_count();
            //start_atomic();
            *output_ref.mut_at(i)  = output_i;
           // end_atomic();
            let end = DWT::get_cycle_count();
            let cycles = end.wrapping_sub(start);

            hprintln!("dnn async_task1 execution time: {}", cycles);
        }

    }

    #[task(priority = 3, shared = [a] )]
    async fn async_task2(mut cx: async_task2::Context,param: &'static Tensor2D<2, 10>, input1: &'static mut Tensor1D<10>, output_ref: &'static mut Tensor1D<2>) {
        // fc_layer_impl2(&PARAM_2, &mut ob1, &mut ob2);

        let param_h = 2;
        let param_w = 10;
        
        for i in 0..param_h {
            let mut sum_i = 0;
            for j in 0..param_w {
                sum_i += *param.at(i, j) * *input1.at(j);
            }
            let output_i = if sum_i > 0 { sum_i } else { 0 };

            let start = DWT::get_cycle_count();
            //start_atomic();
            *output_ref.mut_at(i)  = output_i;
            //end_atomic();
            let end = DWT::get_cycle_count();
            let cycles = end.wrapping_sub(start);

            hprintln!("dnn async_task2 execution time: {}", cycles);
        }

    }

    #[task(priority = 2, shared = [a] )]
    async fn async_task3(mut cx: async_task3::Context){
        unsafe{ hprintln!("Result: {}, {}", ob2.at(0), ob2.at(1));}
        
    }
}
