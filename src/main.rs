#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::buffered_uarte::{self, BufferedUarte};
use embassy_nrf::{bind_interrupts, peripherals, qspi, rng, uarte};
use embassy_time::{Duration, Timer, WithTimeout};
use nrf_sdc::mpsl;
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    UARTE0 => buffered_uarte::InterruptHandler<peripherals::UARTE0>;
    RNG => rng::InterruptHandler<peripherals::RNG>;
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    QSPI => qspi::InterruptHandler<embassy_nrf::peripherals::QSPI>;
});

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mpsl_p =
        mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    static SESSION_MEM: StaticCell<mpsl::SessionMem<1>> = StaticCell::new();
    let mpsl = MPSL.init(unwrap!(mpsl::MultiprotocolServiceLayer::with_timeslots(
        mpsl_p,
        Irqs,
        lfclk_cfg,
        SESSION_MEM.init(mpsl::SessionMem::new())
    )));
    spawner.spawn(mpsl_task(&*mpsl)).unwrap();

    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;

    let mut tx_buffer = [0u8; 4096];
    let mut rx_buffer = [0u8; 4096];

    let mut u = BufferedUarte::new(
        p.UARTE0,
        p.TIMER1,
        p.PPI_CH0,
        p.PPI_CH1,
        p.PPI_GROUP0,
        p.P1_02,
        p.P1_03,
        Irqs,
        config,
        &mut rx_buffer,
        &mut tx_buffer,
    );

    info!("uarte initialized!");

    let mut vesc_tx_buf = [0u8; 16];

    let vesc_tx_size = vesc::encode(vesc::Command::GetValues, &mut vesc_tx_buf).unwrap();

    let mut vesc_decoder = vesc::Decoder::default();
    let mut vesc_rx_buf = [0u8; 512];
    let mut vesc_rx_size = 0usize;

    loop {
        u.write(&vesc_tx_buf[..vesc_tx_size]).await.unwrap();
        let mut response_received = false;

        let read_response = async {
            loop {
                vesc_rx_size = u.read(&mut vesc_rx_buf).await.unwrap();
                vesc_decoder.feed(&vesc_rx_buf[..vesc_rx_size]).unwrap();
                for reply in vesc_decoder.by_ref() {
                    info!("Reply: {:?}", defmt::Debug2Format(&reply));
                    response_received = true;
                }
                if response_received {
                    break;
                }
            }
        };

        if let Err(_) = read_response.with_timeout(Duration::from_millis(100)).await {
            warn!("No response within timeout");
        }
        Timer::after_secs(2).await;
    }
}
