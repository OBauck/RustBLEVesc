use defmt::{debug, error, info, warn};
use embassy_futures::join::join;
use embassy_futures::select::select;
use rand_core::{CryptoRng, RngCore};
use trouble_host::prelude::*;

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 4; // Signal + att

const NUS_BUFFER_SIZE_TX: usize = 20;
const NUS_BUFFER_SIZE_RX: usize = 20;

// GATT Server definition
#[gatt_server]
struct Server {
    nordic_uart_service: NordicUartService,
}

const NUS_SERVICE_UUID: [u8; 16] = [
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
];

// Nordic uart service
#[gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
pub(crate) struct NordicUartService {
    #[characteristic(
        uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
        write,
        write_without_response
    )]
    pub(crate) rx: heapless::Vec<u8, NUS_BUFFER_SIZE_RX>,

    #[characteristic(uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", notify)]
    pub(crate) tx: heapless::Vec<u8, NUS_BUFFER_SIZE_TX>,
}

/// Run the BLE stack.
pub async fn run<C, RNG, W, R>(
    controller: C,
    random_generator: &mut RNG,
    mut uart_writer: W,
    mut uart_reader: R,
) where
    C: Controller,
    RNG: RngCore + CryptoRng,
    W: embedded_io_async::Write,
    R: embedded_io_async::BufRead,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x08, 0x05, 0xe4, 0xff]);
    info!("Our address = {}", defmt::Debug2Format(&address));

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .set_random_generator_seed(random_generator);

    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::human_interface_device::GENERIC_HUMAN_INTERFACE_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("VESC Rust", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn, &mut uart_writer);
                    let b = custom_task(&server, &conn, &stack, &mut uart_reader);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                    info!("Connection dropped");
                }
                Err(e) => {
                    let e = defmt::Debug2Format(&e);
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
///
/// ## Alternative
///
/// If you didn't require this to be generic for your application, you could statically spawn this with i.e.
///
/// ```rust,ignore
///
/// #[embassy_executor::task]
/// async fn ble_task(mut runner: Runner<'static, SoftdeviceController<'static>>) {
///     runner.run().await;
/// }
///
/// spawner.must_spawn(ble_task(runner));
/// ```
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task<W: embedded_io_async::Write>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, DefaultPacketPool>,
    uart_writer: &mut W,
) -> Result<(), Error> {
    let nus_rx = &server.nordic_uart_service.rx;
    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => break reason,
            GattConnectionEvent::PairingComplete { security_level, .. } => {
                info!(
                    "[gatt] pairing complete: {:?}",
                    defmt::Debug2Format(&security_level)
                );
            }
            GattConnectionEvent::PairingFailed(err) => {
                error!("[gatt] pairing error: {:?}", defmt::Debug2Format(&err));
            }
            GattConnectionEvent::Gatt { event } => {
                match &event {
                    GattEvent::Read(_event) => {
                        info!("read");
                    }
                    GattEvent::Write(event) => {
                        if event.handle() == nus_rx.handle {
                            info!(
                                "[gatt] Write Event to Level Characteristic: {:?}",
                                event.data()
                            );
                            if let Err(err) = uart_writer.write_all(event.data()).await {
                                warn!("Unable to write to uart: {:?}", defmt::Debug2Format(&err));
                            }
                        }
                    }
                    GattEvent::NotAllowed(event) => {
                        info!(
                            "[gatt] Disallowed GATT request to handle: {:?}",
                            event.handle()
                        );
                    }
                    _ => (),
                }

                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(e) => warn!(
                        "[gatt] error sending response: {:?}",
                        defmt::Debug2Format(&e)
                    ),
                }
            }
            _ => {} // ignore other Gatt Connection Events
        }
    };
    info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let adv_len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids128(&[NUS_SERVICE_UUID]),
        ],
        &mut advertiser_data[..],
    )?;

    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::CompleteLocalName(name.as_bytes())],
        &mut scan_data[..],
    )?;
    info!("adv data set");
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..adv_len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    info!("[adv] connection established");
    Ok(conn)
}

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
async fn custom_task<C, P, R>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    _stack: &Stack<'_, C, P>,
    uart_reader: &mut R,
) where
    C: Controller,
    P: PacketPool,
    R: embedded_io_async::BufRead,
{
    let nus_tx = &server.nordic_uart_service.tx;
    let mut vesc_decoder = vesc::Decoder::default();
    let mut data: heapless::Vec<u8, NUS_BUFFER_SIZE_TX> = heapless::Vec::new();

    // TODO: Flush uart rx buffer?

    loop {
        match uart_reader.fill_buf().await {
            Err(err) => warn!("Unable to read from uart: {:?}", defmt::Debug2Format(&err)),
            Ok(uart_data) => {
                match vesc_decoder.feed(uart_data) {
                    Ok(count) => uart_reader.consume(count),
                    Err(err) => error!("Vesc decoder feed error: {:?}", defmt::Debug2Format(&err)),
                }

                while let Some((reply, raw_data)) = vesc_decoder.next_item() {
                    debug!("Reply: {:?}", defmt::Debug2Format(&reply));
                    debug!("raw data size: {}", raw_data.len());
                    for chunk in raw_data.chunks(NUS_BUFFER_SIZE_TX) {
                        match data.extend_from_slice(chunk) {
                            Err(_) => error!("No space left in data vector"),
                            Ok(()) => {
                                if let Err(err) = nus_tx.notify(conn, &data).await {
                                    error!(
                                        "[custom_task] error notifying connection: {:?}",
                                        defmt::Debug2Format(&err)
                                    );
                                };
                            }
                        }

                        data.clear();
                    }
                }
            }
        }
    }
}
