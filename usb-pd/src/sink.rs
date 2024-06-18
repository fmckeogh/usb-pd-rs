use {
    crate::{
        header::{DataMessageType, Header, SpecificationRevision},
        messages::{
            pdo::{FixedVariableRequestDataObject, PPSRequestDataObject, SourceCapabilities},
            vdo::{
                CertStatVDO, ProductVDO, UFPTypeVDO, VDMCommand, VDMCommandType, VDMHeader,
                VDMHeaderStructured, VDMIdentityHeader, VDMType, VDMVersionMajor, VDMVersionMinor,
            },
            Message,
        },
        DataRole, PowerRole,
    },
    core::future::Future,
    defmt::{debug, warn, Format},
    embassy_time::Instant,
    heapless::Vec,
};

pub trait Driver {
    fn init(&mut self) -> impl Future<Output = ()>;

    fn poll(&mut self, now: Instant) -> impl Future<Output = ()>;

    fn get_pending_message(&mut self) -> Option<Message>;

    fn did_change_protocol(&mut self) -> bool;

    fn send_message(&mut self, header: Header, payload: &[u8]) -> impl Future<Output = ()>;

    fn state(&mut self) -> DriverState;
}

/// Sink events
#[derive(Format)]
pub enum Event {
    /// Power delivery protocol has changed
    ProtocolChanged,
    /// Source capabilities have changed (immediately request power)
    SourceCapabilitiesChanged(SourceCapabilities),
    /// Requested power has been accepted (but not ready yet)
    PowerAccepted,
    /// Requested power has been rejected
    PowerRejected,
    /// Requested power is now ready
    PowerReady,
    /// VDM received
    VDMReceived((VDMHeader, Vec<u32, 7>)),
}

/// Requests made to sink
#[derive(Format)]
pub enum Request {
    RequestPower {
        /// Index of the desired PowerDataObject
        index: usize,
        current: u16,
    },
    RequestPPS {
        /// Index of the desired PowerDataObject
        index: usize,
        /// Requested voltage (in mV)
        voltage: u16,
        /// Requested maximum current (in mA)
        current: u16,
    },
    REQDiscoverIdentity,
    ACKDiscoverIdentity {
        identity: VDMIdentityHeader,
        cert_stat: CertStatVDO,
        product: ProductVDO,
        product_type_ufp: UFPTypeVDO,
        // Does not exist yet...        product_type_dfp: Option<DFP>,
    },
    REQDiscoverSVIDS,
}

/// Driver state
#[derive(PartialEq, Clone, Copy)]
pub enum DriverState {
    /// VBUS is present, monitoring for activity on CC1/CC2
    Usb20,
    /// Activity on CC1/CC2 has been detected, waiting for first USB PD message
    UsbPdWait,
    /// Successful USB PD communication established
    UsbPd,
    /// Wait period after a failure
    UsbRetryWait,
}

pub struct Sink<DRIVER> {
    driver: DRIVER,

    protocol: Protocol,

    /// Requested voltage (in mV)
    requested_voltage: u16,

    /// Requested maximum current (in mA)
    requested_max_current: u16,

    /// Active voltage (in mV)
    active_voltage: u16,

    /// Active maximum current (in mA)
    active_max_current: u16,

    /// Specification revision (of last message)
    spec_rev: u8,
}

impl<DRIVER: Driver> Sink<DRIVER> {
    pub fn new(driver: DRIVER) -> Self {
        Self {
            driver,
            protocol: Protocol::Usb20,
            requested_voltage: 0,
            requested_max_current: 0,
            active_voltage: 5000,
            active_max_current: 900,
            spec_rev: 1,
        }
    }

    pub async fn init(&mut self) {
        self.driver.init().await;
        self.update_protocol();
    }

    /// Call continously until `None` is returned.
    pub async fn poll(&mut self, now: Instant) -> Option<Event> {
        // poll inner driver
        self.driver.poll(now).await;

        if let Some(message) = self.driver.get_pending_message() {
            return self.handle_msg(message);
        };

        if self.driver.did_change_protocol() {
            if self.update_protocol() {
                Some(Event::ProtocolChanged)
            } else {
                None
            }
        } else {
            None
        }
    }

    pub async fn request(&mut self, request: Request) {
        match request {
            Request::RequestPower { index, current } => self.request_power(current, index).await,

            Request::RequestPPS {
                index,
                voltage,
                current,
            } => {
                // Payload is 4 bytes
                let mut payload = [0; 4];
                // Add one to index to account for array offsets starting at 0 and obj_pos
                // starting at 1...
                let obj_pos = index + 1;
                assert!(obj_pos > 0b0000 && obj_pos <= 0b1110);

                // Create PPS request data object
                let pps = PPSRequestDataObject(0)
                    .with_object_position(obj_pos as u8)
                    .with_raw_operating_current(current / 50) // Convert current from millis to 50ma units
                    .with_raw_output_voltage(voltage / 20) // Convert voltage from millis to 20mv units
                    .with_capability_mismatch(false)
                    .with_epr_mode_capable(false)
                    .with_usb_communications_capable(true);
                pps.to_bytes(&mut payload[0..4]);

                // Create header
                let header = Header(0)
                    .with_message_type_raw(DataMessageType::Request as u8)
                    .with_num_objects(1)
                    .with_spec_revision(SpecificationRevision::from(self.spec_rev))
                    .with_port_power_role(PowerRole::Sink);

                // Send request message
                self.driver.send_message(header, &payload).await
            }

            Request::ACKDiscoverIdentity {
                identity,
                cert_stat,
                product,
                product_type_ufp,
                //product_type_dfp,
            } => {
                debug!("ACKDiscoverIdentity");
                // The size of this array will actually change depending on data...
                // TODO: Fix this!
                let mut payload = [0; 5 * 4];
                let header = Header(0)
                    .with_message_type_raw(DataMessageType::VendorDefined as u8)
                    .with_num_objects(5) // 5 VDOs, vdm header, id header, cert, product, UFP product type
                    .with_port_data_role(DataRole::Ufp)
                    .with_port_power_role(PowerRole::Sink)
                    .with_spec_revision(SpecificationRevision::from(self.spec_rev));

                let vdm_header_vdo = VDMHeader::Structured(
                    VDMHeaderStructured(0)
                        .with_command(VDMCommand::DiscoverIdentity)
                        .with_command_type(VDMCommandType::ResponderACK)
                        .with_object_position(0) // 0 Must be used for descover identity
                        .with_standard_or_vid(0xff00) // PD SID must be used with descover identity
                        //.with_vdm_type(VDMType::Structured)
                        .with_vdm_version_major(VDMVersionMajor::Version2x.into())
                        .with_vdm_version_minor(VDMVersionMinor::Version20.into()),
                );
                vdm_header_vdo.to_bytes(&mut payload[0..4]);
                identity.to_bytes(&mut payload[4..8]);
                cert_stat.to_bytes(&mut payload[8..12]);
                product.to_bytes(&mut payload[12..16]);
                product_type_ufp.to_bytes(&mut payload[16..20]);
                // if let Some(product_type_dfp) = product_type_dfp {
                //     // 20..24 are padding bytes
                //     product_type_dfp.to_bytes(&mut payload[24..32]);
                // }
                debug!("Sending VDM {:x}", payload);
                self.driver.send_message(header, &payload).await;
                debug!("Sent VDM");
            }
            Request::REQDiscoverSVIDS => {
                debug!("REQDiscoverSVIDS");
                let mut payload = [0; 4];
                let header = Header(0)
                    .with_message_type_raw(DataMessageType::VendorDefined as u8)
                    .with_num_objects(1) // 1 VDO, vdm header
                    .with_port_data_role(DataRole::Ufp)
                    .with_port_power_role(PowerRole::Sink)
                    .with_spec_revision(SpecificationRevision::from(self.spec_rev));

                let vdm_header_vdo = VDMHeader::Structured(
                    VDMHeaderStructured(0)
                        .with_command(VDMCommand::DiscoverSVIDS)
                        .with_command_type(VDMCommandType::InitiatorREQ)
                        .with_object_position(0) // 0 Must be used for discover SVIDS
                        .with_standard_or_vid(0xff00) // PD SID must be used with discover SVIDS
                        .with_vdm_type(VDMType::Structured)
                        .with_vdm_version_major(VDMVersionMajor::Version10.into())
                        .with_vdm_version_minor(VDMVersionMinor::Version20.into()),
                );
                vdm_header_vdo.to_bytes(&mut payload[0..4]);
                debug!("Sending VDM {:x}", payload);
                self.driver.send_message(header, &payload).await;
                debug!("Sent VDM");
            }
            Request::REQDiscoverIdentity => {
                debug!("REQDiscoverIdentity");
                let mut payload = [0; 4];
                let header = Header(0)
                    .with_message_type_raw(DataMessageType::VendorDefined as u8)
                    .with_num_objects(1) // 1 VDO, vdm header
                    .with_port_data_role(DataRole::Ufp)
                    .with_port_power_role(PowerRole::Sink)
                    .with_spec_revision(SpecificationRevision::from(self.spec_rev));

                let vdm_header_vdo = VDMHeader::Structured(
                    VDMHeaderStructured(0)
                        .with_command(VDMCommand::DiscoverIdentity)
                        .with_command_type(VDMCommandType::InitiatorREQ)
                        .with_object_position(0) // 0 Must be used for descover identity
                        .with_standard_or_vid(0xff00) // PD SID must be used with descover identity
                        .with_vdm_type(VDMType::Structured)
                        .with_vdm_version_major(VDMVersionMajor::Version10.into())
                        .with_vdm_version_minor(VDMVersionMinor::Version20.into()),
                );
                vdm_header_vdo.to_bytes(&mut payload[0..4]);
                debug!("Sending VDM {:x}", payload);
                self.driver.send_message(header, &payload).await;
                debug!("Sent VDM");
            }
        }
    }

    fn update_protocol(&mut self) -> bool {
        let old_protocol = self.protocol;

        if self.driver.state() == DriverState::UsbPd {
            self.protocol = Protocol::UsbPd;
        } else {
            self.protocol = Protocol::Usb20;
            self.active_voltage = 5000;
            self.active_max_current = 900;
        }

        self.protocol != old_protocol
    }

    fn handle_msg(&mut self, message: Message) -> Option<Event> {
        match message {
            Message::Accept => Some(Event::PowerAccepted),
            Message::Reject => {
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                Some(Event::PowerRejected)
            }
            Message::Ready => {
                self.active_voltage = self.requested_voltage;
                self.active_max_current = self.requested_max_current;
                self.requested_voltage = 0;
                self.requested_max_current = 0;
                Some(Event::PowerReady)
            }
            Message::SourceCapabilities(caps) => Some(Event::SourceCapabilitiesChanged(caps)),
            Message::VendorDefined((hdr, data)) => {
                match hdr {
                    VDMHeader::Structured(hdr) => {
                        warn!(
                            "UNHANDLED: Structured VDM! CMD_TYPE: {:?}, CMD:
                        {:?}",
                            hdr.command_type(),
                            hdr.command()
                        );
                    }
                    VDMHeader::Unstructured(hdr) => {
                        warn!(
                            "UNHANDLED: Unstructured VDM! SVID: {:x}, DATA:
                        {:x}",
                            hdr.standard_or_vid(),
                            hdr.data()
                        );
                    }
                }
                Some(Event::VDMReceived((hdr, data)))
            }
            Message::SoftReset => {
                warn!("UNHANDLED: Soft RESET request.");
                None
            }
            Message::Unknown => unimplemented!(),
        }
    }

    async fn request_power(&mut self, max_current: u16, index: usize) {
        // Create 'request' message
        let mut payload = [0; 4];

        self.set_request_payload_fixed(&mut payload, index as u8, max_current);

        let header = Header(0)
            .with_message_type_raw(DataMessageType::Request as u8)
            .with_num_objects(1)
            .with_spec_revision(SpecificationRevision::from(self.spec_rev))
            .with_port_power_role(PowerRole::Sink);

        // Send message
        self.driver.send_message(header, &payload).await;
    }

    fn set_request_payload_fixed(&mut self, payload: &mut [u8], obj_pos: u8, mut current: u16) {
        current = (current + 5) / 10;

        if current > 0x3ff {
            current = 0x3ff;
        }

        let obj_pos = obj_pos + 1;
        assert!(obj_pos > 0b0000 && obj_pos <= 0b1110);

        FixedVariableRequestDataObject(0)
            .with_raw_operating_current(current)
            .with_raw_max_operating_current(current)
            .with_object_position(obj_pos)
            .with_no_usb_suspend(true)
            .with_usb_communications_capable(true)
            .to_bytes(payload);
    }
}

/// Power supply type
#[derive(Clone, Copy, PartialEq)]
pub enum SupplyType {
    /// Fixed supply (Vmin = Vmax)
    Fixed = 0,
    /// Battery
    Battery = 1,
    /// Variable supply (non-battery)
    Variable = 2,
    /// Programmable power supply
    Pps = 3,
}

/// Power deliver protocol
#[derive(Format, PartialEq, Clone, Copy)]
enum Protocol {
    /// No USB PD communication (5V only)
    Usb20,
    /// USB PD communication
    UsbPd,
}
