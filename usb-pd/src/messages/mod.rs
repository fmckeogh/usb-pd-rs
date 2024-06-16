pub mod pdo;
pub mod vdo;

use {
    self::pdo::{
        AVSRequestDataObject, BatteryRequestDataObject, FixedVariableRequestDataObject,
        PPSRequestDataObject, PowerDataObjectType, RawRequestDataObject, Request,
    },
    crate::header::{ControlMessageType, DataMessageType, Header, MessageType},
    byteorder::{ByteOrder, LittleEndian},
    defmt::{trace, warn, Format},
    heapless::Vec,
    pdo::{
        AugmentedPowerDataObject, AugmentedPowerDataObjectRaw, Battery, EPRAdjustableVoltageSupply,
        FixedSupply, PowerDataObject, PowerDataObjectRaw, SPRProgrammablePowerSupply,
        SourceCapabilities, VariableSupply,
    },
    vdo::{VDMHeader, VDMHeaderRaw, VDMHeaderStructured, VDMHeaderUnstructured, VDMType},
};

pub trait PdoState {
    fn pdo_at_object_position(&self, position: u8) -> Option<PowerDataObjectType>;
}

impl PdoState for () {
    fn pdo_at_object_position(&self, _position: u8) -> Option<PowerDataObjectType> {
        None
    }
}

#[derive(Debug, Clone, Format)]
pub enum Message {
    Control(ControlMessageType),
    SourceCapabilities(SourceCapabilities),
    Request(Request),
    VendorDefined((VDMHeader, Vec<u32, 7>)), // TODO: Incomplete
    Unknown,
}

impl Message {
    pub fn parse_with_state<P: PdoState>(header: Header, payload: &[u8], state: &P) -> Self {
        match header.message_type() {
            MessageType::Control(c) => Message::Control(c),
            MessageType::Data(DataMessageType::SourceCapabilities) => {
                Message::SourceCapabilities(SourceCapabilities(
                    payload
                        .chunks_exact(4)
                        .take(header.num_objects())
                        .map(|buf| PowerDataObjectRaw(LittleEndian::read_u32(buf)))
                        .map(|pdo| match pdo.kind() {
                            0b00 => PowerDataObject::FixedSupply(FixedSupply(pdo.0)),
                            0b01 => PowerDataObject::Battery(Battery(pdo.0)),
                            0b10 => PowerDataObject::VariableSupply(VariableSupply(pdo.0)),
                            0b11 => PowerDataObject::AugmentedPowerDataObject({
                                match AugmentedPowerDataObjectRaw(pdo.0).supply() {
                                    0b00 => AugmentedPowerDataObject::SPR(
                                        SPRProgrammablePowerSupply(pdo.0),
                                    ),
                                    0b01 => AugmentedPowerDataObject::EPR(
                                        EPRAdjustableVoltageSupply(pdo.0),
                                    ),
                                    _ => {
                                        warn!("Unknown AugmentedPowerDataObject supply");
                                        AugmentedPowerDataObject::Unknown(pdo.0)
                                    }
                                }
                            }),
                            _ => {
                                warn!("Unknown PowerDataObject kind");
                                PowerDataObject::Unknown(pdo)
                            }
                        })
                        .collect(),
                ))
            }
            MessageType::Data(DataMessageType::Request) => {
                if payload.len() != 4 {
                    return Message::Unknown;
                }
                let raw = RawRequestDataObject(LittleEndian::read_u32(payload));
                if let Some(t) = state.pdo_at_object_position(raw.object_position()) {
                    Message::Request(match t {
                        PowerDataObjectType::FixedSupply => {
                            Request::FixedSupply(FixedVariableRequestDataObject(raw.0))
                        }
                        PowerDataObjectType::Battery => {
                            Request::Battery(BatteryRequestDataObject(raw.0))
                        }
                        PowerDataObjectType::VariableSupply => {
                            Request::VariableSupply(FixedVariableRequestDataObject(raw.0))
                        }
                        PowerDataObjectType::PPS => Request::PPS(PPSRequestDataObject(raw.0)),
                        PowerDataObjectType::AVS => Request::AVS(AVSRequestDataObject(raw.0)),
                    })
                } else {
                    Message::Request(Request::Unknown(raw))
                }
            }
            MessageType::Data(DataMessageType::VendorDefined) => {
                // Keep for now...
                let len = payload.len();
                if len < 4 {
                    return Message::Unknown;
                }
                let num_obj = header.num_objects();
                trace!("VENDOR: {:?}, {:?}, {:x}", len, num_obj, payload);

                let header = {
                    let raw = VDMHeaderRaw(LittleEndian::read_u32(&payload[..4]));
                    match raw.vdm_type() {
                        VDMType::Unstructured => {
                            VDMHeader::Unstructured(VDMHeaderUnstructured(raw.0))
                        }
                        VDMType::Structured => VDMHeader::Structured(VDMHeaderStructured(raw.0)),
                    }
                };

                let data = payload[4..]
                    .chunks_exact(4)
                    .take(7)
                    .map(LittleEndian::read_u32)
                    .collect::<Vec<u32, 7>>();

                trace!("VDM RX: {:?} {:?}", header, data);
                // trace!("HEADER: VDM:: TYPE: {:?}, VERS: {:?}", header.vdm_type(),
                // header.vdm_version()); trace!("HEADER: CMD:: TYPE: {:?}, CMD:
                // {:?}", header.command_type(), header.command());

                // Keep for now...
                // let pkt = payload
                //     .chunks_exact(1)
                //     .take(8)
                //     .map(|i| i[0])
                //     .collect::<Vec<u8, 8>>();

                Message::VendorDefined((header, data))
            }
            MessageType::Data(_) => {
                warn!("unknown message type");
                Message::Unknown
            }
        }
    }

    pub fn parse(header: Header, payload: &[u8]) -> Self {
        Self::parse_with_state(header, payload, &())
    }
}
