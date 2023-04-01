use {
    crate::{
        header::{ControlMessageType, DataMessageType, Header, MessageType},
        pdo::{
            AugmentedPowerDataObject, AugmentedPowerDataObjectRaw, Battery,
            EPRAdjustableVoltageSupply, FixedSupply, PowerDataObject, PowerDataObjectRaw,
            SPRProgrammablePowerSupply, VariableSupply,
        },
    },
    byteorder::{ByteOrder, LittleEndian},
    defmt::{warn, Format},
};

#[derive(Clone, Copy, Format)]
pub enum Message {
    Accept,
    Reject,
    Ready,
    SourceCapabilities([Option<PowerDataObject>; 10]),
    Unknown,
}

impl Message {
    pub fn parse(header: Header, payload: &[u8]) -> Self {
        match header.message_type() {
            MessageType::Control(ControlMessageType::Accept) => Message::Accept,
            MessageType::Control(ControlMessageType::Reject) => Message::Reject,
            MessageType::Control(ControlMessageType::PsRdy) => Message::Ready,
            MessageType::Data(DataMessageType::SourceCapabilities) => Message::SourceCapabilities(
                parse_source_capabilities(header.num_objects() as usize, payload),
            ),
            _ => {
                warn!("unknown message type");
                Message::Unknown
            }
        }
    }
}

fn parse_source_capabilities(num_objects: usize, payload: &[u8]) -> [Option<PowerDataObject>; 10] {
    let mut capabilities = [None; 10];
    payload
        .chunks_exact(4)
        .take(num_objects)
        .map(|buf| PowerDataObjectRaw(LittleEndian::read_u32(buf)))
        .map(|pdo| match pdo.kind() {
            0b00 => PowerDataObject::FixedSupply(FixedSupply(pdo.0)),
            0b01 => PowerDataObject::Battery(Battery(pdo.0)),
            0b10 => PowerDataObject::VariableSupply(VariableSupply(pdo.0)),
            0b11 => PowerDataObject::AugmentedPowerDataObject({
                match AugmentedPowerDataObjectRaw(pdo.0).supply() {
                    0b00 => AugmentedPowerDataObject::SPR(SPRProgrammablePowerSupply(pdo.0)),
                    0b01 => AugmentedPowerDataObject::EPR(EPRAdjustableVoltageSupply(pdo.0)),
                    _ => unreachable!(),
                }
            }),
            _ => unreachable!(),
        })
        .enumerate()
        .for_each(|(i, pdo)| capabilities[i] = Some(pdo));

    capabilities
}
