//! Stabilizer UDP message capabilities
//!
//! # Design
//! Packets are always sent in a best-effort fashion, and data may be dropped.
//!
//! ## Frame Header
//! The header consists of the following, all in little-endian.
//!
//! * **Magic word 0x057B** <u16>: a constant to identify Stabilizer messages.
//! * **Format Code** <u8>: a unique ID that indicates the serialization format of each batch of data
//!   in the frame. Refer to [MessageFormat] for further information.
//! * **Batch Size** <u8>: the number of samples in each batch of data.
//! * **Sequence Number** <u32>: an the sequence number of the first batch in the frame.
//!   This can be used to determine if and how many messages are lost.
//!

use heapless::spsc::{Consumer, Producer, Queue};
use miniconf::MiniconfAtomic;
use num_enum::IntoPrimitive;
use serde::{Deserialize, Serialize};
use smoltcp_nal::embedded_nal::{IpAddr, Ipv4Addr, SocketAddr, UdpClientStack};

use heapless::pool::{Box, Init, Pool, Uninit};

use super::NetworkReference;

const MAGIC_WORD: u16 = 0x057C;

// The size of the header, calculated in bytes.
// The header has a 16-bit magic word, an 8-bit format, 16-bit message size, and 32-bit sequence
// number, which corresponds to 9 bytes total.
const HEADER_SIZE: usize = 9;

// The number of frames that can be buffered.
const FRAME_COUNT: usize = 4;

// The size of each message frame in bytes.
const FRAME_SIZE: usize = 512 + HEADER_SIZE;

// The size of the frame queue must be at least as large as the number of frame buffers. Every
// allocated frame buffer should fit in the queue.
const FRAME_QUEUE_SIZE: usize = FRAME_COUNT * 2;

// Static storage used for a heapless::Pool of frame buffers.
static mut FRAME_DATA: [u8; FRAME_SIZE * FRAME_COUNT] =
    [0; FRAME_SIZE * FRAME_COUNT];

/// Represents the destination for the UDP messages to be send to.
///
/// # Miniconf
/// `{"ip": <addr>, "port": <port>}`
///
/// * `<addr>` is an array of 4 bytes. E.g. `[192, 168, 0, 1]`
/// * `<port>` is any unsigned 16-bit value.
///
/// ## Example
/// `{"ip": [192, 168,0, 1], "port": 1111}`
#[derive(
    Copy, Clone, Debug, MiniconfAtomic, Serialize, Deserialize, Default,
)]
pub struct MessageTarget {
    pub ip: [u8; 4],
    pub port: u16,
}

/// Specifies the format of messages
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, IntoPrimitive)]
pub enum MessageFormat {
    /// Reserved, unused format specifier.
    Unknown = 0,

    ///  contains sequential 16bit words in little-endian format.
    Sequential = 1,
}

impl From<MessageTarget> for SocketAddr {
    fn from(target: MessageTarget) -> SocketAddr {
        SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(
                target.ip[0],
                target.ip[1],
                target.ip[2],
                target.ip[3],
            )),
            target.port,
        )
    }
}

/// Configure messaging on a device.
///
/// # Args
/// * `stack` - A reference to the shared network stack.
///
/// # Returns
/// (generator, msg_handler) where `generator` can be used to enqueue data for transmission. The
/// ` msg_handler` is the logically consumer (UDP transmitter) of the enqueued data.
pub fn setup_handler(
    stack: NetworkReference,
) -> (MsgFrameGenerator, MessageHandler) {
    // The queue needs to be at least as large as the frame count to ensure that every allocated
    // frame can potentially be enqueued for transmission.
    let queue =
        cortex_m::singleton!(: Queue<MessageFrame, FRAME_QUEUE_SIZE> = Queue::new())
            .unwrap();
    let (producer, consumer) = queue.split();

    let frame_pool =
        cortex_m::singleton!(: Pool<[u8; FRAME_SIZE]>= Pool::new()).unwrap();

    // Note(unsafe): We guarantee that FRAME_DATA is only accessed once in this function.
    let memory = unsafe { &mut FRAME_DATA };
    frame_pool.grow(memory);

    let msg_generator = MsgFrameGenerator::new(producer, frame_pool);

    let msg_handler = MessageHandler::new(stack, consumer, frame_pool);

    (msg_generator, msg_handler)
}

#[derive(Debug)]
struct MessageFrame {
    buffer: Box<[u8; FRAME_SIZE], Init>,
    offset: usize,
}

impl MessageFrame {
    pub fn new(
        buffer: Box<[u8; FRAME_SIZE], Uninit>,
        format: u8,
        sequence_number: u32,
        //
    ) -> Self {
        let mut buffer = unsafe { buffer.assume_init() };
        buffer[0..2].copy_from_slice(&MAGIC_WORD.to_ne_bytes());
        buffer[2] = format;
        buffer[3..5].copy_from_slice(&[0;2]);
        buffer[5..9].copy_from_slice(&sequence_number.to_ne_bytes());
        Self {
            buffer,
            offset: HEADER_SIZE,
        }
    }

    pub fn add_buffer<F, const T: usize>(&mut self, mut f: F)
    where
        F: FnMut(&mut [u8]),
    {
        f(&mut self.buffer[self.offset..self.offset + T]);

        self.offset += T;
    }

    pub fn is_full<const T: usize>(&self) -> bool {
        self.offset + T > self.buffer.len()
    }

    pub fn finish(&mut self) -> &[u8] {
        &self.buffer[..self.offset]
    }

    pub fn set_frame_pl_len( &mut self, len:u16 ){
        self.buffer[3..5].copy_from_slice(&len.to_ne_bytes());
    }

    pub fn pad_buffer(&mut self){
        self.offset = FRAME_SIZE;
    }
}

/// The data generator for a message.
pub struct MsgFrameGenerator {
    queue: Producer<'static, MessageFrame, FRAME_QUEUE_SIZE>,
    pool: &'static Pool<[u8; FRAME_SIZE]>,
    current_frame: Option<MessageFrame>,
    sequence_number: u32,
    format: u8,
    padding: bool,
}

impl MsgFrameGenerator {
    fn new(
        queue: Producer<'static, MessageFrame, FRAME_QUEUE_SIZE>,
        pool: &'static Pool<[u8; FRAME_SIZE]>,
    ) -> Self {
        Self {
            queue,
            pool,
            format: MessageFormat::Unknown.into(),
            current_frame: None,
            sequence_number: 0,
            padding: true,
        }
    }

    pub fn set_msg_len( &mut self, len:u16 ){
        // Note(unwrap): We ensure the frame is present above.
        let current_frame = self.current_frame.as_mut().unwrap();
        current_frame.set_frame_pl_len(len);
    }

    pub fn queue_for_tx( &mut self ){
        if self.current_frame.is_none() {
            if let Some(buffer) = self.pool.alloc() {
                self.current_frame.replace(MessageFrame::new(
                    buffer,
                    self.format as u8,
                    0,
                ));
            } else {
                return;
            }
        }
        let mut current_frame = self.current_frame.take().unwrap();
        if self.padding {
            current_frame.pad_buffer();
        }
        self.queue
            .enqueue(current_frame)
            .unwrap();
    }

    /// Configure the format of the message.
    ///
    /// # Note:
    /// This function shall only be called once upon initialization
    ///
    /// # Args
    ///  todo
    /// * `format` - The desired format of the message.
    /// * `batch_size` - The number of samples in each data batch. See
    /// [crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE]
    #[doc(hidden)]
    pub(crate) fn configure(&mut self, format: impl Into<u8>) {
        self.format = format.into();
    }

    /// Add a data to the current message frame.

    /// # Args
    /// * `f` - A closure that will be provided the buffer to write batch data into. The buffer will
    ///   be the size of the `T` template argument.
    pub fn add<F, const T: usize>(&mut self, f: F)
    where
        F: FnMut(&mut [u8]),
    {
        let sequence_number = self.sequence_number;
        self.sequence_number = self.sequence_number.wrapping_add(1);

        if self.current_frame.is_none() {
            if let Some(buffer) = self.pool.alloc() {
                self.current_frame.replace(MessageFrame::new(
                    buffer,
                    self.format as u8,
                    sequence_number,
                ));
            } else {
                return;
            }
        }

        // Note(unwrap): We ensure the frame is present above.
        let current_frame = self.current_frame.as_mut().unwrap();

        current_frame.add_buffer::<_, T>(f);

        if current_frame.is_full::<T>() {
            // Note(unwrap): The queue is designed to be at least as large as the frame buffer
            // count, so this enqueue should always succeed.
            self.queue
                .enqueue(self.current_frame.take().unwrap())
                .unwrap();
        }
    }

    pub fn flush (&mut self)
    {
        if self.current_frame.is_none() {
            return;
        }
        else{
            let frame = self.current_frame.take().unwrap();
            self.pool.free(frame.buffer);
        }
        self.sequence_number = 0;
    }
}

/// The "consumer" portion of the message service.
///
/// # Note
/// This is responsible for consuming data and sending it over UDP.
pub struct MessageHandler {
    stack: NetworkReference,
    socket: Option<<NetworkReference as UdpClientStack>::UdpSocket>,
    queue: Consumer<'static, MessageFrame, FRAME_QUEUE_SIZE>,
    frame_pool: &'static Pool<[u8; FRAME_SIZE]>,
    remote: SocketAddr,
}

impl MessageHandler {
    /// Construct a new message handler.
    ///
    /// # Args
    /// * `stack` - A reference to the shared network stack.
    /// * `consumer` - The read side of the queue containing data to transmit.
    /// * `frame_pool` - The Pool to return message frame objects into.
    fn new(
        stack: NetworkReference,
        consumer: Consumer<'static, MessageFrame, FRAME_QUEUE_SIZE>,
        frame_pool: &'static Pool<[u8; FRAME_SIZE]>,
    ) -> Self {
        Self {
            stack,
            socket: None,
            remote: MessageTarget::default().into(),
            queue: consumer,
            frame_pool,
        }
    }

    fn close(&mut self) {
        if let Some(socket) = self.socket.take() {
            log::info!("Closing message socket");
            // Note(unwrap): We guarantee that the socket is available above.
            self.stack.close(socket).unwrap();
        }
    }

    // Open new socket.
    fn open(&mut self) -> Result<(), ()> {
        // If there is already a socket of if remote address is unspecified,
        // do not open a new socket.
        if self.socket.is_some() || self.remote.ip().is_unspecified() {
            return Err(());
        }

        log::info!("Opening message socket");

        let mut socket = self.stack.socket().or(Err(()))?;

        // Note(unwrap): We only connect with a new socket, so it is guaranteed to not already be
        // bound.
        self.stack.connect(&mut socket, self.remote).unwrap();

        self.socket.replace(socket);

        Ok(())
    }

    /// Configure the remote endpoint of the messages.
    ///
    /// # Args
    /// * `remote` - The destination to send messages to.
    pub fn set_remote(&mut self, remote: SocketAddr) {
        // Close socket to be reopened if the remote has changed.
        if remote != self.remote {
            self.close();
        }
        self.remote = remote;
    }

    /// Process any data for transmission.
    pub fn process(&mut self) {
        match self.socket.as_mut() {
            None => {
                // If there's no socket available, try to connect to our remote.
                if self.open().is_ok() {
                    // If we just successfully opened the socket, flush old data from queue.
                    while let Some(frame) = self.queue.dequeue() {
                        self.frame_pool.free(frame.buffer);
                    }
                }
            }
            Some(handle) => {
                if let Some(mut frame) = self.queue.dequeue() {
                    // Transmit the frame and return it to the pool.
                    self.stack.send(handle, frame.finish()).ok();
                    self.frame_pool.free(frame.buffer)
                }
            }
        }
    }
}
