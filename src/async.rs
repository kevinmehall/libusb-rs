use std::marker::PhantomData;
use libc::{ c_uint, c_int, c_uchar, c_void };
use std::time::Duration;
use std::slice;
use std::sync::Mutex;
use std::collections::{ VecDeque, HashSet };
use std::cell::UnsafeCell;
use std::mem;

/// An asynchronous transfer that is not currently pending.
/// Specifies the data necessary to perform a transfer on a specified endpoint, and holds the
/// result of a completed transfer. A completed Transfer can be resubmitted.
pub struct Transfer<'d> {
    _handle: PhantomData<&'d ::DeviceHandle<'d>>,  // transfer.dev_handle
    buffer: Vec<u8>, // move buffer into transfer
    transfer: *mut ::libusb::libusb_transfer,
}

/// The status of a Transfer returned by wait_any.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum TransferStatus {
    /// Completed without error
    Success = ::libusb::LIBUSB_TRANSFER_COMPLETED as isize,

    /// Failed (IO error)
    Error = ::libusb::LIBUSB_TRANSFER_ERROR as isize,

    /// Timed out
    Timeout = ::libusb::LIBUSB_TRANSFER_TIMED_OUT as isize,

    /// Cancelled
    Cancelled = ::libusb::LIBUSB_TRANSFER_CANCELLED as isize,

    /// Endpoint stalled or control request not supported
    Stall = ::libusb::LIBUSB_TRANSFER_STALL as isize,

    /// Device was disconnected
    NoDevice = ::libusb::LIBUSB_TRANSFER_NO_DEVICE as isize,

    /// Device sent more data than requested
    Overflow = ::libusb::LIBUSB_TRANSFER_OVERFLOW as isize,

    /// No status, not yet submitted
    Unknown = -1 as isize,
}

impl<'d> Transfer<'d> {
    fn new(handle: &'d ::DeviceHandle<'d>, endpoint: u8, transfer_type: c_uchar, mut buffer: Vec<u8>, timeout: Duration) -> Transfer<'d> {
        let timeout_ms = timeout.as_secs() * 1000 + timeout.subsec_nanos() as u64 / 1_000_000;
        unsafe {
            let t = ::libusb::libusb_alloc_transfer(0);
            (*t).status = -1;
            (*t).dev_handle = handle.as_raw();
            (*t).endpoint = endpoint as c_uchar;
            (*t).transfer_type = transfer_type;
            (*t).timeout = timeout_ms as c_uint;
            (*t).buffer = buffer.as_mut_ptr();
            (*t).length = buffer.len() as i32;
            (*t).actual_length = 0;

            buffer.shrink_to_fit();
            Transfer{ transfer: t, _handle: PhantomData, buffer: buffer }
        }
    }

    /// Creates an asynchronous bulk transfer, but does not submit it.
    pub fn bulk(handle: &'d ::DeviceHandle<'d>, endpoint: u8, buffer: Vec<u8>, timeout: Duration) -> Transfer<'d> {
        Transfer::new(handle, endpoint, ::libusb::LIBUSB_TRANSFER_TYPE_BULK, buffer, timeout)
    }

    /// Creates an asynchronous interrupt transfer, but does not submit it.
    pub fn interrupt(handle: &'d ::DeviceHandle<'d>, endpoint: u8, buffer: Vec<u8>, timeout: Duration) -> Transfer<'d> {
        Transfer::new(handle, endpoint, ::libusb::LIBUSB_TRANSFER_TYPE_INTERRUPT, buffer, timeout)
    }

    /// Creates an asynchronous control transfer, but does not submit it.
    /// In difference to the other functions, this function takes  additional arguments.
    /// The additional arguments are the ones also used in the synchron version
    /// of read_control / write_control.
    pub fn control(handle: &'d ::DeviceHandle<'d>, endpoint: u8, buffer: Vec<u8>,
                   request_type: u8, request: u8, value: u16, index: u16,
                   timeout: Duration) -> Transfer<'d> {
        let length = buffer.len() as u16;
        let vec: Vec<u8> = [
            request_type,
            request,
            (value & 0xff) as u8,
            (value >> 8) as u8,
            (index & 0xff) as u8,
            (index >> 8) as u8,
            (length & 0xff) as u8,
            (length >> 8) as u8,
        ].iter().cloned().chain(buffer).collect();
        Transfer::new(handle, endpoint, ::libusb::LIBUSB_TRANSFER_TYPE_CONTROL, vec, timeout)
    }

    pub fn endpoint(&self) -> u8 {
        unsafe {
            (*self.transfer).endpoint as u8
        }
    }

    pub fn timeout(&self) -> Duration {
        unsafe {
            Duration::from_millis((*self.transfer).timeout as u64)
        }
    }

    /// Gets the status of a completed transfer.
    pub fn status(&self) -> TransferStatus {
        match unsafe { (*self.transfer).status } {
            ::libusb::LIBUSB_TRANSFER_COMPLETED => TransferStatus::Success,
            ::libusb::LIBUSB_TRANSFER_ERROR => TransferStatus::Error,
            ::libusb::LIBUSB_TRANSFER_TIMED_OUT => TransferStatus::Timeout,
            ::libusb::LIBUSB_TRANSFER_CANCELLED => TransferStatus::Cancelled,
            ::libusb::LIBUSB_TRANSFER_STALL => TransferStatus::Stall,
            ::libusb::LIBUSB_TRANSFER_NO_DEVICE => TransferStatus::NoDevice,
            _ => TransferStatus::Unknown,
        }
    }

    /// Access the buffer of a transfer.
    pub fn buffer(&mut self) -> &mut [u8] {
        &mut self.buffer
    }

    /// Replace the buffer of a transfer.
    pub fn set_buffer(&mut self, mut buffer: Vec<u8>) {
        unsafe {
            (*self.transfer).buffer = buffer.as_mut_ptr();
            (*self.transfer).length = buffer.len() as i32;
            (*self.transfer).actual_length = 0;
        }
        self.buffer = buffer;
    }

    /// Access the slice of the buffer containing actual data received on an IN transfer.
    pub fn actual(&mut self) -> &'d mut [u8] {
        unsafe {
            // if this is a control request, the first 8 bytes of the buffer are
            // the setup header
            let offset = match (*self.transfer).transfer_type {
                ::libusb::LIBUSB_TRANSFER_TYPE_CONTROL => 8,
                _ => 0
            };
            slice::from_raw_parts_mut((*self.transfer).buffer.offset(offset), (*self.transfer).actual_length as usize)
        }
    }
}

impl<'d> Drop for Transfer<'d> {
    fn drop(&mut self) {
        unsafe { ::libusb::libusb_free_transfer(self.transfer); }
    }
}

/// Internal type holding data touched by libusb completion callback.
struct CallbackData {
    /// Transfers that have completed, but haven't yet been returned from `wait_any`.
    completed: Mutex<VecDeque<*mut ::libusb::libusb_transfer>>,

    /// Signals a completion to avoid race conditions between callback and
    /// `libusb_handle_events_completed`. This is synchronized with the
    /// Mutex above, but can't be included in it because libusb reads it
    /// without the lock held.
    flag: UnsafeCell<c_int>,
}

/// An AsyncGroup manages outstanding asynchronous transfers.
pub struct AsyncGroup<'d> {
    context: &'d ::Context,

    /// The data touched by the callback, boxed to keep a consistent address if the AsyncGroup
    /// is moved while transfers are active.
    callback_data: Box<CallbackData>,

    /// The set of pending transfers. We need to keep track of them so they can be cancelled on
    /// drop.
    pending: HashSet<*mut ::libusb::libusb_transfer>,
}

/// The libusb transfer completion callback. Careful: libusb may call this on any thread!
extern "C" fn async_group_callback(transfer: *mut ::libusb::libusb_transfer) {
    unsafe {
        let callback_data: &CallbackData = &*((*transfer).user_data as *const CallbackData);
        let mut completed = callback_data.completed.lock().unwrap();
        completed.push_back(transfer);
        *(callback_data.flag.get()) = 1;
    }
}

impl<'d> AsyncGroup<'d> {
    /// Creates an AsyncGroup to process transfers for devices from the given context.
    pub fn new(context: &'d ::Context) -> AsyncGroup<'d> {
        AsyncGroup {
            context: context,
            callback_data: Box::new(CallbackData {
                completed: Mutex::new(VecDeque::new()),
                flag: UnsafeCell::new(0),
            }),
            pending: HashSet::new(),
        }
    }

    /// Starts a transfer.
    ///
    /// The Transfer is owned by the AsyncGroup while it is pending, and is
    /// returned from `wait_any` when it completes or fails.
    pub fn submit(&mut self, t: Transfer<'d>) -> ::Result<()> {
        unsafe {
            (*t.transfer).user_data = &mut *self.callback_data as *mut _ as *mut c_void;
            (*t.transfer).callback = async_group_callback;
            try_unsafe!(::libusb::libusb_submit_transfer(t.transfer));
            self.pending.insert(t.transfer);
            mem::forget(t);
            Ok(())
        }
    }

    /// Waits for any pending transfer to complete, and return it.
    pub fn wait_any(&mut self) -> ::Result<Transfer<'d>> {
        if self.pending.len() == 0 {
            // Otherwise this function would block forever waiting for a transfer to complete
            return Err(::Error::NotFound)
        }

        unsafe {
            let transfer;
            loop {
                {
                    let mut completed = self.callback_data.completed.lock().unwrap();
                    if let Some(t) = completed.pop_front() {
                        transfer = t;
                        break;
                    }
                    *self.callback_data.flag.get() = 0;
                }
                try_unsafe!(::libusb::libusb_handle_events_completed(
                    self.context.as_raw(),
                    self.callback_data.flag.get()
                ));
            }

            if !self.pending.remove(&transfer) {
                panic!("Got a completion for a transfer that wasn't pending");
            }
            
            let vec = Vec::from_raw_parts((*transfer).buffer, (*transfer).length as usize, (*transfer).length as usize);
            Ok(Transfer{ transfer: transfer, _handle: PhantomData, buffer: vec })
        }
    }

    /// Cancels all pending transfers.
    ///
    /// Throws away any received data and errors on transfers that have completed, but haven't been
    /// collected by `wait_any`.
    pub fn cancel_all(&mut self) -> ::Result<()> {
        for &transfer in self.pending.iter() {
            try_unsafe!(::libusb::libusb_cancel_transfer(transfer))
        }

        while self.pending.len() > 0 {
            try!(self.wait_any());
        }

        Ok(())
    }
}

impl<'d> Drop for AsyncGroup<'d> {
    fn drop(&mut self) {
        self.cancel_all().ok();
    }
}
