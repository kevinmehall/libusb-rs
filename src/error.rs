use std::fmt;
use std::error::Error as StdError;
use std::result::Result as StdResult;

use libc::{c_int};


/// A result of a function that may return a `Error`.
pub type Result<T> = StdResult<T, Error>;


/// Errors returned by the `libusb` library.
#[derive(Debug)]
pub enum Error {
    /// Success (no error).
    Success,

    /// Input/output error.
    Io,

    /// Invalid parameter.
    InvalidParam,

    /// Access denied (insufficient permissions).
    Access,

    /// No such device (it may have been disconnected).
    NoDevice,

    /// Entity not found.
    NotFound,

    /// Resource busy.
    Busy,

    /// Operation timed out.
    Timeout,

    /// Overflow.
    Overflow,

    /// Pipe error.
    Pipe,

    /// System call interrupted (perhaps due to signal).
    Interrupted,

    /// Insufficient memory.
    NoMem,

    /// Operation not supported or unimplemented on this platform.
    NotSupported,

    /// Other error.
    Other
}

impl Error {
    /// Returns a description of an error suitable for display to an end user.
    pub fn strerror(&self) -> &'static str {
        match *self {
            Error::Success      => "Success",
            Error::Io           => "Input/Output Error",
            Error::InvalidParam => "Invalid parameter",
            Error::Access       => "Access denied (insufficient permissions)",
            Error::NoDevice     => "No such device (it may have been disconnected)",
            Error::NotFound     => "Entity not found",
            Error::Busy         => "Resource busy",
            Error::Timeout      => "Operation timed out",
            Error::Overflow     => "Overflow",
            Error::Pipe         => "Pipe error",
            Error::Interrupted  => "System call interrupted (perhaps due to signal)",
            Error::NoMem        => "Insufficient memory",
            Error::NotSupported => "Operation not supported or unimplemented on this platform",
            Error::Other        => "Other error"
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> StdResult<(), fmt::Error> {
        fmt.write_str(self.strerror())
    }
}

impl StdError for Error {
    fn description(&self) -> &'static str {
        self.strerror()
    }
}


#[doc(hidden)]
pub fn from_libusb(err: c_int) -> Error {
    match err {
        ::libusb::LIBUSB_SUCCESS             => Error::Success,
        ::libusb::LIBUSB_ERROR_IO            => Error::Io,
        ::libusb::LIBUSB_ERROR_INVALID_PARAM => Error::InvalidParam,
        ::libusb::LIBUSB_ERROR_ACCESS        => Error::Access,
        ::libusb::LIBUSB_ERROR_NO_DEVICE     => Error::NoDevice,
        ::libusb::LIBUSB_ERROR_NOT_FOUND     => Error::NotFound,
        ::libusb::LIBUSB_ERROR_BUSY          => Error::Busy,
        ::libusb::LIBUSB_ERROR_TIMEOUT       => Error::Timeout,
        ::libusb::LIBUSB_ERROR_OVERFLOW      => Error::Overflow,
        ::libusb::LIBUSB_ERROR_PIPE          => Error::Pipe,
        ::libusb::LIBUSB_ERROR_INTERRUPTED   => Error::Interrupted,
        ::libusb::LIBUSB_ERROR_NO_MEM        => Error::NoMem,
        ::libusb::LIBUSB_ERROR_NOT_SUPPORTED => Error::NotSupported,
        ::libusb::LIBUSB_ERROR_OTHER | _     => Error::Other
    }
}

#[doc(hidden)]
macro_rules! try_unsafe {
    ($x:expr) => {
        match unsafe { $x } {
            0 => (),
            err => return Err(::error::from_libusb(err)),
        }
    }
}
