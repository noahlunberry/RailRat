use embassy_stm32::{
    mode::Async,
    usart::{Uart, Error},
};

use embassy_time::{
    with_timeout, 
    Duration
};



#[derive(Debug)]
pub enum UartError {
    Hardware(Error),
    Timeout,
}

impl From<Error> for UartError {
    fn from(error: Error) -> Self {
        Self::Hardware(error)
    }
}

/// Writes to a UART module, utilizing a timeout.
/// 
/// # Arguments
/// 
/// - `uart` (`&mut Uart<'_, Async>`) - The UART module to be written to.
/// - `data` (`&[u8]`) - The data that will be written to the UART.
/// - `timeout` (`Duration`) - Timeout duration.
/// 
/// # Returns
/// 
/// - `Result<(), UartError>` -
///     > Nothing if successful
///     > UartError struct if failure.
/// 
/// # Errors
/// 
/// Describe possible errors.
/// 
/// # Examples
/// 
/// ```no_run
/// use crate::...;
/// 
/// async {
///   let result = uart_write().await;
/// };
/// ```
pub async fn uart_write(
    uart: &mut Uart<'_, Async>,
    data: &[u8],
    timeout: Duration,
) -> Result<(), UartError> {
    with_timeout(timeout, uart.write(data))
        .await
        .map_err(|_| UartError::Timeout)??;

    Ok(())
}

pub async fn uart_read(
    uart: &mut Uart<'_, Async>,
    buf: &mut [u8],
    timeout: Duration,
) -> Result<(), UartError> {
    with_timeout(timeout, uart.read(buf))
        .await
        .map_err(|_| UartError::Timeout)??;

    Ok(())
}