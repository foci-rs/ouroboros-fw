//! TMC4671-subsystem board-local state: RTIC channel sender and channel size.
//! All cross-priority atomics and the disable signal have moved to
//! `foci_firmware::state::tmc`.

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::Mutex;
use foci_firmware::tmc::TmcRequest;
use rtic_sync::channel::Sender;

/// Channel depth for TMC control requests.
pub(crate) const TMC_CHANNEL_SIZE: usize = 8;

/// Shared sender handle for the TMC control channel. Populated in RTIC
/// `init`, used by P2 command handlers to enqueue [`TmcRequest`]s.
pub(crate) static TMC_REQUEST_SENDER: Mutex<
    RefCell<Option<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>>>,
> = Mutex::new(RefCell::new(None));

static BOARD_COMMISSIONING_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Try to reserve the board-global commissioning engine slot.
pub(crate) fn try_acquire_commissioning() -> bool {
    BOARD_COMMISSIONING_ACTIVE
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
        .is_ok()
}

/// Release the board-global commissioning engine slot.
pub(crate) fn release_commissioning() {
    BOARD_COMMISSIONING_ACTIVE.store(false, Ordering::Release);
}
