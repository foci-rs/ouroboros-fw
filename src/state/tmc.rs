//! TMC4671-subsystem board-local state: RTIC channel sender and channel size.
//! All cross-priority atomics and the disable signal have moved to
//! `foci_firmware::state::tmc`.

use core::cell::RefCell;

use critical_section::Mutex;
use foci_firmware::tmc::TmcRequest;
use rtic_sync::channel::Sender;

use crate::board_config;

/// Channel depth for TMC control requests.
///
/// Connect-time TMC config handlers apply state directly instead of
/// enqueuing (see `foci_firmware::state::tmc::CONNECT_TIME_QUEUE_SLOTS_PER_CHANNEL`);
/// `tmc_set_motion_scale` is the one exception and still enqueues one
/// request per channel. The assertion below reserves headroom for that
/// worst case, doubled to leave half the queue free for the action
/// commands that follow connect.
pub(crate) const TMC_CHANNEL_SIZE: usize = 8;
const _: () = assert!(
    board_config::TMC_CHANNEL_COUNT as usize
        * foci_firmware::state::tmc::CONNECT_TIME_QUEUE_SLOTS_PER_CHANNEL
        * 2
        <= TMC_CHANNEL_SIZE
);

/// Shared sender handle for the TMC control channel. Populated in RTIC
/// `init`, used by P2 command handlers to enqueue [`TmcRequest`]s.
pub(crate) static TMC_REQUEST_SENDER: Mutex<
    RefCell<Option<Sender<'static, TmcRequest, TMC_CHANNEL_SIZE>>>,
> = Mutex::new(RefCell::new(None));
