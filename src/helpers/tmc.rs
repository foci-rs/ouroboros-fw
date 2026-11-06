//! TMC4671 helpers: the board's request queue into the `tmc_control` task.

use foci_firmware::context::tmc::TmcRequestQueueBoard;
use foci_firmware::tmc::TmcRequest;

use crate::state::tmc::{TMC_CONTROL_SENDER, TmcControlEvent};

pub(crate) struct OuroborosTmcQueue;

impl TmcRequestQueueBoard for OuroborosTmcQueue {
    fn try_send_tmc_request(&self, req: TmcRequest) -> bool {
        critical_section::with(|cs| {
            if let Some(sender) = TMC_CONTROL_SENDER.borrow(cs).borrow_mut().as_mut() {
                return sender.try_send(TmcControlEvent::Request(req)).is_ok();
            }
            false
        })
    }
}
