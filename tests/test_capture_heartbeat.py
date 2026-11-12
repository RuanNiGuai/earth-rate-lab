"""Compile the production heartbeat policy; exercise recovery and shared pipes."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from native_test import compile_run


class HeartbeatTests(unittest.TestCase):
    def test_recovery_bounds_jitter_and_reply_identity(self):
        parent = ROOT/'.local/native-tests'
        parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=parent) as directory:
            compile_run(r'''
#include <assert.h>
#include "capture_heartbeat.h"
int main(void) {
    for (unsigned id=0; id<32; id++) {
        unsigned changes=0, previous=0;
        for (unsigned n=0; n<2000; n++) {
            unsigned lost=capture_heartbeat_interval(id,n,-1);
            unsigned stale=capture_heartbeat_interval(id,n,20000);
            unsigned fresh=capture_heartbeat_interval(id,n,0);
            assert(lost>=160 && lost<=310 && lost==stale);
            assert(fresh>=600 && fresh<=750);
            changes += lost!=previous; previous=lost;
        }
        assert(changes>1900); /* no fixed phase lock after repeated failures */
    }
    assert(heartbeat_reply_matches(3,3,255,255,true));
    assert(heartbeat_reply_matches(3,3,0,0,true));
    assert(!heartbeat_reply_matches(3,10,0,0,true)); /* shared pipe */
    assert(!heartbeat_reply_matches(3,3,0,255,true)); /* old counter */
    assert(!heartbeat_reply_matches(3,3,0,0,false)); /* unsolicited reply */
    return 0;
}
''', directory, [ROOT/'integration/tracker/src/connection'])
