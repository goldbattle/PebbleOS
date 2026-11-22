# The `aria-imu` branch: PebbleOS changes for a sensor recorder

This branch is upstream `coredevices/pebbleos` `main` plus the changes below, made for
[PebbleRecorder](https://github.com/goldbattle/PebbleRecorder), an app that records a Pebble Time 2's
IMU (accelerometer and gyroscope) during golf rounds and streams it to a phone through data logging. Each change is
one commit with its own tests. The descriptions here say what was wrong (with what we saw on a real
watch), what changed, and how it is tested.

| # | Commit | What | Tests |
|---|---|---|---|
| 1 | `kernel/time: microsecond clocks, and time_us() for apps` | `pbl_uptime_us`, `pbl_realtime_us`, `pbl_realtime_offset_us`, app `time_us()` | `tests/kernel/test_timekeeping.c` |
| 2 | `kernel: clocksync` | map a device's own counter onto watch time | `tests/kernel/test_clocksync.c` |
| 3 | `drivers/imu/lsm6dso: per-sample hardware timestamps, gyroscope and range` | every IMU sample stamped by the sensor; gyro and ±16 g for apps; 208/416 Hz | `tests/pebble/drivers/test_lsm6dso.c`, `tests/pebble/applib/test_accel_service.c` |
| 4 | `applib/click: the time of the physical button press` | `click_recognizer_get_press_time_us()` | `tests/pebble/ui/test_click_press_time.c` |
| 5 | `services/data_logging: drop whole items when the store is full` | a full store no longer cuts items in two | `test_data_logging__full_store_drops_whole_items` |
| 6 | `services/data_logging: run the ack timeout on the storage work queue` | no watchdog reboot when the phone stops acking | `test_data_logging__ack_timeout_runs_on_the_storage_queue` |
| 7 | `services/data_logging: check for data to send every minute` | was every 5 minutes | `test_data_logging__active_session_is_sent_within_a_minute` |
| 8 | `sdk: export ... (revision 117)` | the app APIs of 1, 3 and 4 in the SDK | (SDK generation) |

Changes 5, 6 and 7 are plain bug fixes that stand on their own upstream. 1 to 4 add API and are
specific to sensor recording, but none of them changes existing behaviour for other apps.

## 1. Microsecond clocks and `time_us()`

**Problem.** The kernel counts uptime in microseconds but exposed the wall clock only in milliseconds,
and apps only had `time_ms()`. At 208 Hz two IMU samples are 4.8 ms apart, and comparing watch and phone
clocks to ~1 ms needs better than millisecond stamps. The emulator's simulated accelerometer stamped
samples with `pbl_realtime_ms() * 1000`; when its timer ran late and fired twice within a millisecond,
two samples got the same stamp.

**Change.** `pbl_uptime_us()` (monotonic), `pbl_realtime_us()` (the wall clock of `pbl_realtime_ms()` at
microsecond resolution), `pbl_realtime_offset_us()` (wall clock minus uptime), and for apps
`uint64_t time_us(void)`. Every timestamp in the following changes is on the `time_us()` clock, so an app
can compare a sample's time with "now".

**Tests.** `test_timekeeping__microsecond_clocks`: the clocks advance by exactly the elapsed time, agree
with the millisecond clock, and wall clock = uptime + offset after setting the time.

## 2. clocksync

**Problem.** A sensor that stamps samples with its own counter is only useful if that counter can be
mapped onto the watch clock. One "read counter, read clock" pair is as imprecise as the bus read
(slower whenever the bus or CPU is busy), and the sensor's oscillator differs from the watch's: on our
watch the LSM6DSO ran about 1.7 % fast (inferred from the delivered rate), 17 ms per second, and
oscillators drift with temperature.

**Change.** `kernel/clocksync.c` keeps an exponentially weighted least-squares fit
`uptime = a + b * ticks` over the pairs a driver feeds it. Pairs are weighted by 1/bracket², so one read
while the bus was busy barely counts; old pairs fade with a time constant; the fit is re-centred on
every pair; the counter may wrap; `pbl_clocksync_restart()` forgets the fit when the device is
reconfigured.

```c
pbl_clocksync_init(&cs, 25.0, 32, 60.0);  // 25 us ticks, 32-bit counter, 60 s memory
uint64_t before = pbl_uptime_us();
uint32_t ticks = read_sensor_counter();
uint64_t after = pbl_uptime_us();
pbl_clocksync_add_pair(&cs, ticks, before, after);
uint64_t t_us = pbl_clocksync_to_uptime_us(&cs, sample_ticks) + pbl_realtime_offset_us();
```

**Tests.** `test_clocksync`: the first pair uses the nominal rate, a fast and drifting oscillator is
tracked, the counter wraps, and a restart follows a new rate.

## 3. LSM6DSO: per-sample hardware timestamps, gyroscope, range

**Problem.** Measured on a Pebble Time 2 recording golf swings:

1. *Stamps were a whole batch late.* The driver read the FIFO, then stamped the batch's oldest sample
   with the current time; the accel manager added `i / rate` for the others. With 25-sample batches
   every stamp was ~236 ms late: a button press's vibration appeared at stamp = press + 251 ms (median,
   n = 12), and a tap test put the watch 246 ± 3 ms behind a phone tapped at the same moment.
2. *The spacing was invented.* Stamps advanced by exactly `1/rate` while the sensor runs on its own
   oscillator, so they broke at every batch boundary (946 breaks in one recording, all on batch
   boundaries).
3. *No gyroscope, ±4 g.* The LSM6DSO's gyro stayed powered down with no API, and a golf swing exceeds
   4 g, so the accelerometer clipped.

**Change.** The driver enables the FIFO timestamp, so every sample carries the sensor's 25 µs counter,
and converts it with clocksync (pairs taken at every FIFO read). The gyro (±2000 dps) and the
accelerometer range are set with `pbl_accel_set_config()`; the accel manager merges all subscribers
(gyro on if anyone asks, the largest range) and powers the gyro down when nobody wants it. 208 and
416 Hz are available.

```c
static void imu_handler(ImuData *d, uint32_t n) {
  // d[i].ax/ay/az in mg, d[i].gx/gy/gz in 0.1 deg/s, d[i].timestamp_us on the time_us() clock
}
ImuConfig cfg = {.samples_per_update = 26, .rate = IMU_SAMPLING_208HZ, .gyro = true,
                 .accel_range_g = 16};
imu_data_service_subscribe_config(&cfg, imu_handler);
```

`ImuConfig` members left at 0 keep their defaults, so members can be added without breaking apps.

**Tests.** `test_lsm6dso` runs the driver against a fake sensor FIFO: accel-only streams, the three
orders of accel/gyro/timestamp words, a sample split across two reads, the watermark with the gyro on,
the timestamp tick from the chip's frequency trim, range changes, and restarting the fit on
reconfiguration. `test_accel_service` checks apps get each sample's own time.

## 4. Button press time

**Problem.** Click handlers run well after the press: a single click with a long click configured fires
at the release, a long click after its delay. An app labelling sensor data with button presses had no
way to know when the button went down.

**Change.** Input events carry a timestamp; the GPIO button driver takes it in the interrupt of the
first edge, before debouncing. Button events carry its low 32 bits (keeping the event at 5 bytes) and
the click recognizer expands them back: the latest time with those bits that is not after now.

```c
static void select_long(ClickRecognizerRef r, void *ctx) {
  uint64_t pressed_us = click_recognizer_get_press_time_us(r);  // time_us() clock
}
```

**Tests.** `test_click_press_time`: a single click handled at the release reports the press, so does a
long click; a press just before the low 32 bits wrap; a press without an edge time is "now".

## 5. Data logging: drop whole items when the store is full

**Problem.** Data logging stores items in chunks of at most 100 B, so a 296 B item is 100 + 100 + 96.
When the store was full, the drop of old data counted whole chunks and could stop inside an item. The
data left then started with the tail of a cut item, and every later delivery was shifted by it. A real
recording: every delivery started 96 B into an item, the phone could parse none of them, and the rest of
the session was lost.

**Change.** `dls_storage_consume()` keeps consuming chunks until it ends on an item boundary.

**Tests.** `test_data_logging__full_store_drops_whole_items` fills the store twice over with 296 B items
whose words hold the item's index; what is left must be whole items, in order, ending with the last one.
It fails without the change.

## 6. Data logging: the ack timeout on the storage work queue

**Problem.** Every message to the phone waits for its ack with a 30 s timeout. `ack_timer_cb` takes the
session list lock, which storage work holds for seconds while compacting a full store. On
`pbl_sys_workq_high` it blocked behind that work until the task watchdog rebooted the watch. Reproduced
in the emulator with a phone that stops acking for 2 minutes while the store is full: a reboot every
time.

**Change.** The timeout runs on `pbl_sys_workq`. No reboot; everything is sent once the phone acks
again.

**Tests.** `test_data_logging__ack_timeout_runs_on_the_storage_queue`.

## 7. Data logging: check for data every minute

**Problem.** An active session is sent when the periodic check finds 8000 B or more. The check ran every
5 minutes, and the ~560 KB store holds only ~4-5 minutes of a 208 Hz accelerometer + gyroscope stream.
A recording overflowed the store before its first upload and the full store dropped the oldest half of
it, with the phone in range the whole time.

**Change.** The check runs every minute. Small sessions still wait, finished sessions are still sent at
once, every session is still flushed every 15 minutes.

**Tests.** `test_data_logging__active_session_is_sent_within_a_minute` (fails with 5 minutes).

## 8. SDK revision 117

The app APIs of 1, 3 and 4 are exported as revision 117 / SDK 0x5.0x74, after upstream's newest
(116), so apps built with the upstream SDK keep their ABI. When upstream adds revisions, ours move past
them (`tools/generate_native_sdk/exported_symbols.json`, `pebble_process_info.h`).

## Open issue found on the way (not fixed here)

**A session whose read position is not on an item boundary is never sent and never deleted.**
`dls_storage_read()` only hands out data up to the last whole item counted from the read position. A
session left mid-item by the old drop (change 5 prevents new ones) reads 0 bytes on every attempt: it
is never sent, never consumed, and keeps its space. On our watch two such sessions from before change 5
filled the store; every write then compacted storage (the log showed hundreds of "Compacting storage"
lines), the system task stayed busy, apps blocked in `data_logging_log()` and lost IMU samples, and new
sessions were cut in half. A factory reset cleared it. A fix would delete (or realign) a session whose
read returns nothing while it still holds data.

## Building and testing

```sh
pbl test -R 'clocksync|timekeeping|lsm6dso|accel_service|click_press_time|data_logging'
pbl test                                # everything
pbl configure -b build --board obelix@pvt && pbl build -b build    # Pebble Time 2
```

End to end, PebbleRecorder's `tools/emulator/e2e.sh` runs its watch app on this firmware in QEMU against
a scripted phone (long recordings, a phone that stops acking, a backlog in the store), and the app records
the watch's data-logging traffic and log on real hardware (its Watch Debug Log setting).
