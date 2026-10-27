^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for ouster_sim_core
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Versions follow ``project(VERSION ...)`` in ``CMakeLists.txt``. The minor version moves when
synthesized channel output, packet output or the optical contract changes, and
``kOpticalChannelContractVersion`` moves with any change to the frozen random or channel golden
vectors. Parents pin a commit; each entry lists what reached them under that version.

0.7.0 (2026-10-26)
------------------
* [BREAKING]: Retroreflectors encode on Ouster's calibrated reflectivity scale, where byte 255 is
  a retroreflector 864x stronger than a 100% Lambertian target (firmware 3.2 and later). The
  log2 band above 100 now climbs 155 / log2(864) = 15.89 bytes per doubling instead of 22, which
  saturated at 132x. Diffuse targets (0-100) are unchanged; a target 10x a white one reads 152
  instead of 173. Ouster's public Rev7 recordings agree: their retroreflective bins imply about
  9-11x near byte 154 and 77x near byte 199, where this scale gives 10.5x and 75x.
* The ``conformance_v1`` fixtures move with it: the retroreflector case, twice a white target,
  expects 115 instead of 122, and the saturated case uses 1024x so it still reaches 255. A
  parent that encodes reflectivity itself must take the same slope when it moves the pin.
  ``kOpticalChannelContractVersion`` is 4.

0.6.0 (2026-10-26)
------------------
* [BREAKING]: Rev7 OS0 and OS1 profiles carry an ``OusterSignalResponse`` fitted to Ouster's
  public sample recordings, and the channel model reports SIGNAL from it:

  - the linear signal of a 100% diffuse target against range, which holds the near-field overlap
    of beam and receiver (an OS1 returns least near 4 m and reaches full overlap near 20 m);
  - the L3 detector's saturation, ``c = x / (1 + (x / 5680)^(4/3))^(3/4)``;
  - shot noise with a 1.337 excess factor through that saturation, plus a 0.77% relative floor;
  - scaling by shots per measurement, so 2048x10 reads half of 1024x10.

  ``opticalChannelModelConfigFromProfile()`` copies the response with the mode's gain, and the
  model scales it by ``base_signal / kReferenceBaseSignal`` (800). An OS1 at 2.2 m on a 29%
  surface reads about 3,000 counts with 1-2% noise, where the inverse-square model gave 44 with
  15%. Products without recordings keep ``base_signal * reflectance / r^2`` with Poisson noise,
  bit for bit. ``kOpticalChannelContractVersion`` is 3.
* Only fitted parameters are included; the recordings (CC BY-NC-SA 4.0) are not distributed.

0.5.0 (2026-10-05)
------------------
* Add ``OusterImuPacketPipeline``, which encodes native LEGACY and ACCEL32_GYRO32_NMEA IMU
  packets: packet geometry and sample cadence come from the metadata, physical IMU state is
  interpolated, and units, identities, status and CRC are SDK-readable.
* Packets match captures from physical sensors that ship with the Ouster SDK:

  - the metadata ``column_window`` is honored. Only packets with an in-window column are
    emitted (``lidarPacketsPerFrame()``), and out-of-window columns keep their measurement ID and
    timestamp with status 0 and zeroed data. A window may wrap through measurement 0.
  - legacy packets repeat the frame ID in every column, report ``measurement_id * 90112 / W``
    encoder counts and mark valid columns with status ``0xffffffff``.
  - [BUGFIX]: channel fields narrower than their source saturate instead of wrapping; the
    low-data ``NIR8`` slot encoded a NEAR_IR of 5000 as 896.

* [BUGFIX]: an IMU packet that failed to encode left its samples buffered, which stalled the IMU
  stream until an epoch reset. Its samples are now dropped and the stream resumes.
* Add ``test_real_capture_conformance`` (enabled by ``OUSTER_SIM_CORE_SDK_TEST_DATA_DIR``), a
  standalone test superproject, an encoder throughput benchmark, and CI that runs Release and
  ASan+UBSan builds against the parents' SDK pin.

0.4.0 (2026-09-08)
------------------
* ``OusterScanFrameView`` encodes from existing producer buffers without copying them, and the
  reusable-output encoder validates the whole frame before it changes packet storage.
* ``optical_value.hpp`` gives device code the presence rule for dense optical buffers: zero is
  supplied data, and a missing or invalid element takes the adapter's default.
* Add the shared ``test/support/conformance_v1.hpp`` fixtures (metadata and profiles, complete
  frames, exact noiseless optical cases), which both simulator parents run against their
  production adapters, and an optional Zenoh packet-burst harness.

0.3.0 (2026-08-28)
------------------
* ``OpticalChannelModel::processRevolution()`` validates one complete revolution in any order and
  derives depth-edge suppression from its canonical neighborhoods, matching standalone
  ``gz_sensors_ouster``. Scalar processing refuses calls while edge suppression is enabled.
* Packet pacing gains a burst delivery mode that makes a whole frame due at once.

0.2.0 (2026-08-27)
------------------
* Add the deterministic optical channel model: detection, range noise, SIGNAL, reflectivity,
  NEAR_IR, dropout and false alarms, with stateless random draws keyed by sensor, epoch, firing
  identity and effect, so batching and call order cannot change a sample.
* Add Gen1 through Gen4 product and revision profiles, and a clock-injected packet-pacing policy.
* Metadata exposes the UDP lidar profile, return count, RANGE value mask and packet frame IDs
  (16-bit for standard and legacy layouts, 32-bit for FUSA). The encoder rejects dual-return
  profiles and ranges the wire layout cannot represent exactly.
* The normalized optical return separates the physical path length, which attenuates the signal,
  from the reported range, which drives detection, noise and the packet.
* [BUGFIX]: a failed assembler reset keeps the partial pre-reset revolution and its identity
  instead of discarding them.
* A ``COLCON_IGNORE`` marker keeps ROS workspace scans from building the embedded core as a
  package.

0.1.0 (2026-08-27)
------------------
* First version: calibrated firing geometry with step-independent scheduling, revolution and
  firing identity, Ouster SDK-backed metadata parsing, transactional revolution assembly,
  row-major image layout, and standard packet encoding with headers and modern CRC.
