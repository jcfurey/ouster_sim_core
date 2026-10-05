// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

// Encoder throughput against the real-time budget of common sensor modes.
// Run a Release build; the output is informational, not a pass/fail gate.

#include "ouster_sim_core/metadata.hpp"
#include "ouster_sim_core/packet_encoder.hpp"
#include "support/conformance_v1.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <string>
#include <vector>

#ifndef OUSTER_SIM_CORE_TEST_DATA_DIR
#error "OUSTER_SIM_CORE_TEST_DATA_DIR must identify the core metadata fixtures"
#endif

namespace {

std::string resized(std::string json, unsigned width, unsigned hz)
{
    const auto replace_all = [&](const std::string & before,
                                 const std::string & after) {
        for (auto at = json.find(before); at != std::string::npos;
             at = json.find(before, at + after.size())) {
            json.replace(at, before.size(), after);
        }
    };
    const std::string mode = std::to_string(width) + "x" + std::to_string(hz);
    replace_all("\"1024x10\"", "\"" + mode + "\"");
    replace_all("\"columns_per_frame\": 1024",
                "\"columns_per_frame\": " + std::to_string(width));
    replace_all("1023\n    ]", std::to_string(width - 1) + "\n    ]");
    replace_all("\"fps\": 10", "\"fps\": " + std::to_string(hz));
    return json;
}

}  // namespace

int main()
{
    namespace fixture = ouster_sim_core::conformance_v1;
    const std::string path =
        std::string(OUSTER_SIM_CORE_TEST_DATA_DIR) + "/os1_64_rev7.json";
    struct Mode { unsigned width; unsigned hz; };
    std::printf("%-24s %-10s %8s %10s %10s %9s\n", "profile", "mode", "packets",
                "median_us", "p99_us", "budget_%");
    for (const char * profile : fixture::primaryProfiles) {
        for (const Mode mode : {Mode{512, 20}, Mode{1024, 10}, Mode{1024, 20},
                                Mode{2048, 10}}) {
            const auto metadata = ouster_sim_core::OusterMetadata::fromJson(
                resized(fixture::metadataJson(path, profile), mode.width, mode.hz));
            const ouster_sim_core::OusterPacketEncoder encoder(metadata);
            const auto frame = fixture::frame(metadata, 1, 1'000'000'000 / mode.hz);
            const auto view =
                ouster_sim_core::OusterScanFrameView::fromFrame(frame);
            std::vector<ouster_sim_core::EncodedLidarPacket> packets;
            encoder.encode(view, packets);  // warm storage

            std::vector<double> samples;
            for (int iteration = 0; iteration < 200; ++iteration) {
                const auto start = std::chrono::steady_clock::now();
                encoder.encode(view, packets);
                samples.push_back(std::chrono::duration<double, std::micro>(
                    std::chrono::steady_clock::now() - start).count());
            }
            std::sort(samples.begin(), samples.end());
            const double median = samples[samples.size() / 2];
            const double p99 = samples[samples.size() * 99 / 100];
            const double budget_us = 1e6 / mode.hz;
            std::printf("%-24s %4ux%-5u %8zu %10.1f %10.1f %9.2f\n", profile,
                        mode.width, mode.hz, packets.size(), median, p99,
                        100.0 * p99 / budget_us);
        }
    }
    return 0;
}
