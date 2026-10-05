// Copyright 2026 John C. Furey
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Minimal, dependency-free pcap reader for wire-conformance fixtures.
//
// It understands classic pcap files with Ethernet link types, optional single
// 802.1Q tags, IPv4, and UDP, and reassembles fragmented IPv4 datagrams (an
// Ouster lidar packet is far larger than an Ethernet MTU). It is test support
// only: it validates every length it reads and throws on malformed input.

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace ouster_sim_core::test_support {

struct UdpDatagram {
    std::uint16_t destination_port = 0;
    std::uint64_t capture_time_ns = 0;
    std::vector<std::uint8_t> payload;
};

namespace detail {

inline std::uint16_t bigEndian16(const std::uint8_t * data)
{
    return static_cast<std::uint16_t>((data[0] << 8) | data[1]);
}

inline std::uint32_t readU32(const std::uint8_t * data, bool little_endian)
{
    return little_endian
        ? static_cast<std::uint32_t>(data[0]) |
              (static_cast<std::uint32_t>(data[1]) << 8) |
              (static_cast<std::uint32_t>(data[2]) << 16) |
              (static_cast<std::uint32_t>(data[3]) << 24)
        : (static_cast<std::uint32_t>(data[0]) << 24) |
              (static_cast<std::uint32_t>(data[1]) << 16) |
              (static_cast<std::uint32_t>(data[2]) << 8) |
              static_cast<std::uint32_t>(data[3]);
}

struct Fragment {
    std::vector<std::uint8_t> bytes;
    bool more_fragments = false;
};

}  // namespace detail

inline std::vector<UdpDatagram> readUdpDatagrams(const std::string & path)
{
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        throw std::runtime_error("cannot open pcap fixture '" + path + "'");
    }
    const std::vector<std::uint8_t> file{
        std::istreambuf_iterator<char>(stream),
        std::istreambuf_iterator<char>()};
    if (file.size() < 24) {
        throw std::runtime_error("pcap fixture '" + path + "' has no header");
    }

    bool little_endian = false;
    bool nanosecond = false;
    const std::uint32_t magic = detail::readU32(file.data(), true);
    if (magic == 0xa1b2c3d4u || magic == 0xa1b23c4du) {
        little_endian = true;
        nanosecond = magic == 0xa1b23c4du;
    } else if (magic == 0xd4c3b2a1u || magic == 0x4d3cb2a1u) {
        nanosecond = magic == 0x4d3cb2a1u;
    } else {
        throw std::runtime_error("unsupported pcap magic in '" + path + "'");
    }
    if (detail::readU32(file.data() + 20, little_endian) != 1u) {
        throw std::runtime_error("pcap fixture '" + path + "' is not Ethernet");
    }

    using FragmentKey = std::tuple<std::uint32_t, std::uint32_t, std::uint16_t>;
    std::map<FragmentKey, std::map<std::size_t, detail::Fragment>> pending;
    std::vector<UdpDatagram> datagrams;

    std::size_t offset = 24;
    while (offset + 16 <= file.size()) {
        const std::uint64_t seconds = detail::readU32(&file[offset], little_endian);
        const std::uint64_t fraction =
            detail::readU32(&file[offset + 4], little_endian);
        const std::size_t captured =
            detail::readU32(&file[offset + 8], little_endian);
        offset += 16;
        if (captured > file.size() - offset) {
            throw std::runtime_error("truncated pcap record in '" + path + "'");
        }
        const std::uint8_t * frame = &file[offset];
        offset += captured;
        const std::uint64_t time_ns =
            seconds * 1'000'000'000ull + fraction * (nanosecond ? 1u : 1000u);

        std::size_t ip_offset = 14;
        if (captured < ip_offset) {
            continue;
        }
        std::uint16_t ether_type = detail::bigEndian16(frame + 12);
        if (ether_type == 0x8100u) {
            ip_offset = 18;
            if (captured < ip_offset) {
                continue;
            }
            ether_type = detail::bigEndian16(frame + 16);
        }
        if (ether_type != 0x0800u || captured < ip_offset + 20) {
            continue;
        }
        const std::uint8_t * ip = frame + ip_offset;
        const std::size_t header_bytes = static_cast<std::size_t>(ip[0] & 0x0fu) * 4u;
        const std::size_t total_bytes = detail::bigEndian16(ip + 2);
        if ((ip[0] >> 4) != 4u || header_bytes < 20 ||
            total_bytes < header_bytes || ip_offset + total_bytes > captured) {
            throw std::runtime_error("malformed IPv4 header in '" + path + "'");
        }
        if (ip[9] != 17u) {
            continue;
        }
        const std::uint16_t flags_offset = detail::bigEndian16(ip + 6);
        const FragmentKey key{
            detail::readU32(ip + 12, false), detail::readU32(ip + 16, false),
            detail::bigEndian16(ip + 4)};
        auto & fragments = pending[key];
        fragments[static_cast<std::size_t>(flags_offset & 0x1fffu) * 8u] = {
            std::vector<std::uint8_t>(ip + header_bytes, ip + total_bytes),
            (flags_offset & 0x2000u) != 0u};

        std::size_t expected = 0;
        bool complete = false;
        for (const auto & [fragment_offset, fragment] : fragments) {
            if (fragment_offset != expected) {
                break;
            }
            expected += fragment.bytes.size();
            if (!fragment.more_fragments) {
                complete = true;
                break;
            }
        }
        if (!complete) {
            continue;
        }
        std::vector<std::uint8_t> udp;
        udp.reserve(expected);
        for (const auto & [fragment_offset, fragment] : fragments) {
            static_cast<void>(fragment_offset);
            udp.insert(udp.end(), fragment.bytes.begin(), fragment.bytes.end());
            if (!fragment.more_fragments) {
                break;
            }
        }
        pending.erase(key);
        if (udp.size() < 8) {
            throw std::runtime_error("truncated UDP header in '" + path + "'");
        }
        const std::size_t udp_bytes = detail::bigEndian16(udp.data() + 4);
        if (udp_bytes < 8 || udp_bytes > udp.size()) {
            throw std::runtime_error("malformed UDP length in '" + path + "'");
        }
        datagrams.push_back({
            detail::bigEndian16(udp.data() + 2), time_ns,
            std::vector<std::uint8_t>(udp.begin() + 8, udp.begin() + udp_bytes)});
    }
    return datagrams;
}

}  // namespace ouster_sim_core::test_support
