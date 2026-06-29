#!/usr/bin/env bash
# SHM / best-effort end-to-end test — reproduces the robot lidar pattern:
# a bounded zero-copy PointCloud2 published over SHM with best-effort QoS.
# Auto-discovery records it (domain 94); playback replays it (domain 95).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${HERE}/../build"
REC=94; PLAY=95; N=12; BAG="${BUILD}/shm.mcap"

cd "${BUILD}" || { echo "build dir missing"; exit 2; }
rm -f "${BAG}"

echo "== 1. record --all (SHM enabled) on domain ${REC} =="
./dds_recorder --all --domain ${REC} --output "${BAG}" --shm-mb 128 &
REC_PID=$!; sleep 2

echo "== 2. publish UDP + SHM-best-effort cloud (domain ${REC}) =="
./test_publisher --shm --domain ${REC} --count ${N} --warmup 1.5

sleep 1; kill -INT ${REC_PID}; wait ${REC_PID} 2>/dev/null
if [ ! -s "${BAG}" ]; then echo "FAIL: no bag"; exit 1; fi

echo "== 3. subscriber --shm on domain ${PLAY} =="
./test_subscriber --shm --domain ${PLAY} --expect ${N} --seconds 12 &
SUB_PID=$!; sleep 2

echo "== 4. replay (SHM-aware) on domain ${PLAY} =="
./dds_player "${BAG}" --domain ${PLAY} --warmup 2 --shm-mb 128

wait ${SUB_PID}; RESULT=$?
echo "== result: $([ ${RESULT} -eq 0 ] && echo PASS || echo FAIL) =="
exit ${RESULT}
