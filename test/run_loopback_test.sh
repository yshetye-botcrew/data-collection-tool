#!/usr/bin/env bash
# End-to-end loopback test:
#   record (domain 88)  ->  loopback.mcap  ->  play (domain 89)  ->  verify
#
# Recording and playback use DIFFERENT DDS domains, so a PASS proves the data
# came out of the file, not from leftover live traffic.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${HERE}/../build"
REC_DOMAIN=88
PLAY_DOMAIN=89
N=15
BAG="${BUILD}/loopback.mcap"

cd "${BUILD}" || { echo "build dir missing — run cmake/make first"; exit 2; }
rm -f "${BAG}"

# RECORD_MODE=discover exercises automatic topic discovery (no topic list);
# anything else uses the explicit config list.
if [ "${RECORD_MODE:-config}" = "discover" ]; then
  echo "== 1. start recorder in AUTO-DISCOVERY mode (domain ${REC_DOMAIN}) =="
  ./dds_recorder --all --output "${BAG}" --domain ${REC_DOMAIN} &
else
  echo "== 1. start recorder with config list (domain ${REC_DOMAIN}) =="
  ./dds_recorder --config "${HERE}/../config/test_record.json" \
      --output "${BAG}" --domain ${REC_DOMAIN} &
fi
REC_PID=$!
sleep 2   # let the recorder's subscribers come up

echo "== 2. publish ${N} msgs/topic (domain ${REC_DOMAIN}) =="
./test_publisher --domain ${REC_DOMAIN} --count ${N} --warmup 1.5

sleep 1   # flush
echo "== 3. stop recorder =="
kill -INT ${REC_PID}; wait ${REC_PID} 2>/dev/null

if [ ! -s "${BAG}" ]; then echo "FAIL: no bag written"; exit 1; fi
echo "   bag size: $(stat -c%s "${BAG}") bytes"

echo "== 4. start subscriber (domain ${PLAY_DOMAIN}) =="
./test_subscriber --domain ${PLAY_DOMAIN} --expect ${N} --seconds 12 &
SUB_PID=$!
sleep 2   # let the subscriber discover

echo "== 5. play bag (domain ${PLAY_DOMAIN}) =="
./dds_player "${BAG}" --domain ${PLAY_DOMAIN} --warmup 2

wait ${SUB_PID}; RESULT=$?
echo "== result: $([ ${RESULT} -eq 0 ] && echo PASS || echo FAIL) =="
exit ${RESULT}
