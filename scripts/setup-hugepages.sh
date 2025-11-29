#!/usr/bin/env bash
set -euo pipefail

PAGES="${1:-1024}"

echo "Requesting ${PAGES} huge pages (2 MB each)..."
echo "${PAGES}" > /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

mkdir -p /mnt/huge
if ! mountpoint -q /mnt/huge; then
	mount -t hugetlbfs nodev /mnt/huge
fi
chmod 1777 /mnt/huge

grep -H 'Huge' /proc/meminfo
echo "Huge pages ready under /mnt/huge"
