#!/usr/bin/env bash
set -euo pipefail

if (( $# == 0 || $# > 3 )); then
	echo "Usage: $0 <pci_addr1> [pci_addr2] [pci_addr3]" >&2
	exit 1
fi

for module in vfio vfio_iommu_type1 vfio-pci; do
	modprobe "${module}"
done

relax_permissions() {
	local group_id="$1"
	local dev_path="/dev/vfio/${group_id}"
	if [[ -e "${dev_path}" ]]; then
		chmod 666 /dev/vfio/vfio "${dev_path}"
	fi
}

for pci_addr in "$@"; do
	sys_dev="/sys/bus/pci/devices/${pci_addr}"
	if [[ ! -d "${sys_dev}" ]]; then
		echo "Skipping ${pci_addr}: device not found"
		continue
	fi

	group_link="$(readlink "${sys_dev}/iommu_group" 2>/dev/null || true)"
	if [[ -z "${group_link}" ]]; then
		echo "Skipping ${pci_addr}: no iommu_group (IOMMU disabled?)"
		continue
	fi

	group_id="$(basename "${group_link}")"
	group_dir="/sys/kernel/iommu_groups/${group_id}/devices"
	echo "Processing IOMMU group ${group_id} (${pci_addr})"

	for dev in "${group_dir}"/*; do
		dev_addr="$(basename "${dev}")"
		if [[ -L "${dev}/driver" ]]; then
			current_driver="$(basename "$(readlink "${dev}/driver")")"
			if [[ "${current_driver}" != "vfio-pci" ]]; then
				echo "${dev_addr}" > "/sys/bus/pci/drivers/${current_driver}/unbind"
			fi
		fi
	done

	for dev in "${group_dir}"/*; do
		dev_addr="$(basename "${dev}")"
		echo "vfio-pci" > "${dev}/driver_override"
		echo "${dev_addr}" > /sys/bus/pci/drivers/vfio-pci/bind
		echo "" > "${dev}/driver_override"
	done

	relax_permissions "${group_id}"
	echo "Group ${group_id} bound to vfio-pci and permissions adjusted."
done
