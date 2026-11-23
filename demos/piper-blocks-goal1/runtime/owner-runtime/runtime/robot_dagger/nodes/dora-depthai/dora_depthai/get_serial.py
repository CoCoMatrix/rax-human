"""Get serial numbers of all detected OAK cameras."""

import depthai as dai


def get_all_serial_numbers():
    """
    Get serial numbers of all connected OAK cameras.

    Returns:
        list: List of serial numbers as strings
    """
    devices = dai.Device.getAllAvailableDevices()
    serial_numbers = []

    if not devices:
        print("No OAK cameras detected.")
        return serial_numbers

    print(f"Found {len(devices)} OAK camera(s):\n")

    for i, device in enumerate(devices, 1):
        serial = device.deviceId
        serial_numbers.append(serial)
        print(f"{i}. Serial Number: {serial}")
        print(f"   State: {device.state.name}")
        print(f"   Protocol: {device.protocol.name}")
        if hasattr(device, "name") and device.name:
            print(f"   Name: {device.name}")
        if hasattr(device, "platform"):
            print(f"   Platform: {device.platform.name}")
        print()

    return serial_numbers


def get_device_info():
    """
    Get detailed information about all connected OAK cameras.

    Returns:
        list: List of dictionaries containing device information
    """
    devices = dai.Device.getAllAvailableDevices()
    device_info_list = []

    if not devices:
        print("No OAK cameras detected.")
        return device_info_list

    print(f"Found {len(devices)} OAK camera(s):\n")

    for i, device in enumerate(devices, 1):
        info = {
            "serial": device.deviceId,
            "state": device.state.name,
            "protocol": device.protocol.name,
            "name": device.name if device.name else "N/A",
            "platform": device.platform.name if hasattr(device, "platform") else "N/A",
        }
        device_info_list.append(info)

        print(f"{i}. Camera Details:")
        print(f"   Serial Number: {info['serial']}")
        print(f"   State: {info['state']}")
        print(f"   Protocol: {info['protocol']}")
        print(f"   Name: {info['name']}")
        print(f"   Platform: {info['platform']}")
        print()

    return device_info_list


def main():
    """Main function to display all detected OAK cameras."""
    print("=" * 60)
    print("OAK Camera Detection")
    print("=" * 60)
    print()

    serial_numbers = get_all_serial_numbers()

    if serial_numbers:
        print("\nSerial numbers only:")
        for serial in serial_numbers:
            print(f"  - {serial}")

        print("\n" + "=" * 60)
        print(f"CAMERA_SERIAL={serial_numbers[0]}")
        print("=" * 60)


if __name__ == "__main__":
    main()
