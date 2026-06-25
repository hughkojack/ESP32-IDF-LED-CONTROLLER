# CAN OTA (mechanical nodes)

Node firmware is updated from a **laptop** running `can_ota_bench.py`. The hub forwards raw CAN frames only (exclusive tunnel); it does not implement the OTA protocol.

## Remote (hub tunnel)

1. Open the CAN tunnel from the hub UI (**Nodes** tab) or `POST /api/can/tunnel/open`.
2. From your laptop:

```bash
pip install python-can
python tools/can_ota_bench.py --tunnel http://<hub-ip> --node-id 4 path/to/firmware.bin
```

3. Close the tunnel when finished (`POST /api/can/tunnel/close` or UI).

Wire format: TCP port **5250**, 13-byte frames: `0xCA 0xFE` + CAN ID (u16 LE) + DLC + 8 data bytes.

## Local (USB-CAN at panel)

```bash
python tools/can_ota_bench.py --interface slcan --channel COM9 --node-id 4 firmware.bin
```

## Hub firmware

Hub self-OTA remains at `POST /api/ota` (Advanced tab).
