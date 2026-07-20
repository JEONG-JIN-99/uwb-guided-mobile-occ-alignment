# Log schema

## Tx CSV exact order

```text
experiment_id
node_id
sample_index
nominal_elapsed_sec
actual_elapsed_sec
tx_uwb_azimuth_deg
tx_gimbal_command_deg
```

## Rx CSV exact order

```text
experiment_id
node_id
sample_index
nominal_elapsed_sec
actual_elapsed_sec
rx_uwb_azimuth_deg
rx_correction_deg
rx_gimbal_command_deg
rx_x_m
rx_y_m
rx_heading_deg
tx_x_m
tx_y_m
tx_heading_deg
qr_detected
```

## Meaning

- `nominal_elapsed_sec`: `sample_index × alignment_period_sec`
- `actual_elapsed_sec`: local monotonic elapsed time after the shared UTC
  experiment start
- `*_uwb_azimuth_deg`: relative azimuth snapshot used for that command
- `rx_correction_deg`: relative correction applied after deadband and per-sample limiting
- `*_gimbal_command_deg`: target yaw command, not measured output angle
- `rx_x_m`, `rx_y_m`, `rx_heading_deg`: Rx body pose in `map`
- `tx_x_m`, `tx_y_m`, `tx_heading_deg`: LiDAR-derived Tx pose in `map`
- `qr_detected`: target QR detection result for that sample

Missing observations are stored as empty CSV cells. Do not replace them with zero.

## Experiment timing modes

### Scheduled alignment

Entry points: `code/rx_main.py`, `code/tx_main.py`

- Tx and Rx wait for the same UTC start.
- Alignment runs at each monotonic target time.
- `nominal_elapsed_sec` is the scheduled alignment time.
- The newest queued UWB packet is used at the target time.

### Packet-immediate alignment

Entry points: `code/rx_packet_main.py`, `code/tx_packet_main.py`

- Tx and Rx wait for the same UTC start.
- Packets received before the UTC start are not used for alignment.
- Each latest valid packet received after the start is aligned as soon as it
  can be processed.
- `nominal_elapsed_sec` is `sample_index × nominal_period_sec` for comparison;
  it does not schedule the command.
- `actual_elapsed_sec` is the actual command time after the shared UTC start.
