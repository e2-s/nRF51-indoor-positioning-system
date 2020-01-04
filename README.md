# nRF51-indoor-positioning-system

For testing indoor positioning system, we need to have minimum of three BLE beacons with nRF51.Each beacon should be programmed with "ble_app_becon" example of nordic SDK.

Here we are using "RSSI" for finding the indoor position.So, after programming each beacon with "ble_app_beacon" calculate average RSSI for each beacon by placing each at 1M distance, which is used for calculating environmental constants.Substitute the average RSSI values in "RSSI-AVG-1M" array defined in the "main.c" file.Place the beacons on the wall in a such a way they will form a trinagular shape.Take a corner as origin and notedown the distance of the each beacon on X and Y axis from the origin.Substitute these X and Y axis vlaues in the "main.c" file of the this project, where macros are defined for X and Y axis of each beacon.Make sure to add the "MAC Id" of each beacon in 2D array named as "ref_node_entry" in "main.c" file.

We can use nRF51 or nRF52 DK as central device and flash this code in the DK board.Now the central device will start reading the RSSI from each beacon and will print the position of the device in terms of X and Y axis on the UART terminal.

