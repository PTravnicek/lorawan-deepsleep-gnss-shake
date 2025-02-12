let incoming = msg.payload;

// 1) Decode the hex "data" field into a 10-byte Buffer.
//    - Byte 0: Battery percentage
//    - Byte 1: Wake-up reason (esp_sleep_get_wakeup_cause())
//    - Bytes 2..5: Latitude (signed 32-bit, microdegrees)
//    - Bytes 6..9: Longitude (signed 32-bit, microdegrees)
let dataHex = incoming.data;               // e.g. "550102fc88b400db8fd1" (example hex)
let bytes = Buffer.from(dataHex, "hex");   // Convert hex to Buffer

// Battery (first byte)
let battery = bytes[0];  // e.g. 0x55 = 85 decimal

// Wake-up reason (second byte)
let wakeUpReason = bytes[1]; 

// Latitude (signed 32-bit, big-endian at bytes[2..5])
let latInt = bytes.readInt32BE(2);
// Longitude (signed 32-bit, big-endian at bytes[6..9])
let lonInt = bytes.readInt32BE(6);

// Convert to float degrees
let latitude = latInt / 1_000_000;
let longitude = lonInt / 1_000_000;

// 2) Count gateways
let gatewayCount = (incoming.gws || []).length;

// 2a) Compute bestRSSI (max RSSI from the gateways)
let bestRSSI = (incoming.gws || []).reduce(
    (acc, gw) => Math.max(acc, gw.rssi),
    Number.NEGATIVE_INFINITY
);

// 3) Create an array of measurement objects for InfluxDB
//    (One measurement for the device info)
let deviceMeasurement = {
    measurement: "device_data",  // <-- rename as you like
    tags: {
        deviceEUI: incoming.EUI || "unknown_device"
    },
    fields: {
        battery: battery,
        wakeUpReason: wakeUpReason,  // New field added from byte 1
        latitude: latitude,
        longitude: longitude,
        gatewayCount: gatewayCount,
        bestRSSI: bestRSSI,
        // Add a new field called 'gates'
        gates: gatewayCount
    },
    timestamp: new Date() // or use incoming.ts for server-provided timestamp
};

// Optionally, store each gateway's RSSI/SNR as separate points:
let gatewayMeasurements = (incoming.gws || []).map(gateway => ({
    measurement: "gateway_data",  // <-- rename as you like
    tags: {
        deviceEUI: incoming.EUI || "unknown_device",
        gatewayEUI: gateway.gweui
    },
    fields: {
        rssi: gateway.rssi,
        snr: gateway.snr
    },
    timestamp: new Date()
}));

// Combine into a single array
msg.payload = [deviceMeasurement, ...gatewayMeasurements];

// Return to InfluxDB Out node
return msg;
