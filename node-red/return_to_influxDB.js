let incoming = msg.payload;

// 1) Decode the hex "data" field into an 11-byte Buffer.
//    - Byte 0: Battery percentage
//    - Byte 1: GPS fix time in seconds
//    - Byte 2: Wake-up reason (esp_sleep_get_wakeup_cause())
//    - Bytes 3..6: Latitude (signed 32-bit, microdegrees)
//    - Bytes 7..10: Longitude (signed 32-bit, microdegrees)
let dataHex = incoming.data;               // e.g. "550102fc88b400db8fd1" (example hex)
let bytes = Buffer.from(dataHex, "hex");   // Convert hex to Buffer

// Battery (first byte)
let battery = bytes[0];  // e.g. 0x55 = 85 decimal

// GPS fix time in seconds (second byte)
let gpsFixTime = bytes[1];

// Wake-up reason (second byte)
let wakeUpReason = bytes[2]; 
let wakeUpReasonText = (() => {
    switch(wakeUpReason) {
        case 0: return "Normal Reset";
        case 1: return "All Wakeup Sources";
        case 2: return "Shake Detection";
        case 3: return "External Signal RTC";
        case 4: return "Timer";
        case 5: return "Touchpad";
        case 6: return "ULP Program";
        case 7: return "GPIO";
        case 8: return "UART";
        case 9: return "WIFI";
        case 10: return "COCPU Int";
        case 11: return "COCPU Crash";
        case 12: return "Bluetooth";
        default: return "Unknown";
    }
})();

// Latitude (signed 32-bit, big-endian at bytes[3..6])
let latInt = bytes.readInt32BE(3);
// Longitude (signed 32-bit, big-endian at bytes[7..10])
let lonInt = bytes.readInt32BE(7);

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
        gpsFixTime: gpsFixTime,  // Add GPS fix time in seconds
        wakeUpReason: wakeUpReason,  // Keep the numeric value
        wakeUpReasonText: wakeUpReasonText,  // Add the human-readable text
        latitude: latitude,
        longitude: longitude,
        gatewayCount: gatewayCount,
        bestRSSI: bestRSSI,
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
