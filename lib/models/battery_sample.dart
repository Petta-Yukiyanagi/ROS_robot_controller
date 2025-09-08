class BatterySample {
final DateTime t;
final double? voltage; // V
final double? current; // A (positive = charging; depends on publisher)
final double? percentage; // 0..1 (or null)
final double? charge; // Ah
final double? capacity; // Ah
final int? powerSupplyStatus; // sensor_msgs/msg/BatteryState power_supply_status
final int? chargingStateCode; // optional: /charging_state custom code


const BatterySample({
required this.t,
this.voltage,
this.current,
this.percentage,
this.charge,
this.capacity,
this.powerSupplyStatus,
this.chargingStateCode,
});
}






