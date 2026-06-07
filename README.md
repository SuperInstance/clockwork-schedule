# Clockwork Schedule

[![crates.io](https://img.shields.io/crates/v/clockwork-schedule.svg)](https://crates.io/crates/clockwork-schedule)
[![docs.rs](https://docs.rs/clockwork-schedule/badge.svg)](https://docs.rs/clockwork-schedule)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> **Deterministic scheduling via clockwork mechanics — gears, escapement, mainspring, chimes, and synchronization.**

---

## The Problem

Task scheduling is either too simple (cron) or too complex (distributed schedulers). Cron lacks composable periodicity — you can't easily ask "when do these three schedules next align?" Distributed schedulers are overkill for deterministic, single-node scheduling where you need precise control over timing and phase.

## Why This Exists

Clockwork Schedule models scheduling as mechanical clockwork:
- **Gear**: A periodic task with configurable period and phase offset
- **GearTrain**: A collection of gears that tracks when they align
- **Escapement**: The regulator that determines which gear fires next
- **Mainspring**: Pending work storage
- **Chime**: Notification when gears fire
- **Synchronizer**: Multi-clock coordination

The metaphor makes scheduling intuitive: you literally build a clock mechanism.

## Architecture

```
  ┌──────────────────────────────────────────┐
  │              Gear Train                  │
  │                                          │
  │  Gear A (period=5)   │││││││││││││││││  │
  │  Gear B (period=3)   │ │ │ │ │ │ │ │ │  │
  │  Gear C (period=7)   │  │  │  │  │  │   │
  │                                          │
  │  Fire at tick where gear alignment:      │
  │  LCM(5,3) = 15, LCM(5,3,7) = 105       │
  │                                          │
  │  next_event(after) → (tick, [gears])     │
  │  simulate(n) → [(tick, [gear_names])]    │
  └──────────────────────────────────────────┘
```

## Installation

```toml
[dependencies]
clockwork-schedule = "0.1"
```

## API Reference

### `Gear`

A periodic task with period and phase offset:

```rust
use clockwork_schedule::gear::Gear;

let g = Gear::new("health_check", 60); // every 60 ticks
assert!(g.fires_at(0));
assert!(g.fires_at(60));
assert!(!g.fires_at(30));

let offset = Gear::new("delayed_task", 10).with_offset(5);
assert!(offset.fires_at(5));
assert!(!offset.fires_at(0));

let next = g.next_fire(45); // next fire after tick 45
assert_eq!(next, 60);
```

### `GearTrain`

A collection of gears with alignment tracking:

```rust
use clockwork_schedule::gear::{Gear, GearTrain};

let mut train = GearTrain::new();
train.add(Gear::new("heartbeat", 5));
train.add(Gear::new("sync", 10));
train.add(Gear::new("daily", 60));

// When do all gears align?
let alignment = train.alignment_period(); // LCM(5, 10, 60) = 60

// Which gears fire at tick 10?
let firing = train.firing_at(10);
// heartbeat + sync

// Next event after tick 7
let next = train.next_event(7); // tick 10, [heartbeat, sync]
```

### Simulation

```rust
use clockwork_schedule::gear::{Gear, GearTrain};

let mut train = GearTrain::new();
train.add(Gear::new("fast", 3));
train.add(Gear::new("slow", 5));

let events = train.simulate(20);
for (tick, gears) in &events {
    println!("tick {}: {:?}", tick, gears);
}
```

## Usage Examples

### Example 1: Multi-Rate Scheduling

```rust
use clockwork_schedule::gear::*;

let mut train = GearTrain::new();
train.add(Gear::new("telemetry", 1));       // every tick
train.add(Gear::new("heartbeat", 10));      // every 10 ticks
train.add(Gear::new("snapshot", 60));       // every 60 ticks
train.add(Gear::new("backup", 3600));       // hourly

// Full alignment: LCM(1, 10, 60, 3600) = 3600
assert_eq!(train.alignment_period(), 3600);
```

### Example 2: Phase-Offset Scheduling

```rust
use clockwork_schedule::gear::*;

let a = Gear::new("agent_a_check", 5);
let b = Gear::new("agent_b_check", 5).with_offset(2);

// a fires at 0, 5, 10, 15...
// b fires at 2, 7, 12, 17...
// They never fire at the same tick
```

### Example 3: Simulate and Log

```rust
use clockwork_schedule::gear::*;

let mut train = GearTrain::new();
train.add(Gear::new("poll", 3));
train.add(Gear::new("report", 9));

for (tick, gears) in train.simulate(30) {
    for name in gears {
        println!("[tick {}] Executing: {}", tick, name);
    }
}
```

## Performance

| Operation | Complexity |
|-----------|-----------|
| `fires_at` | O(1) |
| `next_fire` | O(1) |
| LCM computation | O(log(min(a,b))) |
| `firing_at` | O(n) |
| `next_event` | O(max_period × n) |
| `simulate` | O(ticks × n) |

## License

Licensed under the [MIT License](LICENSE).

## Contributing

1. Fork the repository
2. Create a feature branch
3. Write tests
4. Push and open a Pull Request
