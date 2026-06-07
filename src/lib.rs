//! # Clockwork Schedule
//!
//! Deterministic scheduling via clockwork mechanics.
//!
//! Models scheduling as clockwork: gears as periodic tasks, escapement
//! as the regulator, mainspring as pending work storage, chime as
//! notification, and synchronizer for multi-clock coordination.

/// Task with a period (gear).
pub mod gear {
    /// A gear represents a periodic task.
    #[derive(Debug, Clone)]
    pub struct Gear {
        name: String,
        period: u64,
        offset: u64,
        teeth: u64,
    }

    impl Gear {
        /// Create a new gear.
        pub fn new(name: &str, period: u64) -> Self {
            Self {
                name: name.to_string(),
                period: period.max(1),
                offset: 0,
                teeth: period.max(1),
            }
        }

        /// Get the name.
        pub fn name(&self) -> &str {
            &self.name
        }

        /// Get the period.
        pub fn period(&self) -> u64 {
            self.period
        }

        /// Set an offset (phase shift).
        pub fn with_offset(mut self, offset: u64) -> Self {
            self.offset = offset % self.period;
            self
        }

        /// Get the offset.
        pub fn offset(&self) -> u64 {
            self.offset
        }

        /// Check if the gear fires at a given tick.
        pub fn fires_at(&self, tick: u64) -> bool {
            tick >= self.offset && (tick - self.offset).is_multiple_of(self.period)
        }

        /// Next fire time after the given tick.
        pub fn next_fire(&self, after: u64) -> u64 {
            if after < self.offset {
                return self.offset;
            }
            let elapsed = after - self.offset;
            let remainder = elapsed % self.period;
            if remainder == 0 && after >= self.offset {
                after
            } else {
                after + self.period - remainder
            }
        }

        /// Compute the LCM of two periods (when gears align).
        pub fn lcm_period(&self, other: &Gear) -> u64 {
            lcm(self.period, other.period)
        }

        /// Compute the GCD of two periods.
        pub fn gcd_period(&self, other: &Gear) -> u64 {
            gcd(self.period, other.period)
        }

        /// Get number of teeth.
        pub fn teeth(&self) -> u64 {
            self.teeth
        }

        /// Set number of teeth.
        pub fn with_teeth(mut self, teeth: u64) -> Self {
            self.teeth = teeth.max(1);
            self
        }

        /// Rotation angle at a given tick (in teeth units).
        pub fn rotation(&self, tick: u64) -> u64 {
            tick % self.teeth
        }
    }

    fn gcd(a: u64, b: u64) -> u64 {
        let mut a = a;
        let mut b = b;
        while b != 0 {
            let t = b;
            b = a % b;
            a = t;
        }
        a
    }

    fn lcm(a: u64, b: u64) -> u64 {
        a / gcd(a, b) * b
    }

    /// A gear train (collection of gears).
    #[derive(Debug, Clone)]
    pub struct GearTrain {
        gears: Vec<Gear>,
    }

    impl GearTrain {
        /// Create a new gear train.
        pub fn new() -> Self {
            Self { gears: Vec::new() }
        }

        /// Add a gear.
        pub fn add(&mut self, gear: Gear) {
            self.gears.push(gear);
        }

        /// Get gears.
        pub fn gears(&self) -> &[Gear] {
            &self.gears
        }

        /// Number of gears.
        pub fn len(&self) -> usize {
            self.gears.len()
        }

        /// Check if empty.
        pub fn is_empty(&self) -> bool {
            self.gears.is_empty()
        }

        /// Find all gears that fire at a given tick.
        pub fn firing_at(&self, tick: u64) -> Vec<&Gear> {
            self.gears.iter().filter(|g| g.fires_at(tick)).collect()
        }

        /// Find the next tick where any gear fires.
        pub fn next_event(&self, after: u64) -> Option<(u64, Vec<&Gear>)> {
            if self.gears.is_empty() {
                return None;
            }
            // Search within a reasonable window
            let max_period = self.gears.iter().map(|g| g.period).max().unwrap_or(1);
            for tick in (after + 1)..=(after + max_period) {
                let firing = self.firing_at(tick);
                if !firing.is_empty() {
                    return Some((tick, firing));
                }
            }
            None
        }

        /// Compute when all gears align (LCM of all periods).
        pub fn alignment_period(&self) -> u64 {
            if self.gears.is_empty() {
                return 0;
            }
            self.gears.iter().skip(1).fold(self.gears[0].period, |acc, g| lcm(acc, g.period))
        }

        /// Simulate ticks and collect firing events.
        pub fn simulate(&self, ticks: u64) -> Vec<(u64, Vec<String>)> {
            let mut events = Vec::new();
            for t in 0..ticks {
                let firing: Vec<String> = self.firing_at(t).iter().map(|g| g.name().to_string()).collect();
                if !firing.is_empty() {
                    events.push((t, firing));
                }
            }
            events
        }
    }

    impl Default for GearTrain {
        fn default() -> Self {
            Self::new()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_gear_creation() {
            let g = Gear::new("tick", 5);
            assert_eq!(g.name(), "tick");
            assert_eq!(g.period(), 5);
        }

        #[test]
        fn test_gear_period_minimum() {
            let g = Gear::new("t", 0);
            assert_eq!(g.period(), 1);
        }

        #[test]
        fn test_gear_fires_at() {
            let g = Gear::new("t", 3);
            assert!(g.fires_at(0));
            assert!(!g.fires_at(1));
            assert!(!g.fires_at(2));
            assert!(g.fires_at(3));
        }

        #[test]
        fn test_gear_fires_with_offset() {
            let g = Gear::new("t", 3).with_offset(1);
            assert!(!g.fires_at(0));
            assert!(g.fires_at(1));
            assert!(!g.fires_at(2));
            assert!(!g.fires_at(3));
            assert!(g.fires_at(4));
        }

        #[test]
        fn test_next_fire() {
            let g = Gear::new("t", 5);
            assert_eq!(g.next_fire(3), 5);
            assert_eq!(g.next_fire(5), 5);
            assert_eq!(g.next_fire(6), 10);
        }

        #[test]
        fn test_lcm_period() {
            let a = Gear::new("a", 4);
            let b = Gear::new("b", 6);
            assert_eq!(a.lcm_period(&b), 12);
        }

        #[test]
        fn test_gcd_period() {
            let a = Gear::new("a", 12);
            let b = Gear::new("b", 8);
            assert_eq!(a.gcd_period(&b), 4);
        }

        #[test]
        fn test_teeth() {
            let g = Gear::new("t", 5).with_teeth(10);
            assert_eq!(g.teeth(), 10);
        }

        #[test]
        fn test_rotation() {
            let g = Gear::new("t", 5).with_teeth(10);
            assert_eq!(g.rotation(3), 3);
            assert_eq!(g.rotation(13), 3);
        }

        #[test]
        fn test_gear_train_empty() {
            let gt = GearTrain::new();
            assert!(gt.is_empty());
        }

        #[test]
        fn test_gear_train_add() {
            let mut gt = GearTrain::new();
            gt.add(Gear::new("a", 2));
            gt.add(Gear::new("b", 3));
            assert_eq!(gt.len(), 2);
        }

        #[test]
        fn test_firing_at() {
            let mut gt = GearTrain::new();
            gt.add(Gear::new("a", 2));
            gt.add(Gear::new("b", 3));
            let firing = gt.firing_at(6);
            assert_eq!(firing.len(), 2);
        }

        #[test]
        fn test_next_event() {
            let mut gt = GearTrain::new();
            gt.add(Gear::new("a", 5));
            let next = gt.next_event(3);
            assert!(next.is_some());
            assert_eq!(next.unwrap().0, 5);
        }

        #[test]
        fn test_alignment_period() {
            let mut gt = GearTrain::new();
            gt.add(Gear::new("a", 4));
            gt.add(Gear::new("b", 6));
            assert_eq!(gt.alignment_period(), 12);
        }

        #[test]
        fn test_alignment_period_empty() {
            let gt = GearTrain::new();
            assert_eq!(gt.alignment_period(), 0);
        }

        #[test]
        fn test_simulate() {
            let mut gt = GearTrain::new();
            gt.add(Gear::new("a", 2));
            let events = gt.simulate(6);
            assert_eq!(events.len(), 3); // at 0, 2, 4
        }

        #[test]
        fn test_gear_train_default() {
            let gt = GearTrain::default();
            assert!(gt.is_empty());
        }
    }
}

/// Regulates when tasks fire (escapement).
pub mod escapement {
    use super::gear::{Gear, GearTrain};

    /// Escapement mode.
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum EscapementMode {
        /// Fire all ready gears.
        All,
        /// Fire only the highest priority (smallest period) gear.
        PriorityFirst,
        /// Fire at most N gears per tick.
        Limited(usize),
    }

    /// Result of an escapement tick.
    #[derive(Debug, Clone)]
    pub struct TickResult {
        pub tick: u64,
        pub fired: Vec<String>,
        pub skipped: Vec<String>,
    }

    /// An escapement that regulates gear firing.
    #[derive(Debug, Clone)]
    pub struct Escapement {
        mode: EscapementMode,
        gear_train: GearTrain,
        last_tick: u64,
    }

    impl Escapement {
        /// Create a new escapement.
        pub fn new(mode: EscapementMode) -> Self {
            Self {
                mode,
                gear_train: GearTrain::new(),
                last_tick: 0,
            }
        }

        /// Add a gear.
        pub fn add_gear(&mut self, gear: Gear) {
            self.gear_train.add(gear);
        }

        /// Advance to the next tick and return results.
        pub fn tick(&mut self) -> TickResult {
            self.last_tick += 1;
            self.evaluate(self.last_tick)
        }

        /// Evaluate a specific tick.
        pub fn evaluate(&self, tick: u64) -> TickResult {
            let ready = self.gear_train.firing_at(tick);
            let ready_names: Vec<&Gear> = ready;

            let (fired, skipped) = match self.mode {
                EscapementMode::All => {
                    let fired: Vec<String> = ready_names.iter().map(|g| g.name().to_string()).collect();
                    (fired, vec![])
                }
                EscapementMode::PriorityFirst => {
                    if let Some(best) = ready_names.iter().min_by_key(|g| g.period()) {
                        let fired = vec![best.name().to_string()];
                        let skipped: Vec<String> = ready_names
                            .iter()
                            .filter(|g| g.name() != best.name())
                            .map(|g| g.name().to_string())
                            .collect();
                        (fired, skipped)
                    } else {
                        (vec![], vec![])
                    }
                }
                EscapementMode::Limited(n) => {
                    let mut sorted: Vec<&Gear> = ready_names;
                    sorted.sort_by_key(|g| g.period());
                    let fired: Vec<String> = sorted.iter().take(n).map(|g| g.name().to_string()).collect();
                    let skipped: Vec<String> = sorted.iter().skip(n).map(|g| g.name().to_string()).collect();
                    (fired, skipped)
                }
            };

            TickResult {
                tick,
                fired,
                skipped,
            }
        }

        /// Get current tick.
        pub fn current_tick(&self) -> u64 {
            self.last_tick
        }

        /// Get the mode.
        pub fn mode(&self) -> &EscapementMode {
            &self.mode
        }

        /// Get the gear train.
        pub fn gear_train(&self) -> &GearTrain {
            &self.gear_train
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_escapement_all_mode() {
            let mut esc = Escapement::new(EscapementMode::All);
            esc.add_gear(Gear::new("a", 2));
            esc.add_gear(Gear::new("b", 2));
            let result = esc.evaluate(2);
            assert_eq!(result.fired.len(), 2);
            assert!(result.skipped.is_empty());
        }

        #[test]
        fn test_escapement_priority_first() {
            let mut esc = Escapement::new(EscapementMode::PriorityFirst);
            esc.add_gear(Gear::new("fast", 2));
            esc.add_gear(Gear::new("slow", 5));
            let result = esc.evaluate(10);
            assert_eq!(result.fired.len(), 1);
            assert_eq!(result.fired[0], "fast");
            assert_eq!(result.skipped.len(), 1);
        }

        #[test]
        fn test_escapement_limited() {
            let mut esc = Escapement::new(EscapementMode::Limited(1));
            esc.add_gear(Gear::new("a", 1));
            esc.add_gear(Gear::new("b", 1));
            esc.add_gear(Gear::new("c", 1));
            let result = esc.evaluate(1);
            assert_eq!(result.fired.len(), 1);
            assert_eq!(result.skipped.len(), 2);
        }

        #[test]
        fn test_tick_advances() {
            let mut esc = Escapement::new(EscapementMode::All);
            esc.add_gear(Gear::new("a", 1));
            assert_eq!(esc.current_tick(), 0);
            esc.tick();
            assert_eq!(esc.current_tick(), 1);
            esc.tick();
            assert_eq!(esc.current_tick(), 2);
        }

        #[test]
        fn test_no_fire_at_empty_tick() {
            let esc = Escapement::new(EscapementMode::All);
            let result = esc.evaluate(7);
            assert!(result.fired.is_empty());
        }

        #[test]
        fn test_tick_result_tick_value() {
            let mut esc = Escapement::new(EscapementMode::All);
            esc.add_gear(Gear::new("a", 1));
            let result = esc.tick();
            assert_eq!(result.tick, 1);
        }

        #[test]
        fn test_mode_access() {
            let esc = Escapement::new(EscapementMode::All);
            assert_eq!(*esc.mode(), EscapementMode::All);
        }

        #[test]
        fn test_gear_train_access() {
            let mut esc = Escapement::new(EscapementMode::All);
            esc.add_gear(Gear::new("a", 1));
            assert_eq!(esc.gear_train().len(), 1);
        }

        #[test]
        fn test_priority_nothing_to_skip() {
            let mut esc = Escapement::new(EscapementMode::PriorityFirst);
            esc.add_gear(Gear::new("a", 2));
            let result = esc.evaluate(2);
            assert_eq!(result.fired.len(), 1);
            assert!(result.skipped.is_empty());
        }
    }
}

/// Stores pending work (mainspring).
pub mod mainspring {
    /// A pending task in the mainspring.
    #[derive(Debug, Clone)]
    pub struct PendingTask {
        name: String,
        priority: u32,
        payload: String,
        created_at: u64,
    }

    impl PendingTask {
        /// Create a new pending task.
        pub fn new(name: &str, priority: u32, created_at: u64) -> Self {
            Self {
                name: name.to_string(),
                priority,
                payload: String::new(),
                created_at,
            }
        }

        /// With payload.
        pub fn with_payload(mut self, payload: &str) -> Self {
            self.payload = payload.to_string();
            self
        }

        /// Get name.
        pub fn name(&self) -> &str {
            &self.name
        }

        /// Get priority.
        pub fn priority(&self) -> u32 {
            self.priority
        }

        /// Get payload.
        pub fn payload(&self) -> &str {
            &self.payload
        }

        /// Get creation time.
        pub fn created_at(&self) -> u64 {
            self.created_at
        }

        /// Age of the task.
        pub fn age(&self, now: u64) -> u64 {
            now.saturating_sub(self.created_at)
        }
    }

    impl PartialEq for PendingTask {
        fn eq(&self, other: &Self) -> bool {
            self.name == other.name
        }
    }

    impl Eq for PendingTask {}

    impl PartialOrd for PendingTask {
        fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
            Some(self.cmp(other))
        }
    }

    impl Ord for PendingTask {
        fn cmp(&self, other: &Self) -> std::cmp::Ordering {
            // Higher priority first, then earlier creation time
            self.priority
                .cmp(&other.priority)
                .then_with(|| other.created_at.cmp(&self.created_at))
        }
    }

    /// The mainspring stores pending work.
    #[derive(Debug, Clone)]
    pub struct Mainspring {
        tasks: Vec<PendingTask>,
        capacity: usize,
    }

    impl Mainspring {
        /// Create a new mainspring with capacity.
        pub fn new(capacity: usize) -> Self {
            Self {
                tasks: Vec::new(),
                capacity,
            }
        }

        /// Wind (add) a task. Returns false if at capacity.
        pub fn wind(&mut self, task: PendingTask) -> bool {
            if self.tasks.len() >= self.capacity {
                return false;
            }
            self.tasks.push(task);
            true
        }

        /// Release (pop) the highest priority task.
        pub fn release(&mut self) -> Option<PendingTask> {
            if self.tasks.is_empty() {
                return None;
            }
            self.tasks.sort();
            self.tasks.pop()
        }

        /// Release all tasks.
        pub fn release_all(&mut self) -> Vec<PendingTask> {
            let tasks = self.tasks.clone();
            self.tasks.clear();
            tasks
        }

        /// Number of pending tasks.
        pub fn len(&self) -> usize {
            self.tasks.len()
        }

        /// Check if empty.
        pub fn is_empty(&self) -> bool {
            self.tasks.is_empty()
        }

        /// Check if full.
        pub fn is_full(&self) -> bool {
            self.tasks.len() >= self.capacity
        }

        /// Get capacity.
        pub fn capacity(&self) -> usize {
            self.capacity
        }

        /// Tension (utilization) of the mainspring.
        pub fn tension(&self) -> f64 {
            if self.capacity == 0 {
                return 0.0;
            }
            self.tasks.len() as f64 / self.capacity as f64
        }

        /// Peek at the highest priority task without removing.
        pub fn peek(&self) -> Option<&PendingTask> {
            self.tasks.iter().max()
        }

        /// Remove a task by name.
        pub fn remove(&mut self, name: &str) -> Option<PendingTask> {
            if let Some(pos) = self.tasks.iter().position(|t| t.name == name) {
                Some(self.tasks.remove(pos))
            } else {
                None
            }
        }

        /// Get tasks sorted by priority.
        pub fn sorted(&self) -> Vec<&PendingTask> {
            let mut tasks: Vec<&PendingTask> = self.tasks.iter().collect();
            tasks.sort();
            tasks
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_pending_task_creation() {
            let t = PendingTask::new("task1", 5, 100);
            assert_eq!(t.name(), "task1");
            assert_eq!(t.priority(), 5);
            assert_eq!(t.created_at(), 100);
        }

        #[test]
        fn test_pending_task_payload() {
            let t = PendingTask::new("t", 1, 0).with_payload("data");
            assert_eq!(t.payload(), "data");
        }

        #[test]
        fn test_pending_task_age() {
            let t = PendingTask::new("t", 1, 100);
            assert_eq!(t.age(150), 50);
        }

        #[test]
        fn test_pending_task_age_zero() {
            let t = PendingTask::new("t", 1, 100);
            assert_eq!(t.age(50), 0);
        }

        #[test]
        fn test_mainspring_creation() {
            let ms = Mainspring::new(10);
            assert!(ms.is_empty());
            assert_eq!(ms.capacity(), 10);
        }

        #[test]
        fn test_wind() {
            let mut ms = Mainspring::new(10);
            assert!(ms.wind(PendingTask::new("t1", 1, 0)));
            assert_eq!(ms.len(), 1);
        }

        #[test]
        fn test_wind_full() {
            let mut ms = Mainspring::new(2);
            ms.wind(PendingTask::new("t1", 1, 0));
            ms.wind(PendingTask::new("t2", 1, 0));
            assert!(!ms.wind(PendingTask::new("t3", 1, 0)));
        }

        #[test]
        fn test_release() {
            let mut ms = Mainspring::new(10);
            ms.wind(PendingTask::new("low", 1, 0));
            ms.wind(PendingTask::new("high", 10, 0));
            let released = ms.release().unwrap();
            assert_eq!(released.name(), "high");
        }

        #[test]
        fn test_release_empty() {
            let mut ms = Mainspring::new(10);
            assert!(ms.release().is_none());
        }

        #[test]
        fn test_release_all() {
            let mut ms = Mainspring::new(10);
            ms.wind(PendingTask::new("t1", 1, 0));
            ms.wind(PendingTask::new("t2", 2, 0));
            let all = ms.release_all();
            assert_eq!(all.len(), 2);
            assert!(ms.is_empty());
        }

        #[test]
        fn test_is_full() {
            let mut ms = Mainspring::new(1);
            assert!(!ms.is_full());
            ms.wind(PendingTask::new("t", 1, 0));
            assert!(ms.is_full());
        }

        #[test]
        fn test_tension() {
            let mut ms = Mainspring::new(4);
            ms.wind(PendingTask::new("t", 1, 0));
            assert!((ms.tension() - 0.25).abs() < 1e-10);
        }

        #[test]
        fn test_peek() {
            let mut ms = Mainspring::new(10);
            ms.wind(PendingTask::new("low", 1, 0));
            ms.wind(PendingTask::new("high", 5, 0));
            assert_eq!(ms.peek().unwrap().name(), "high");
            assert_eq!(ms.len(), 2);
        }

        #[test]
        fn test_remove() {
            let mut ms = Mainspring::new(10);
            ms.wind(PendingTask::new("t1", 1, 0));
            ms.wind(PendingTask::new("t2", 2, 0));
            let removed = ms.remove("t1");
            assert!(removed.is_some());
            assert_eq!(ms.len(), 1);
        }

        #[test]
        fn test_remove_not_found() {
            let mut ms = Mainspring::new(10);
            assert!(ms.remove("missing").is_none());
        }

        #[test]
        fn test_sorted() {
            let mut ms = Mainspring::new(10);
            ms.wind(PendingTask::new("low", 1, 0));
            ms.wind(PendingTask::new("mid", 3, 0));
            ms.wind(PendingTask::new("high", 5, 0));
            let sorted = ms.sorted();
            // sorted() sorts ascending by Ord (high priority = greater = last)
            assert_eq!(sorted.last().unwrap().name(), "high");
            assert_eq!(sorted.first().unwrap().name(), "low");
        }
    }
}

/// Notification at scheduled time (chime).
pub mod chime {
    /// A chime notification.
    #[derive(Debug, Clone)]
    pub struct Chime {
        id: String,
        message: String,
        scheduled_tick: u64,
        repeat: Option<u64>,
    }

    impl Chime {
        /// Create a one-time chime.
        pub fn once(id: &str, message: &str, scheduled_tick: u64) -> Self {
            Self {
                id: id.to_string(),
                message: message.to_string(),
                scheduled_tick,
                repeat: None,
            }
        }

        /// Create a repeating chime.
        pub fn repeating(id: &str, message: &str, start_tick: u64, interval: u64) -> Self {
            Self {
                id: id.to_string(),
                message: message.to_string(),
                scheduled_tick: start_tick,
                repeat: Some(interval.max(1)),
            }
        }

        /// Get the ID.
        pub fn id(&self) -> &str {
            &self.id
        }

        /// Get the message.
        pub fn message(&self) -> &str {
            &self.message
        }

        /// Check if the chime should fire at a tick.
        pub fn fires_at(&self, tick: u64) -> bool {
            if tick < self.scheduled_tick {
                return false;
            }
            match self.repeat {
                Some(interval) => (tick - self.scheduled_tick).is_multiple_of(interval),
                None => tick == self.scheduled_tick,
            }
        }

        /// Check if repeating.
        pub fn is_repeating(&self) -> bool {
            self.repeat.is_some()
        }

        /// Get scheduled tick.
        pub fn scheduled_tick(&self) -> u64 {
            self.scheduled_tick
        }

        /// Get repeat interval.
        pub fn repeat_interval(&self) -> Option<u64> {
            self.repeat
        }
    }

    /// A collection of chimes.
    #[derive(Debug, Clone)]
    pub struct ChimeSet {
        chimes: Vec<Chime>,
    }

    impl ChimeSet {
        /// Create an empty chime set.
        pub fn new() -> Self {
            Self { chimes: Vec::new() }
        }

        /// Add a chime.
        pub fn add(&mut self, chime: Chime) {
            self.chimes.push(chime);
        }

        /// Get all chimes firing at a tick.
        pub fn firing_at(&self, tick: u64) -> Vec<&Chime> {
            self.chimes.iter().filter(|c| c.fires_at(tick)).collect()
        }

        /// Remove a chime by ID.
        pub fn remove(&mut self, id: &str) -> Option<Chime> {
            if let Some(pos) = self.chimes.iter().position(|c| c.id == id) {
                Some(self.chimes.remove(pos))
            } else {
                None
            }
        }

        /// Number of chimes.
        pub fn len(&self) -> usize {
            self.chimes.len()
        }

        /// Check if empty.
        pub fn is_empty(&self) -> bool {
            self.chimes.is_empty()
        }

        /// Simulate chimes over a range of ticks.
        pub fn simulate(&self, start: u64, end: u64) -> Vec<(u64, Vec<String>)> {
            let mut events = Vec::new();
            for tick in start..=end {
                let firing: Vec<String> = self.firing_at(tick).iter().map(|c| c.message.clone()).collect();
                if !firing.is_empty() {
                    events.push((tick, firing));
                }
            }
            events
        }

        /// Get all chimes.
        pub fn chimes(&self) -> &[Chime] {
            &self.chimes
        }
    }

    impl Default for ChimeSet {
        fn default() -> Self {
            Self::new()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_chime_once() {
            let c = Chime::once("c1", "hello", 5);
            assert_eq!(c.id(), "c1");
            assert_eq!(c.message(), "hello");
            assert!(!c.is_repeating());
        }

        #[test]
        fn test_chime_once_fires() {
            let c = Chime::once("c1", "hello", 5);
            assert!(!c.fires_at(4));
            assert!(c.fires_at(5));
            assert!(!c.fires_at(6));
        }

        #[test]
        fn test_chime_repeating() {
            let c = Chime::repeating("c1", "tick", 0, 3);
            assert!(c.is_repeating());
            assert!(c.fires_at(0));
            assert!(!c.fires_at(1));
            assert!(c.fires_at(3));
            assert!(c.fires_at(6));
        }

        #[test]
        fn test_chime_repeating_offset() {
            let c = Chime::repeating("c1", "tick", 2, 5);
            assert!(!c.fires_at(0));
            assert!(c.fires_at(2));
            assert!(c.fires_at(7));
            assert!(!c.fires_at(6));
        }

        #[test]
        fn test_chime_set_empty() {
            let cs = ChimeSet::new();
            assert!(cs.is_empty());
        }

        #[test]
        fn test_chime_set_add() {
            let mut cs = ChimeSet::new();
            cs.add(Chime::once("c1", "hello", 5));
            assert_eq!(cs.len(), 1);
        }

        #[test]
        fn test_chime_set_firing() {
            let mut cs = ChimeSet::new();
            cs.add(Chime::once("c1", "hello", 5));
            cs.add(Chime::once("c2", "world", 5));
            assert_eq!(cs.firing_at(5).len(), 2);
            assert_eq!(cs.firing_at(4).len(), 0);
        }

        #[test]
        fn test_chime_set_remove() {
            let mut cs = ChimeSet::new();
            cs.add(Chime::once("c1", "hello", 5));
            let removed = cs.remove("c1");
            assert!(removed.is_some());
            assert!(cs.is_empty());
        }

        #[test]
        fn test_chime_set_remove_missing() {
            let mut cs = ChimeSet::new();
            assert!(cs.remove("missing").is_none());
        }

        #[test]
        fn test_simulate() {
            let mut cs = ChimeSet::new();
            cs.add(Chime::once("c1", "hello", 2));
            cs.add(Chime::once("c2", "world", 4));
            let events = cs.simulate(0, 5);
            assert_eq!(events.len(), 2);
        }

        #[test]
        fn test_chime_set_default() {
            let cs = ChimeSet::default();
            assert!(cs.is_empty());
        }

        #[test]
        fn test_chime_scheduled_tick() {
            let c = Chime::once("c1", "hello", 42);
            assert_eq!(c.scheduled_tick(), 42);
        }

        #[test]
        fn test_chime_repeat_interval() {
            let c = Chime::repeating("c1", "tick", 0, 5);
            assert_eq!(c.repeat_interval(), Some(5));
        }

        #[test]
        fn test_chimes_access() {
            let mut cs = ChimeSet::new();
            cs.add(Chime::once("c1", "hello", 1));
            assert_eq!(cs.chimes().len(), 1);
        }
    }
}

/// Aligns multiple clocks (synchronizer).
pub mod synchronizer {
    

    /// A clock in the synchronization system.
    #[derive(Debug, Clone)]
    pub struct Clock {
        name: String,
        tick: u64,
        rate: u64,
    }

    impl Clock {
        /// Create a new clock.
        pub fn new(name: &str, rate: u64) -> Self {
            Self {
                name: name.to_string(),
                tick: 0,
                rate: rate.max(1),
            }
        }

        /// Get name.
        pub fn name(&self) -> &str {
            &self.name
        }

        /// Get current tick.
        pub fn tick(&self) -> u64 {
            self.tick
        }

        /// Get rate.
        pub fn rate(&self) -> u64 {
            self.rate
        }

        /// Advance the clock by one step.
        pub fn advance(&mut self) {
            self.tick += self.rate;
        }

        /// Reset to zero.
        pub fn reset(&mut self) {
            self.tick = 0;
        }

        /// Set to a specific tick.
        pub fn set_tick(&mut self, tick: u64) {
            self.tick = tick;
        }
    }

    /// Synchronizer for multiple clocks.
    #[derive(Debug, Clone)]
    pub struct Synchronizer {
        clocks: Vec<Clock>,
        master_rate: u64,
    }

    impl Synchronizer {
        /// Create a new synchronizer.
        pub fn new(master_rate: u64) -> Self {
            Self {
                clocks: Vec::new(),
                master_rate: master_rate.max(1),
            }
        }

        /// Add a clock.
        pub fn add_clock(&mut self, clock: Clock) {
            self.clocks.push(clock);
        }

        /// Get clocks.
        pub fn clocks(&self) -> &[Clock] {
            &self.clocks
        }

        /// Get mutable clocks.
        pub fn clocks_mut(&mut self) -> &mut Vec<Clock> {
            &mut self.clocks
        }

        /// Advance all clocks by one step.
        pub fn tick_all(&mut self) {
            for clock in &mut self.clocks {
                clock.advance();
            }
        }

        /// Find when all clocks align.
        pub fn alignment_tick(&self) -> u64 {
            if self.clocks.is_empty() {
                return 0;
            }
            let rates: Vec<u64> = self.clocks.iter().map(|c| c.rate).collect();
            rates.iter().skip(1).fold(rates[0], |acc, &r| lcm(acc, r))
        }

        /// Synchronize all clocks to the same tick.
        pub fn synchronize(&mut self) {
            let target = self.alignment_tick();
            for clock in &mut self.clocks {
                clock.set_tick(target);
            }
        }

        /// Check if all clocks are at the same tick.
        pub fn is_synchronized(&self) -> bool {
            if self.clocks.len() < 2 {
                return true;
            }
            let first = self.clocks[0].tick;
            self.clocks.iter().all(|c| c.tick == first)
        }

        /// Reset all clocks.
        pub fn reset_all(&mut self) {
            for clock in &mut self.clocks {
                clock.reset();
            }
        }

        /// Number of clocks.
        pub fn len(&self) -> usize {
            self.clocks.len()
        }

        /// Check if empty.
        pub fn is_empty(&self) -> bool {
            self.clocks.is_empty()
        }

        /// Get master rate.
        pub fn master_rate(&self) -> u64 {
            self.master_rate
        }

        /// Find the clock furthest ahead.
        pub fn leader(&self) -> Option<&Clock> {
            self.clocks.iter().max_by_key(|c| c.tick)
        }

        /// Find the clock furthest behind.
        pub fn laggard(&self) -> Option<&Clock> {
            self.clocks.iter().min_by_key(|c| c.tick)
        }

        /// Compute the maximum skew between clocks.
        pub fn skew(&self) -> u64 {
            if self.clocks.len() < 2 {
                return 0;
            }
            let max_tick = self.clocks.iter().map(|c| c.tick).max().unwrap_or(0);
            let min_tick = self.clocks.iter().map(|c| c.tick).min().unwrap_or(0);
            max_tick - min_tick
        }
    }

    fn gcd(a: u64, b: u64) -> u64 {
        let mut a = a;
        let mut b = b;
        while b != 0 {
            let t = b;
            b = a % b;
            a = t;
        }
        a
    }

    fn lcm(a: u64, b: u64) -> u64 {
        a / gcd(a, b) * b
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        #[test]
        fn test_clock_creation() {
            let c = Clock::new("main", 1);
            assert_eq!(c.name(), "main");
            assert_eq!(c.tick(), 0);
            assert_eq!(c.rate(), 1);
        }

        #[test]
        fn test_clock_advance() {
            let mut c = Clock::new("c", 5);
            c.advance();
            assert_eq!(c.tick(), 5);
            c.advance();
            assert_eq!(c.tick(), 10);
        }

        #[test]
        fn test_clock_reset() {
            let mut c = Clock::new("c", 5);
            c.advance();
            c.reset();
            assert_eq!(c.tick(), 0);
        }

        #[test]
        fn test_clock_set_tick() {
            let mut c = Clock::new("c", 1);
            c.set_tick(42);
            assert_eq!(c.tick(), 42);
        }

        #[test]
        fn test_clock_rate_minimum() {
            let c = Clock::new("c", 0);
            assert_eq!(c.rate(), 1);
        }

        #[test]
        fn test_synchronizer_creation() {
            let sync = Synchronizer::new(1);
            assert!(sync.is_empty());
            assert_eq!(sync.master_rate(), 1);
        }

        #[test]
        fn test_add_clocks() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.add_clock(Clock::new("b", 3));
            assert_eq!(sync.len(), 2);
        }

        #[test]
        fn test_tick_all() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.add_clock(Clock::new("b", 3));
            sync.tick_all();
            assert_eq!(sync.clocks()[0].tick(), 2);
            assert_eq!(sync.clocks()[1].tick(), 3);
        }

        #[test]
        fn test_alignment_tick() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 4));
            sync.add_clock(Clock::new("b", 6));
            assert_eq!(sync.alignment_tick(), 12);
        }

        #[test]
        fn test_alignment_tick_single() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 5));
            assert_eq!(sync.alignment_tick(), 5);
        }

        #[test]
        fn test_synchronize() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 4));
            sync.add_clock(Clock::new("b", 6));
            sync.tick_all();
            sync.synchronize();
            assert!(sync.is_synchronized());
            assert_eq!(sync.clocks()[0].tick(), 12);
        }

        #[test]
        fn test_is_synchronized_initially() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 1));
            sync.add_clock(Clock::new("b", 1));
            assert!(sync.is_synchronized());
        }

        #[test]
        fn test_is_not_synchronized() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.add_clock(Clock::new("b", 3));
            sync.tick_all();
            assert!(!sync.is_synchronized());
        }

        #[test]
        fn test_reset_all() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.tick_all();
            sync.reset_all();
            assert_eq!(sync.clocks()[0].tick(), 0);
        }

        #[test]
        fn test_leader() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.add_clock(Clock::new("b", 5));
            sync.tick_all();
            assert_eq!(sync.leader().unwrap().name(), "b");
        }

        #[test]
        fn test_laggard() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.add_clock(Clock::new("b", 5));
            sync.tick_all();
            assert_eq!(sync.laggard().unwrap().name(), "a");
        }

        #[test]
        fn test_skew() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 2));
            sync.add_clock(Clock::new("b", 5));
            sync.tick_all();
            assert_eq!(sync.skew(), 3);
        }

        #[test]
        fn test_skew_synchronized() {
            let mut sync = Synchronizer::new(1);
            sync.add_clock(Clock::new("a", 1));
            sync.add_clock(Clock::new("b", 1));
            assert_eq!(sync.skew(), 0);
        }
    }
}

pub use gear::{Gear, GearTrain};
pub use escapement::{Escapement, EscapementMode, TickResult};
pub use mainspring::{Mainspring, PendingTask};
pub use chime::{Chime, ChimeSet};
pub use synchronizer::{Clock, Synchronizer};
