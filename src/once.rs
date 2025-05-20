pub struct OverrideOnce<T> {
    once: Option<T>,
    default: T,
}

impl<T> OverrideOnce<T> {
    pub fn new(default: T) -> Self {
        Self {
            once: None,
            default,
        }
    }

    pub fn set(&mut self, default: T) {
        self.default = default;
    }

    pub fn once(&mut self, once: T) {
        self.once = Some(once);
    }

    pub fn once_with(&mut self, f: impl FnOnce() -> T) {
        self.once = Some(f());
    }

    pub fn get(&self) -> &T {
        self.once.as_ref().unwrap_or(&self.default)
    }
}
