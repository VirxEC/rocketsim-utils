#[derive(Clone, Copy, Debug)]
struct LinearPiece {
    pub base_x: f32,
    pub base_y: f32,
    pub max_x: f32,
    pub max_y: f32,
    pub x_diff: f32,
    pub y_diff: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct LinearPieceCurve<const N: usize> {
    curve: [LinearPiece; N],
}

impl<const N: usize> LinearPieceCurve<N> {
    pub const fn new(value_mappings: [(f32, f32); N]) -> Self {
        let mut curve = [LinearPiece {
            base_x: 0.0,
            base_y: 0.0,
            max_x: 0.0,
            max_y: 0.0,
            x_diff: 0.0,
            y_diff: 0.0,
        }; N];

        curve[0].max_x = value_mappings[0].0;
        curve[0].max_y = value_mappings[0].1;

        let mut i = 1;
        while i < N {
            let prev = &value_mappings[i - 1];
            let this = &value_mappings[i];

            curve[i].base_x = prev.0;
            curve[i].base_y = prev.1;
            curve[i].max_x = this.0;
            curve[i].max_y = this.1;
            curve[i].x_diff = this.0 - prev.0;
            curve[i].y_diff = this.1 - prev.1;

            i += 1;
        }

        Self { curve }
    }

    /// # Arguments
    ///
    /// * `input` - The input to the curve
    pub const fn get_output(&self, input: f32) -> f32 {
        if input <= self.curve[0].max_x {
            return self.curve[0].max_y;
        }

        let mut i = 1;
        while i < N {
            let pair = self.curve[i];
            if pair.max_x > input {
                let interp_frac = (input - pair.base_x) / pair.x_diff;
                return pair.y_diff * interp_frac + pair.base_y;
            }
            i += 1;
        }

        self.curve[N - 1].max_y
    }
}
