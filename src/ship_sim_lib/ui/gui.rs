// === GUI Libraries ===
use eframe::egui;
use egui::{Color32, Pos2, Shape, Stroke, Ui, Vec2};
use std::sync::{Arc, RwLock};

// Libraries for maths
use nalgebra::Vector3;

/// === SharedState ===
/// Holds all shared simulation state across threads and GUI
/// - ship_pos: Position of the ship in world coordinates
/// - ship_angle: Rotation angle of the ship in radians
/// - keyboard_state: Boolean state of W, A, S, D keys
#[derive(Clone, Default)]
pub struct SharedState {
    pub ship_pos: Arc<RwLock<[f32; 2]>>, // Ship position [x, y]
    pub ship_angle: Arc<RwLock<f32>>, // Ship orientation in radians
    pub ship_speed: Arc<RwLock<[f32; 2]>>, // Heading speed and yaw speed [m/s, °/s]

    pub key_state_w: Arc<RwLock<bool>>, // WASD state: [W, A, S, D]
    pub key_state_a: Arc<RwLock<bool>>, // WASD state: [W, A, S, D]
    pub key_state_s: Arc<RwLock<bool>>, // WASD state: [W, A, S, D]
    pub key_state_d: Arc<RwLock<bool>>, // WASD state: [W, A, S, D]

    pub frame_interval_ms: Arc<RwLock<u64>>, // fps in ms

    pub show_external_forces: Arc<RwLock<bool>>,
    pub wind_speed_max: Arc<RwLock<f32>>, // [m/s]
    pub wind_speed: Arc<RwLock<f32>>,  // [m/s]
    pub wind_angle: Arc<RwLock<f32>>, // [°]
    pub wind_noise: Arc<RwLock<f32>>, // [%]
    pub current_speed_max: Arc<RwLock<f32>>, // [m/s]
    pub current_speed: Arc<RwLock<f32>>,  // [m/s]
    pub current_angle: Arc<RwLock<f32>>, // [°]
    pub current_noise: Arc<RwLock<f32>>, // [%]
}

/// === draw_scene ===
/// Draws the simulation scene including:
/// - background grid
/// - ship shape
/// - mouse crosshair and coordinate labels
pub fn draw_scene(
    ui: &mut Ui,
    pos: [f32; 2],
    angle: f32,
    cam_offset: [f32; 2],
    zoom: f32,
    show_external_forces: bool,
    wind_speed_vector: Vector3<f32>,
    wind_noise: f32,
    current_speed_vector: Vector3<f32>,
    current_noise: f32,
) {
    // Allocate full window canvas for drawing
    let size = ui.available_size();
    let (rect, _) = ui.allocate_exact_size(size, egui::Sense::hover());
    let painter = ui.painter_at(rect);
    let origin = rect.left_top(); // Canvas top-left in screen space

    // Convert world coordinates to screen coordinates
    let to_screen = |world: [f32; 2]| -> Pos2 {
        Pos2::new(
            origin.x + (world[0] - cam_offset[0]) * zoom,
            origin.y + (world[1] - cam_offset[1]) * zoom,
        )
    };

    // === Draw grid background ===
    let base_spacing = 50.0; // world unit spacing
    let spacing = (base_spacing / zoom).clamp(1.0, 1000.0); // adjust spacing by zoom level
    let grid_color = Color32::DARK_GRAY; // line color

    // Compute visible grid bounds in world units
    let grid_min_x = cam_offset[0] - size.x / zoom;
    let grid_max_x = cam_offset[0] + size.x / zoom;
    let grid_min_y = cam_offset[1] - size.y / zoom;
    let grid_max_y = cam_offset[1] + size.y / zoom;

    // Draw vertical grid lines
    let mut x = (grid_min_x / spacing).floor() * spacing;
    while x <= grid_max_x {
        let a = to_screen([x, grid_min_y]);
        let b = to_screen([x, grid_max_y]);
        painter.line_segment([a, b], Stroke::new(0.5, grid_color));
        x += spacing;
    }

    // Draw horizontal grid lines
    let mut y = (grid_min_y / spacing).floor() * spacing;
    while y <= grid_max_y {
        let a = to_screen([grid_min_x, y]);
        let b = to_screen([grid_max_x, y]);
        painter.line_segment([a, b], Stroke::new(0.5, grid_color));
        y += spacing;
    }

    // === Draw mouse crosshair and label ===
    let response = ui.interact(rect, ui.id().with("canvas"), egui::Sense::hover());
    if let Some(mouse_pos) = response.hover_pos() {
        let world_mouse = [
            (mouse_pos.x - origin.x) / zoom + cam_offset[0],
            (mouse_pos.y - origin.y) / zoom + cam_offset[1],
        ];

        // Draw crosshair lines
        painter.line_segment(
            [Pos2::new(mouse_pos.x, rect.top()), Pos2::new(mouse_pos.x, rect.bottom())],
            Stroke::new(1.0, Color32::LIGHT_BLUE),
        );
        painter.line_segment(
            [Pos2::new(rect.left(), mouse_pos.y), Pos2::new(rect.right(), mouse_pos.y)],
            Stroke::new(1.0, Color32::LIGHT_BLUE),
        );

        // Show world coordinate label
        let label = format!("x: {:.1}, y: {:.1}", world_mouse[0], world_mouse[1]);
        painter.text(
            mouse_pos + Vec2::new(5.0, 5.0),
            egui::Align2::LEFT_TOP,
            label,
            egui::FontId::monospace(12.0),
            Color32::WHITE,
        );
    }

    // Draw external forces speed vectors
    if show_external_forces {
        let center_screen = rect.center();
        let origin_world = [
            (center_screen.x - origin.x) / zoom + cam_offset[0],
            (center_screen.y - origin.y) / zoom + cam_offset[1],
        ];
        let origin_screen = to_screen(origin_world);

        // === Draw wind speed vector ===
        // Calculate location of vector
        let length_scale_wind = (rect.height() * 0.01)/zoom; // 1% of screen height
        let thickness_factor = 0.2 * wind_noise;

        let wind_scaled = Vec2::new(
            wind_speed_vector.x * length_scale_wind,
            wind_speed_vector.y * length_scale_wind,
        );

        let target_world = [
            origin_world[0] + wind_scaled.x,
            origin_world[1] + wind_scaled.y,
        ];

        let target_screen = to_screen(target_world);

        let dir = (target_screen - origin_screen).normalized();
        let perp = Vec2::new(-dir.y, dir.x);
        let mut arrow_size = 20.0;
        arrow_size += thickness_factor * arrow_size;

        // Adjusted target for the line to leave space for the arrowhead
        let arrow_offset = dir * arrow_size;
        let line_end = target_screen - arrow_offset;
        let mut line_width = 5.0;
        line_width += thickness_factor * line_width;

        // Draw main line ending before the arrowhead
        painter.line_segment(
            [origin_screen, line_end],
            Stroke::new(line_width, Color32::GREEN),
        );

        // Draw arrowhead at the original target
        let p1 = target_screen;
        let p2 = target_screen - dir * arrow_size + perp * (arrow_size * 0.5);
        let p3 = target_screen - dir * arrow_size - perp * (arrow_size * 0.5);

        painter.add(Shape::convex_polygon(vec![p1, p2, p3], Color32::GREEN, Stroke::NONE));

        // === Draw current speed vector ===
        // Calculate location of vector
        let length_scale_current = (rect.height() * 0.1)/zoom; // 10% of screen height
        let thickness_factor = 0.2 * current_noise;

        let current_scaled = Vec2::new(
            current_speed_vector.x * length_scale_current,
            current_speed_vector.y * length_scale_current,
        );

        let target_world = [
            origin_world[0] + current_scaled.x,
            origin_world[1] + current_scaled.y,
        ];

        let target_screen = to_screen(target_world);

        let dir = (target_screen - origin_screen).normalized();
        let perp = Vec2::new(-dir.y, dir.x);
        let mut arrow_size = 20.0;
        arrow_size += thickness_factor * arrow_size;

        // Adjusted target for the line to leave space for the arrowhead
        let arrow_offset = dir * arrow_size;
        let line_end = target_screen - arrow_offset;
        let mut line_width = 5.0;
        line_width += thickness_factor * line_width;

        // Draw main line ending before the arrowhead
        painter.line_segment(
            [origin_screen, line_end],
            Stroke::new(line_width, Color32::BLUE),
        );

        // Draw arrowhead at the original target
        let p1 = target_screen;
        let p2 = target_screen - dir * arrow_size + perp * (arrow_size * 0.5);
        let p3 = target_screen - dir * arrow_size - perp * (arrow_size * 0.5);

        painter.add(Shape::convex_polygon(vec![p1, p2, p3], Color32::BLUE, Stroke::NONE));
    }
    
    // === Draw ship shape ===
    let ship_shape = [
        Vec2::new(-20.0, 0.0), // tip
        Vec2::new(10.0, -10.0), // left base
        Vec2::new(10.0, 10.0),  // right base
    ];

    // Transform ship shape by rotation and translation
    let transformed: Vec<Pos2> = ship_shape
        .iter()
        .map(|v| {
            let x = v.x * angle.cos() - v.y * angle.sin();
            let y = v.x * angle.sin() + v.y * angle.cos();
            to_screen([pos[0] + x, pos[1] + y])
        })
        .collect();

    // Draw ship polygon (fill + outline)
    painter.add(Shape::convex_polygon(transformed.clone(), Color32::RED, Stroke::NONE));
    painter.add(Shape::closed_line(transformed, Stroke::new(2.0, Color32::BLACK)));
}

// ====================
// SIMULATOR APP STATE
// ====================

/// Main GUI app structure managing camera and shared state
pub struct SimulatorWindow {
    pub state: SharedState,         // All shared state across app
    pub camera_offset: [f32; 2],    // Pan position of camera
    pub zoom: f32,                  // Zoom level (1.0 = normal scale)
}

impl Default for SimulatorWindow {
    fn default() -> Self {
        Self {
            state: SharedState::default(),
            camera_offset: [0.0, 0.0],
            zoom: 1.0,
        }
    }
}

impl eframe::App for SimulatorWindow {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // === Poll keyboard inputs into shared state ===
        ctx.input(|i| {
            let keys = [
                (egui::Key::W, &self.state.key_state_w),
                (egui::Key::A, &self.state.key_state_a),
                (egui::Key::S, &self.state.key_state_s),
                (egui::Key::D, &self.state.key_state_d),
            ];
        
            for (key, state_lock) in keys {
                {
                    let is_down = i.key_down(key);
                    let mut state = state_lock.write().unwrap();
                    *state = is_down;
                }
            }
        });
        // === GUI canvas with drag + zoom ===
        egui::CentralPanel::default().show(ctx, |ui| {
            // Allow camera panning by dragging ----------
            let response = ui.interact(ui.max_rect(), ui.id().with("canvas"), egui::Sense::drag());
            if response.dragged() {
                let delta = response.drag_delta();
                self.camera_offset[0] -= delta.x / self.zoom;
                self.camera_offset[1] -= delta.y / self.zoom;
            }

            // Zoom using scroll wheel ----------
            let scroll_delta = ui.input(|i| i.raw_scroll_delta.y);
            if scroll_delta != 0.0 {
                let zoom_factor = (1.0 + scroll_delta * 0.01).clamp(0.1, 10.0);
                let old_zoom = self.zoom;
                self.zoom *= zoom_factor;

                // Keep zoom centered on mouse
                if let Some(mouse_pos) = ui.ctx().pointer_hover_pos() {
                    let rect = ui.max_rect();
                    let origin = rect.left_top();
                    let screen_pos = [mouse_pos.x - origin.x, mouse_pos.y - origin.y];
                    let world_before = [
                        screen_pos[0] / old_zoom + self.camera_offset[0],
                        screen_pos[1] / old_zoom + self.camera_offset[1],
                    ];
                    let world_after = [
                        screen_pos[0] / self.zoom + self.camera_offset[0],
                        screen_pos[1] / self.zoom + self.camera_offset[1],
                    ];
                    self.camera_offset[0] += world_before[0] - world_after[0];
                    self.camera_offset[1] += world_before[1] - world_after[1];
                }
            }

            // Read shared state
            let pos = *self.state.ship_pos.read().unwrap();
            let angle = *self.state.ship_angle.read().unwrap();

            // Show debug info in panel ----------
            let ship_speed  = self.state.ship_speed.write().unwrap();

            ui.horizontal(|ui| {
                ui.label(format!("Ship X: {:.1} m", pos[0]));
                ui.label(format!("Ship Y: {:.1} m", -pos[1]));
                ui.label(format!("Ship θ: {:.2}°", angle.to_degrees()));
                ui.label(format!("Ship v: {:.2} m/s", ship_speed[0]));
                ui.label(format!("Ship ω: {:.3}°/s", ship_speed[1]));
                ui.label(format!("Zoom: {:.2}x", self.zoom));
                ui.label(format!("Pan X: {:.1}", self.camera_offset[0]));
                ui.label(format!("Pan Y: {:.1}", self.camera_offset[1]));
            });

            // Show external forces interface ----------
            ui.separator();
            ui.heading("External Forces");

            // Checkbox to toggle visibility
            {
                let mut show_external_forces = self.state.show_external_forces.write().unwrap();
                ui.checkbox(&mut *show_external_forces, "Show External Forces");
            }

            ui.horizontal(|ui| {
                // === Wind Column ===
                ui.vertical(|ui| {
                    ui.label("Wind");
            
                    let wind_speed_max = self.state.wind_speed_max.read().unwrap();
                    let mut wind_speed = self.state.wind_speed.write().unwrap();
                    ui.horizontal(|ui| {
                        ui.label("Speed:");
                        ui.add(egui::Slider::new(&mut *wind_speed, 0.0..=*wind_speed_max).text("m/s"));
                    });
            
                    let mut wind_angle = self.state.wind_angle.write().unwrap();
                    ui.horizontal(|ui| {
                        ui.label("Angle: ");
                        ui.add(egui::Slider::new(&mut *wind_angle, 0.0..=360.0).text("°"));
                    });

                    let mut wind_noise = self.state.wind_noise.write().unwrap();
                    ui.horizontal(|ui| {
                        ui.label("Noise:  ");
                        ui.add(egui::Slider::new(&mut *wind_noise, 0.0..=5.0).text("%"));
                    });
                });
            
                ui.add_space(40.0); // spacing between wind and current columns
            
                // === Current Column ===
                ui.vertical(|ui| {
                    ui.label("Current");
            
                    let current_speed_max = self.state.current_speed_max.read().unwrap();
                    let mut current_speed = self.state.current_speed.write().unwrap();
                    ui.horizontal(|ui| {
                        ui.label("Speed:");
                        ui.add(egui::Slider::new(&mut *current_speed, 0.0..=*current_speed_max).text("m/s"));
                    });
            
                    let mut current_angle = self.state.current_angle.write().unwrap();
                    ui.horizontal(|ui| {
                        ui.label("Angle: ");
                        ui.add(egui::Slider::new(&mut *current_angle, 0.0..=360.0).text("°"));
                    });

                    let mut current_noise = self.state.current_noise.write().unwrap();
                    ui.horizontal(|ui| {
                        ui.label("Noise:  ");
                        ui.add(egui::Slider::new(&mut *current_noise, 0.0..=1.0).text("%"));
                    });
                });
            });

            // Convert external forces to vectors
            let min_wind_length = 10.0;
            let wind_speed = *self.state.wind_speed.read().unwrap() + min_wind_length;
            let wind_angle_deg = *self.state.wind_angle.read().unwrap();
            let wind_angle_rad = wind_angle_deg.to_radians();
            let wind_speed_vector = Vector3::new(
                wind_speed * wind_angle_rad.cos(),
                -wind_speed * wind_angle_rad.sin(),
                0.0,
            );

            let min_current_length = 1.0;
            let current_speed = *self.state.current_speed.read().unwrap() + min_current_length;
            let current_angle_deg = *self.state.current_angle.read().unwrap();
            let current_angle_rad = current_angle_deg.to_radians();
            let current_speed_vector = Vector3::new(
                current_speed * current_angle_rad.cos(),
                -current_speed * current_angle_rad.sin(),
                0.0,
            );

            // render 2D space ----------
            ui.separator(); 

            let show_external_forces = *self.state.show_external_forces.read().unwrap();
            let wind_noise = *self.state.wind_noise.read().unwrap();
            let current_noise = *self.state.current_noise.read().unwrap() * 5.0; // Since wind noise % slider is x5 bigger, for consistent vectors compensate for it here

            draw_scene(
                ui,
                pos,
                angle,
                self.camera_offset,
                self.zoom,
                show_external_forces,
                wind_speed_vector,
                wind_noise,
                current_speed_vector,
                current_noise
            );
            
            // Wait a bit until next render
            let interval = *self.state.frame_interval_ms.read().unwrap();
            ctx.request_repaint_after(std::time::Duration::from_millis(interval)); 
        });
    }
}

/// === window ===
/// Launch GUI with provided shared state and run until closed
pub fn window(state: SharedState) {
    let options = eframe::NativeOptions::default();
    let _ = eframe::run_native(
        "Ship Simulator GUI",
        options,
        Box::new(|_cc| Box::new(SimulatorWindow {
            state,
            ..Default::default()
        })),
    );
}