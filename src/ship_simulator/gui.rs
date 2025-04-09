// === GUI Libraries ===
use eframe::egui;
use egui::{Color32, Pos2, Shape, Stroke, Ui, Vec2};
use std::sync::{Arc, RwLock};

/// === SharedState ===
/// Holds all shared simulation state across threads and GUI
/// - ship_pos: Position of the ship in world coordinates
/// - ship_angle: Rotation angle of the ship in radians
/// - keyboard_state: Boolean state of W, A, S, D keys
#[derive(Clone, Default)]
pub struct SharedState {
    pub ship_pos: Arc<RwLock<[f32; 2]>>, // Ship position [x, y]
    pub ship_angle: Arc<RwLock<f32>>, // Ship orientation in radians
    pub keyboard_state: Arc<RwLock<[bool; 4]>>, // WASD state: [W, A, S, D]
    pub frame_interval_ms: Arc<RwLock<u64>>, // fps in ms
}

/// === draw_scene ===
/// Draws the simulation scene including:
/// - background grid
/// - ship shape
/// - mouse crosshair and coordinate labels
pub fn draw_scene(ui: &mut Ui, pos: [f32; 2], angle: f32, cam_offset: [f32; 2], zoom: f32) {
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
            to_screen([pos[1] + y, pos[0] + x])
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
            let mut keyboard_state = self.state.keyboard_state.write().unwrap();
            keyboard_state[0] = i.key_down(egui::Key::W);
            keyboard_state[1] = i.key_down(egui::Key::A);
            keyboard_state[2] = i.key_down(egui::Key::S);
            keyboard_state[3] = i.key_down(egui::Key::D);
        });

        // === GUI canvas with drag + zoom ===
        egui::CentralPanel::default().show(ctx, |ui| {
            // Allow camera panning by dragging
            let response = ui.interact(ui.max_rect(), ui.id().with("canvas"), egui::Sense::drag());
            if response.dragged() {
                let delta = response.drag_delta();
                self.camera_offset[0] -= delta.x / self.zoom;
                self.camera_offset[1] -= delta.y / self.zoom;
            }

            // Zoom using scroll wheel
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

            // Show debug info in panel
            ui.horizontal(|ui| {
                ui.label(format!("Ship X: {:.1}", pos[1]));
                ui.label(format!("Ship Y: {:.1}", pos[0]));
                ui.label(format!("Ship θ: {:.1}°", angle.to_degrees()));
                ui.label(format!("Zoom: {:.2}x", self.zoom));
                ui.label(format!("Pan X: {:.1}", self.camera_offset[0]));
                ui.label(format!("Pan Y: {:.1}", self.camera_offset[1]));
            });

            ui.separator(); // visual divider
            
            draw_scene(ui, pos, angle, self.camera_offset, self.zoom); // render everything
            
            let interval = *self.state.frame_interval_ms.read().unwrap();
            ctx.request_repaint_after(std::time::Duration::from_millis(interval)); // Wait a bit until next render
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