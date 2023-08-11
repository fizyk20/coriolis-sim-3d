mod description;
mod utils;

use egui::Vec2;
use glium::glutin;

use crate::simulation::Object;

pub use description::{InitialStateDefinition, ObjectDescription, ObjectKind, ObjectKindTag};

pub struct RenderSettings {
    pub draw_grid: bool,
    pub draw_solid_surface: bool,
    pub use_texture: bool,
    pub draw_velocities: bool,
    pub draw_forces: bool,
    pub vel_scale: f64,
    pub force_scale: f64,
    pub max_t: f64,
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            draw_grid: true,
            draw_solid_surface: true,
            use_texture: true,
            draw_velocities: false,
            draw_forces: false,
            vel_scale: 1e4,
            force_scale: 1e4,
            max_t: 0.0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct ExternalState {
    pub lat: f32,
    pub lon: f32,
    pub tilt: f32,
    pub turn: f32,
    pub distance: f32,
}

#[derive(Debug, Clone, Copy)]
pub struct FollowingState {
    pub obj: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateTag {
    External,
    Following,
}

#[derive(Debug, Clone, Copy)]
pub struct CameraState {
    pub tag: StateTag,
    pub external: ExternalState,
    pub following: FollowingState,
}

impl CameraState {
    pub fn drag(&mut self, drag_delta: Vec2) {
        if self.tag == StateTag::External {
            self.external.lat = (self.external.lat + drag_delta.y * 0.01).clamp(-1.57, 1.57);
            self.external.lon = (self.external.lon - drag_delta.x * 0.01) % 6.2831853;
        }
    }

    pub fn shift_drag(&mut self, drag_delta: Vec2) {
        if self.tag == StateTag::External {
            self.external.tilt = (self.external.tilt + drag_delta.y * 0.01).clamp(-1.57, 1.57);
            self.external.turn = (self.external.turn + drag_delta.x * 0.01).clamp(-3.14, 3.14);
        }
    }

    pub fn scroll(&mut self, scroll: glutin::event::MouseScrollDelta) {
        use glutin::event::MouseScrollDelta::*;
        match scroll {
            LineDelta(_x, y) => {
                if self.tag == StateTag::External {
                    self.external.distance =
                        (self.external.distance / 2.0_f32.powf(y as f32 * 0.2)).clamp(6378e3, 2e9);
                }
            }
            PixelDelta(pos) => {
                println!("PixelDelta({:?})", pos);
            }
        }
    }
}

pub struct State {
    pub t: f64,
    pub omega: f64,
    pub ang: f64,
    pub camera_state: CameraState,
    pub running: bool,
    pub time_step: f64,
    pub objects: Vec<Object>,
    pub current_state_def: InitialStateDefinition,
    pub new_state_def: Option<InitialStateDefinition>,
    pub render_settings: RenderSettings,
}

impl Default for State {
    fn default() -> Self {
        Self {
            t: 0.0,
            omega: 1.0,
            ang: 0.0,
            camera_state: CameraState {
                tag: StateTag::External,
                external: ExternalState {
                    lat: 0.0,
                    lon: 0.0,
                    tilt: 0.0,
                    turn: 0.0,
                    distance: 60e6,
                },
                following: FollowingState { obj: 0 },
            },
            running: false,
            time_step: 10.0,
            objects: vec![],
            current_state_def: Default::default(),
            new_state_def: None,
            render_settings: Default::default(),
        }
    }
}

impl State {
    pub fn reset_state(&mut self) {
        self.t = 0.0;
        self.ang = 0.0;
        self.omega = 1.0;

        self.objects = vec![];
        for object_def in self.current_state_def.objects.iter() {
            let objects = object_def.into_objects();
            self.objects.extend(objects);
        }
    }
}
