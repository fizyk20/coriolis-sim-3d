mod renderer;
mod simulation;

use egui::Vec2;
use glium::glutin;
use numeric_algs::integration::RK4Integrator;

use renderer::Renderer;

use crate::simulation::{explosion, Object, Position, Velocity, OMEGA};

pub struct State {
    pub t: f64,
    pub omega: f64,
    pub ang: f64,
    pub lat: f32,
    pub lon: f32,
    pub distance: f32,
    pub running: bool,
    pub time_step: f64,
    pub objects: Vec<Object>,
}

impl State {
    fn drag(&mut self, drag_delta: Vec2) {
        self.lat = (self.lat + drag_delta.y * 0.01).clamp(-1.57, 1.57);
        self.lon = (self.lon - drag_delta.x * 0.01) % 6.2831853;
    }

    fn scroll(&mut self, scroll: glutin::event::MouseScrollDelta) {
        use glutin::event::MouseScrollDelta::*;
        match scroll {
            LineDelta(_x, y) => {
                self.distance = (self.distance / 2.0_f32.powf(y as f32 * 0.2)).clamp(6378e3, 2e9);
            }
            PixelDelta(pos) => {
                println!("PixelDelta({:?})", pos);
            }
        }
    }
}

fn create_object(lat: f64, lon: f64, elev: f64, v_e: f64, v_n: f64, v_u: f64) -> Object {
    let pos = Position::from_lat_lon_elev(lat, lon, elev);
    let vel = Velocity::from_east_north_up(pos, v_e, v_n, v_u);
    Object::new(pos, vel)
}

fn main() {
    let event_loop = glutin::event_loop::EventLoop::with_user_event();
    let display = create_display(&event_loop);

    let mut egui_glium = egui_glium::EguiGlium::new(&display);

    let mut renderer = Renderer::new(&display);

    // A satellite
    let objects = vec![create_object(52.0, 0.0, 400e3, 7700.0, 0.0, 0.0)];

    // Anticyclones
    //let mut objects = explosion(45.0, 0.0, 10e3, 100.0, 10.0, 8, (0.7, 0.7, 0.0));
    //objects.extend(explosion(-45.0, 0.0, 10e3, 100.0, 10.0, 8, (0.0, 0.7, 0.7)));

    // Foucault pendulums
    /*let objects = vec![
        create_object(89.9, 0.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(75.0, -15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(60.0, 15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(45.0, 0.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(30.0, 15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(15.0, 0.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(0.0, 15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(-15.0, 0.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(-30.0, 15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(-45.0, 0.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(-60.0, 15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(-75.0, -15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
        create_object(-89.9, 15.0, 1e3, 0.0, 1000.0, 0.0).as_pendulum(2e-6),
    ];*/

    let mut state = State {
        t: 0.0,
        omega: 1.0,
        ang: 0.0,
        lat: 0.0,
        lon: 0.0,
        distance: 60e6,
        running: false,
        time_step: 10.0,
        objects,
    };

    let mut integrator = RK4Integrator::new(10.0);

    event_loop.run(move |event, _, control_flow| {
        let mut redraw = || {
            let mut quit = false;

            let needs_repaint = egui_glium.run(&display, |egui_ctx| {
                egui::CentralPanel::default()
                    .frame(egui::Frame::none())
                    .show(egui_ctx, |ui| {
                        let available_size = ui.available_size();
                        let (id, rect) = ui.allocate_space(available_size);
                        let response = ui.interact(rect, id, egui::Sense::drag());
                        state.drag(response.drag_delta());
                    });

                egui::Window::new("Simulation data").show(egui_ctx, |ui| {
                    ui.checkbox(&mut state.running, "Simulation running");
                    ui.label(format!("Current view lat: {:3.1}", state.lat.to_degrees()));
                    let mut lon = state.lon.to_degrees() % 360.0;
                    if lon > 180.0 {
                        lon -= 360.0;
                    }
                    ui.label(format!("Current view lon: {:4.1}", lon));
                    ui.label("Rotation of the reference frame:");
                    ui.add(egui::Slider::new(&mut state.omega, 0.0..=1.0));
                    ui.label("Time step:");
                    ui.add(egui::Slider::new(&mut state.time_step, 1.0..=1000.0).logarithmic(true));

                    ui.label("Objects");
                    ui.indent(0u64, |ui| {
                        for (i, obj) in state.objects.iter().enumerate() {
                            ui.collapsing(format!("Object {}", i), |ui| {
                                let vel = obj.vel.to_omega(obj.pos, state.omega * OMEGA);
                                ui.label(format!("Vel = {:5.1} m/s", vel.vel.norm()));
                            });
                        }
                    });

                    if ui.button("Quit").clicked() {
                        quit = true;
                    }
                });
            });

            let needs_repaint = needs_repaint || true;

            *control_flow = if quit {
                glutin::event_loop::ControlFlow::Exit
            } else if needs_repaint {
                display.gl_window().window().request_redraw();
                glutin::event_loop::ControlFlow::Poll
            } else {
                glutin::event_loop::ControlFlow::Poll
            };

            if state.running {
                for obj in &mut state.objects {
                    obj.step(&mut integrator, state.time_step);
                }
                state.t += state.time_step;
                state.ang += state.omega * OMEGA * state.time_step;
            }

            {
                use glium::Surface as _;
                let mut target = display.draw();

                let color = egui::Rgba::from_rgb(0.1, 0.3, 0.2);
                target.clear_color(color[0], color[1], color[2], color[3]);

                // draw here
                renderer.draw(&display, &mut target, &state);

                egui_glium.paint(&display, &mut target);

                target.finish().unwrap();
            }
        };

        match event {
            glutin::event::Event::RedrawEventsCleared if cfg!(windows) => redraw(),
            glutin::event::Event::RedrawRequested(_) if !cfg!(windows) => redraw(),

            glutin::event::Event::WindowEvent { event, .. } => {
                use glutin::event::WindowEvent;
                match event {
                    WindowEvent::CloseRequested | WindowEvent::Destroyed => {
                        *control_flow = glutin::event_loop::ControlFlow::Exit;
                    }
                    WindowEvent::MouseWheel { delta, .. } => {
                        state.scroll(delta);
                    }
                    _ => (),
                }

                egui_glium.on_event(&event);

                display.gl_window().window().request_redraw(); // TODO: ask egui if the events warrants a repaint instead
            }

            _ => (),
        }
    });
}

fn create_display(event_loop: &glutin::event_loop::EventLoop<()>) -> glium::Display {
    let window_builder = glutin::window::WindowBuilder::new()
        .with_resizable(true)
        .with_inner_size(glutin::dpi::LogicalSize {
            width: 800.0,
            height: 600.0,
        })
        .with_title("Coriolis Demo 3D");

    let context_builder = glutin::ContextBuilder::new()
        .with_depth_buffer(24)
        .with_srgb(true)
        .with_stencil_buffer(0)
        .with_vsync(true);

    glium::Display::new(window_builder, context_builder, event_loop).unwrap()
}
