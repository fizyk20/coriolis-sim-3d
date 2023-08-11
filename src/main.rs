mod interface;
mod renderer;
mod simulation;
mod state;

use glium::glutin;
use numeric_algs::integration::RK4Integrator;

use renderer::Renderer;

use crate::{
    simulation::OMEGA,
    state::{CameraStateDef, ObjectDescription, ObjectKind, ObjectKindTag, State, StateTag},
};

use interface::display_object;

enum EditResult {
    None,
    Ok,
    Cancel,
}

fn main() {
    let event_loop = glutin::event_loop::EventLoop::with_user_event();
    let display = create_display(&event_loop);

    let mut egui_glium = egui_glium::EguiGlium::new(&display);

    let mut renderer = Renderer::new(&display);

    let mut state = State::default();

    let mut integrator = RK4Integrator::new(10.0);

    event_loop.run(move |event, _, control_flow| {
        let mut redraw = || {
            let mut quit = false;

            if state.running {
                for obj in &mut state.objects {
                    obj.step(&mut integrator, state.time_step);
                }
                state.t += state.time_step;
                state.ang += state.omega * OMEGA * state.time_step;
            }

            let needs_repaint = egui_glium.run(&display, |egui_ctx| {
                egui::CentralPanel::default()
                    .frame(egui::Frame::none())
                    .show(egui_ctx, |ui| {
                        let available_size = ui.available_size();
                        let (id, rect) = ui.allocate_space(available_size);
                        let response = ui.interact(rect, id, egui::Sense::drag());
                        if ui.input().modifiers.shift {
                            state.camera_state.shift_drag(response.drag_delta());
                        } else {
                            state.camera_state.drag(response.drag_delta());
                        }
                    });

                egui::Window::new("Simulation data").show(egui_ctx, |ui| {
                    ui.horizontal(|ui| {
                        if state.running {
                            if ui.button("Pause simulation").clicked() {
                                state.running = false;
                            }
                        } else {
                            if ui.button("Resume simulation").clicked() {
                                state.running = true;
                            }
                        }
                        if ui.button("Reset").clicked() {
                            state.reset_state();
                        }
                    });

                    if state.running {
                        state.render_settings.max_t = state.t;
                    }
                    ui.label("Time range to render:");
                    ui.add(egui::Slider::new(
                        &mut state.render_settings.max_t,
                        0.0..=state.t,
                    ));

                    ui.separator();

                    ui.checkbox(&mut state.render_settings.draw_grid, "Draw grid");
                    ui.checkbox(
                        &mut state.render_settings.draw_solid_surface,
                        "Draw solid surface",
                    );
                    ui.checkbox(&mut state.render_settings.use_texture, "Use the texture");
                    ui.checkbox(
                        &mut state.render_settings.draw_velocities,
                        "Draw velocities",
                    );
                    ui.label("Velocity scale:");
                    ui.add(
                        egui::Slider::new(&mut state.render_settings.vel_scale, 1e1..=1e8)
                            .logarithmic(true),
                    );
                    ui.checkbox(&mut state.render_settings.draw_forces, "Draw forces");
                    ui.label("Force scale:");
                    ui.add(
                        egui::Slider::new(&mut state.render_settings.force_scale, 1e2..=1e9)
                            .logarithmic(true),
                    );

                    ui.separator();

                    ui.label(format!(
                        "Current lat: {:3.1}",
                        state.camera_state.external.lat.to_degrees()
                    ));
                    let mut lon = (state.camera_state.external.lon as f64 + state.ang
                        - OMEGA * state.t)
                        .to_degrees()
                        % 360.0;
                    if lon > 180.0 {
                        lon -= 360.0;
                    }
                    if lon < -180.0 {
                        lon += 360.0;
                    }
                    ui.label(format!("Current lon: {:4.1}", lon));

                    ui.separator();

                    ui.label("Rotation of the reference frame:");
                    ui.add(egui::Slider::new(&mut state.omega, 0.0..=1.0));
                    ui.label("Time step:");
                    ui.add(egui::Slider::new(&mut state.time_step, 1.0..=1000.0).logarithmic(true));

                    ui.separator();

                    ui.horizontal(|ui| {
                        ui.label("Camera:");
                        let mut selected_camera = state.camera_state.as_def();
                        egui::ComboBox::from_label("")
                            .selected_text(format!("{}", selected_camera))
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut selected_camera,
                                    CameraStateDef::External,
                                    format!("{}", CameraStateDef::External),
                                );
                                for i in 0..state.objects.len() {
                                    ui.selectable_value(
                                        &mut selected_camera,
                                        CameraStateDef::Following(i),
                                        format!("{}", CameraStateDef::Following(i)),
                                    );
                                }
                            });
                        state.camera_state.set_from_def(selected_camera);
                    });

                    ui.separator();

                    if ui.button("Edit state").clicked() {
                        state.new_state_def = Some(state.current_state_def.clone());
                    }

                    ui.label("Objects");
                    ui.indent(0u64, |ui| {
                        for (i, obj) in state.objects.iter().enumerate() {
                            ui.collapsing(format!("Object {}", i), |ui| {
                                let status =
                                    obj.status(state.omega * OMEGA, &state.render_settings);
                                for text in status {
                                    ui.label(text);
                                }
                            });
                        }
                    });

                    ui.separator();

                    if ui.button("Quit").clicked() {
                        quit = true;
                    }
                });

                let mut edit_result = EditResult::None;
                if let Some(ref mut new_state_def) = state.new_state_def {
                    egui::Window::new("Editing state").show(egui_ctx, |ui| {
                        ui.horizontal(|ui| {
                            ui.label("Object to add:");
                            egui::ComboBox::from_label("")
                                .selected_text(format!("{}", new_state_def.selected_kind))
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(
                                        &mut new_state_def.selected_kind,
                                        ObjectKindTag::Free,
                                        format!("{}", ObjectKindTag::Free),
                                    );
                                    ui.selectable_value(
                                        &mut new_state_def.selected_kind,
                                        ObjectKindTag::Cyclone,
                                        format!("{}", ObjectKindTag::Cyclone),
                                    );
                                    ui.selectable_value(
                                        &mut new_state_def.selected_kind,
                                        ObjectKindTag::Anticyclone,
                                        format!("{}", ObjectKindTag::Anticyclone),
                                    );
                                    ui.selectable_value(
                                        &mut new_state_def.selected_kind,
                                        ObjectKindTag::Foucault,
                                        format!("{}", ObjectKindTag::Foucault),
                                    );
                                    ui.selectable_value(
                                        &mut new_state_def.selected_kind,
                                        ObjectKindTag::Plane,
                                        format!("{}", ObjectKindTag::Plane),
                                    );
                                });
                            if ui.button("Add").clicked() {
                                let new_object_kind = match new_state_def.selected_kind {
                                    ObjectKindTag::Free => ObjectKind::default_free(),
                                    ObjectKindTag::Cyclone => ObjectKind::default_cyclone(),
                                    ObjectKindTag::Anticyclone => ObjectKind::default_anticyclone(),
                                    ObjectKindTag::Foucault => ObjectKind::default_foucault(),
                                    ObjectKindTag::Plane => ObjectKind::default_plane(),
                                };
                                let new_object = ObjectDescription {
                                    kind: new_object_kind,
                                    ..Default::default()
                                };
                                new_state_def.objects.push(new_object);
                            }
                        });
                        ui.separator();
                        let mut to_remove: Option<usize> = None;
                        egui::ScrollArea::vertical()
                            .max_height(300.0)
                            .show(ui, |ui| {
                                for (index, obj) in new_state_def.objects.iter_mut().enumerate() {
                                    if display_object(obj, ui) {
                                        to_remove = Some(index);
                                    }
                                }
                            });
                        if let Some(index) = to_remove {
                            new_state_def.objects.remove(index);
                        }
                        ui.separator();
                        ui.horizontal(|ui| {
                            if ui.button("OK").clicked() {
                                edit_result = EditResult::Ok;
                            }
                            if ui.button("Cancel").clicked() {
                                edit_result = EditResult::Cancel;
                            }
                        });
                    });
                }
                match edit_result {
                    EditResult::None => (),
                    EditResult::Cancel => {
                        state.new_state_def = None;
                    }
                    EditResult::Ok => {
                        if let Some(new_state) = state.new_state_def.take() {
                            state.current_state_def = new_state;
                            state.reset_state();
                        }
                    }
                }
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
                        state.camera_state.scroll(delta);
                    }
                    _ => (),
                }

                egui_glium.on_event(&event);

                display.gl_window().window().request_redraw();
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
