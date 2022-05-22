use egui::Ui;

use crate::state::{ObjectDescription, ObjectKind};

pub fn display_object(obj: &mut ObjectDescription, ui: &mut Ui) -> bool {
    let mut remove = false;

    ui.horizontal(|ui| {
        ui.label(format!("{}", obj.kind.as_tag()));
        if ui.button("Remove").clicked() {
            remove = true;
        }
    });

    ui.horizontal(|ui| {
        ui.label("Latitude:");
        ui.text_edit_singleline(&mut obj.lat);
        ui.label("°");
    });
    ui.horizontal(|ui| {
        ui.label("Longitude:");
        ui.text_edit_singleline(&mut obj.lon);
        ui.label("°");
    });
    ui.horizontal(|ui| {
        ui.label("Elevation:");
        ui.text_edit_singleline(&mut obj.elev);
        ui.label("m");
    });

    match &mut obj.kind {
        ObjectKind::Free {
            vel_e,
            vel_n,
            vel_u,
            friction,
            gravity,
        } => {
            ui.horizontal(|ui| {
                ui.label("Velocity east:");
                ui.text_edit_singleline(vel_e);
                ui.label("m/s");
            });
            ui.horizontal(|ui| {
                ui.label("Velocity north:");
                ui.text_edit_singleline(vel_n);
                ui.label("m/s");
            });
            ui.horizontal(|ui| {
                ui.label("Velocity up:");
                ui.text_edit_singleline(vel_u);
                ui.label("m/s");
            });
            ui.horizontal(|ui| {
                ui.label("Strength of gravity:");
                ui.text_edit_singleline(gravity);
                ui.label("g");
            });
            ui.horizontal(|ui| {
                ui.label("Friction coefficient:");
                ui.text_edit_singleline(friction);
            });
        }
        ObjectKind::Cyclone {
            n_particles,
            radius,
            vel,
        } => {
            ui.horizontal(|ui| {
                ui.label("Number of particles:");
                ui.text_edit_singleline(n_particles);
            });
            ui.horizontal(|ui| {
                ui.label("Radius:");
                ui.text_edit_singleline(radius);
                ui.label("km");
            });
            ui.horizontal(|ui| {
                ui.label("Velocity:");
                ui.text_edit_singleline(vel);
                ui.label("m/s");
            });
        }
        ObjectKind::Anticyclone { n_particles, vel } => {
            ui.horizontal(|ui| {
                ui.label("Number of particles:");
                ui.text_edit_singleline(n_particles);
            });
            ui.horizontal(|ui| {
                ui.label("Velocity:");
                ui.text_edit_singleline(vel);
                ui.label("m/s");
            });
        }
        ObjectKind::Foucault { azim, vel } | ObjectKind::Plane { azim, vel } => {
            ui.horizontal(|ui| {
                ui.label("Starting azimuth:");
                ui.text_edit_singleline(azim);
                ui.label("°");
            });
            ui.horizontal(|ui| {
                ui.label("Starting velocity:");
                ui.text_edit_singleline(vel);
                ui.label("m/s");
            });
        }
    }

    ui.horizontal(|ui| {
        ui.label("Color:");
        ui.color_edit_button_rgb(&mut obj.color);
    });

    ui.separator();

    remove
}
