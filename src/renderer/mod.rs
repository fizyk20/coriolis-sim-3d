mod mesh;

use glium::{implement_vertex, uniform, Display, Frame, Program, Surface};
use nalgebra::{Matrix4, Point3, Vector3};

use crate::{
    simulation::{Object, OMEGA},
    State,
};
pub use mesh::Mesh;

const VERTEX_SHADER_SRC: &'static str = r#"
    #version 140

    in vec3 position;

    uniform mat4 matrix;
    uniform vec3 color;
    out vec3 in_color;

    void main() {
        gl_Position = matrix * vec4(position, 1.0);
        in_color = color;
    }
"#;

const FRAGMENT_SHADER_SRC: &'static str = r#"
    #version 140

    in vec3 in_color;
    out vec4 color;

    void main() {
        color = vec4(in_color, 1.0);
    }
"#;

#[derive(Debug, Clone, Copy)]
pub struct Vertex {
    pub position: [f32; 3],
}

implement_vertex!(Vertex, position);

pub struct Renderer {
    program: Program,
    sphere: Mesh,
}

impl Renderer {
    pub fn new(display: &Display) -> Self {
        Renderer {
            program: Program::from_source(display, VERTEX_SHADER_SRC, FRAGMENT_SHADER_SRC, None)
                .unwrap(),
            sphere: Mesh::ellipsoid(display),
        }
    }

    pub fn draw(
        &mut self,
        display: &Display,
        target: &mut Frame,
        state: &State,
        objects: &[Object],
    ) {
        target.clear_color(0.0, 0.0, 0.02, 1.0);
        target.clear_depth(1.0);

        let (width, height) = target.get_dimensions();
        let aspect = width as f32 / height as f32;
        let dist = state.distance;
        let lat = state.lat;
        let lon = state.lon;
        let matrix = Matrix4::new_perspective(aspect, 45.0_f32.to_radians(), 1000.0, 1e9)
            * Matrix4::look_at_rh(
                &Point3::new(
                    dist * lon.sin() * lat.cos(),
                    dist * lat.sin(),
                    dist * lon.cos() * lat.cos(),
                ),
                &Point3::new(0.0, 0.0, 0.0),
                &Vector3::new(0.0, 1.0, 0.0),
            );

        let omega = OMEGA * state.omega;
        let ang = (OMEGA - omega) * objects[0].time();

        let rotation = Matrix4::new_rotation(Vector3::new(0.0, ang as f32, 0.0));

        let uniforms = uniform! {
            matrix: *(matrix * rotation).as_ref(),
            color: [0.4_f32, 1.0, 0.4],
        };

        self.sphere.draw(
            target,
            &self.program,
            &uniforms,
            &glium::DrawParameters {
                depth: glium::draw_parameters::Depth {
                    test: glium::draw_parameters::DepthTest::IfLess,
                    write: true,
                    ..Default::default()
                },
                line_width: Some(4.0),
                ..Default::default()
            },
        );

        for obj in objects {
            obj.draw(
                omega,
                display,
                target,
                &self.program,
                &matrix,
                &glium::DrawParameters {
                    depth: glium::draw_parameters::Depth {
                        test: glium::draw_parameters::DepthTest::IfLess,
                        write: true,
                        ..Default::default()
                    },
                    line_width: Some(6.0),
                    ..Default::default()
                },
            );
        }
    }
}
