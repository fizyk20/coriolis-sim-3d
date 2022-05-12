use std::io::Cursor;

use glium::{uniform, Display, DrawParameters, Frame, IndexBuffer, Program, Surface, VertexBuffer};
use nalgebra::Matrix4;

use super::Vertex;

pub struct Cubemap {
    program: Program,
    vertex_buffer: VertexBuffer<Vertex>,
    index_buffer: IndexBuffer<u16>,
    cubemap: glium::texture::Cubemap,
}

impl Cubemap {
    pub fn new(display: &Display) -> Self {
        let image = image::load(
            Cursor::new(&include_bytes!("media/px.png")[..]),
            image::ImageFormat::Png,
        )
        .unwrap()
        .to_rgba8();
        let image_dimensions = image.dimensions();
        let image =
            glium::texture::RawImage2d::from_raw_rgba_reversed(&image.into_raw(), image_dimensions);
        let tex_px = glium::Texture2d::new(display, image).unwrap();

        let image = image::load(
            Cursor::new(&include_bytes!("media/py.png")[..]),
            image::ImageFormat::Png,
        )
        .unwrap()
        .to_rgba8();
        let image_dimensions = image.dimensions();
        let image =
            glium::texture::RawImage2d::from_raw_rgba_reversed(&image.into_raw(), image_dimensions);
        let tex_py = glium::Texture2d::new(display, image).unwrap();

        let image = image::load(
            Cursor::new(&include_bytes!("media/pz.png")[..]),
            image::ImageFormat::Png,
        )
        .unwrap()
        .to_rgba8();
        let image_dimensions = image.dimensions();
        let image =
            glium::texture::RawImage2d::from_raw_rgba_reversed(&image.into_raw(), image_dimensions);
        let tex_pz = glium::Texture2d::new(display, image).unwrap();

        let image = image::load(
            Cursor::new(&include_bytes!("media/nx.png")[..]),
            image::ImageFormat::Png,
        )
        .unwrap()
        .to_rgba8();
        let image_dimensions = image.dimensions();
        let image =
            glium::texture::RawImage2d::from_raw_rgba_reversed(&image.into_raw(), image_dimensions);
        let tex_nx = glium::Texture2d::new(display, image).unwrap();

        let image = image::load(
            Cursor::new(&include_bytes!("media/ny.png")[..]),
            image::ImageFormat::Png,
        )
        .unwrap()
        .to_rgba8();
        let image_dimensions = image.dimensions();
        let image =
            glium::texture::RawImage2d::from_raw_rgba_reversed(&image.into_raw(), image_dimensions);
        let tex_ny = glium::Texture2d::new(display, image).unwrap();

        let image = image::load(
            Cursor::new(&include_bytes!("media/nz.png")[..]),
            image::ImageFormat::Png,
        )
        .unwrap()
        .to_rgba8();
        let image_dimensions = image.dimensions();
        let image =
            glium::texture::RawImage2d::from_raw_rgba_reversed(&image.into_raw(), image_dimensions);
        let tex_nz = glium::Texture2d::new(display, image).unwrap();

        let cubemap = glium::texture::Cubemap::empty(display, 1000).unwrap();

        let vertex_buffer = {
            let side2: f32 = 5000.0 / 2.0;

            VertexBuffer::new(
                display,
                &[
                    // Front
                    Vertex {
                        position: [-side2, -side2, side2],
                    },
                    Vertex {
                        position: [side2, -side2, side2],
                    },
                    Vertex {
                        position: [side2, side2, side2],
                    },
                    Vertex {
                        position: [-side2, side2, side2],
                    },
                    // Right
                    Vertex {
                        position: [side2, -side2, side2],
                    },
                    Vertex {
                        position: [side2, -side2, -side2],
                    },
                    Vertex {
                        position: [side2, side2, -side2],
                    },
                    Vertex {
                        position: [side2, side2, side2],
                    },
                    // Back
                    Vertex {
                        position: [-side2, -side2, -side2],
                    },
                    Vertex {
                        position: [-side2, side2, -side2],
                    },
                    Vertex {
                        position: [side2, side2, -side2],
                    },
                    Vertex {
                        position: [side2, -side2, -side2],
                    },
                    // Left
                    Vertex {
                        position: [-side2, -side2, side2],
                    },
                    Vertex {
                        position: [-side2, side2, side2],
                    },
                    Vertex {
                        position: [-side2, side2, -side2],
                    },
                    Vertex {
                        position: [-side2, -side2, -side2],
                    },
                    // Bottom
                    Vertex {
                        position: [-side2, -side2, side2],
                    },
                    Vertex {
                        position: [-side2, -side2, -side2],
                    },
                    Vertex {
                        position: [side2, -side2, -side2],
                    },
                    Vertex {
                        position: [side2, -side2, side2],
                    },
                    // Top
                    Vertex {
                        position: [-side2, side2, side2],
                    },
                    Vertex {
                        position: [side2, side2, side2],
                    },
                    Vertex {
                        position: [side2, side2, -side2],
                    },
                    Vertex {
                        position: [-side2, side2, -side2],
                    },
                ],
            )
            .unwrap()
        };

        let index_buffer = IndexBuffer::new(
            display,
            glium::index::PrimitiveType::TrianglesList,
            &[
                // Front
                0u16, 2, 1, 0, 3, 2, // Right
                4, 6, 5, 4, 7, 6, // Back
                8, 10, 9, 8, 11, 10, // Left
                12, 14, 13, 12, 15, 14, // Bottom
                16, 18, 17, 16, 19, 18, // Top
                20, 22, 21, 20, 23, 22,
            ],
        )
        .unwrap();

        let program = glium::Program::from_source(
            display,
            " #version 140

            in vec3 position;
            out vec3 ReflectDir;

            uniform mat4 matrix;

            void main() {
                ReflectDir = position;
                gl_Position = matrix * vec4(position, 1.0);
            }
            ",
            " #version 140
            in vec3 ReflectDir;
            out vec4 color;

            uniform samplerCube cubetex;

            void main() {
                color = texture(cubetex, ReflectDir) * texture(cubetex, ReflectDir);
            }
            ",
            None,
        )
        .unwrap();

        let framebuffer1 = glium::framebuffer::SimpleFrameBuffer::new(
            display,
            cubemap
                .main_level()
                .image(glium::texture::CubeLayer::PositiveX),
        )
        .unwrap();
        let framebuffer2 = glium::framebuffer::SimpleFrameBuffer::new(
            display,
            cubemap
                .main_level()
                .image(glium::texture::CubeLayer::NegativeX),
        )
        .unwrap();
        let framebuffer3 = glium::framebuffer::SimpleFrameBuffer::new(
            display,
            cubemap
                .main_level()
                .image(glium::texture::CubeLayer::PositiveY),
        )
        .unwrap();
        let framebuffer4 = glium::framebuffer::SimpleFrameBuffer::new(
            display,
            cubemap
                .main_level()
                .image(glium::texture::CubeLayer::NegativeY),
        )
        .unwrap();
        let framebuffer5 = glium::framebuffer::SimpleFrameBuffer::new(
            display,
            cubemap
                .main_level()
                .image(glium::texture::CubeLayer::PositiveZ),
        )
        .unwrap();
        let framebuffer6 = glium::framebuffer::SimpleFrameBuffer::new(
            display,
            cubemap
                .main_level()
                .image(glium::texture::CubeLayer::NegativeZ),
        )
        .unwrap();

        let dest_rect1 = glium::BlitTarget {
            left: 0,
            bottom: 0,
            width: 1000,
            height: 1000,
        };

        tex_px.as_surface().blit_whole_color_to(
            &framebuffer1,
            &dest_rect1,
            glium::uniforms::MagnifySamplerFilter::Linear,
        );
        tex_nx.as_surface().blit_whole_color_to(
            &framebuffer2,
            &dest_rect1,
            glium::uniforms::MagnifySamplerFilter::Linear,
        );
        tex_ny.as_surface().blit_whole_color_to(
            &framebuffer3,
            &dest_rect1,
            glium::uniforms::MagnifySamplerFilter::Linear,
        );
        tex_py.as_surface().blit_whole_color_to(
            &framebuffer4,
            &dest_rect1,
            glium::uniforms::MagnifySamplerFilter::Linear,
        );
        tex_pz.as_surface().blit_whole_color_to(
            &framebuffer5,
            &dest_rect1,
            glium::uniforms::MagnifySamplerFilter::Linear,
        );
        tex_nz.as_surface().blit_whole_color_to(
            &framebuffer6,
            &dest_rect1,
            glium::uniforms::MagnifySamplerFilter::Linear,
        );

        Self {
            program,
            vertex_buffer,
            index_buffer,
            cubemap,
        }
    }

    pub fn draw(
        &self,
        target: &mut Frame,
        matrix: &Matrix4<f32>,
        draw_parameters: &DrawParameters,
    ) {
        let skybox_uniforms = uniform! {
             matrix: *matrix.as_ref(),
             cubetex: self.cubemap.sampled()
                          .magnify_filter(glium::uniforms::MagnifySamplerFilter::Linear),
        };

        let params = DrawParameters {
            depth: glium::Depth {
                write: false,
                test: glium::DepthTest::Overwrite,
                ..Default::default()
            },
            ..draw_parameters.clone()
        };

        target
            .draw(
                &self.vertex_buffer,
                &self.index_buffer,
                &self.program,
                &skybox_uniforms,
                &params,
            )
            .unwrap();
    }
}
