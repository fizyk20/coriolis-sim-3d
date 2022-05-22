use std::collections::HashMap;

use glium::{
    index, uniforms::Uniforms, Display, DrawParameters, Frame, IndexBuffer, Program, Surface,
    VertexBuffer,
};

use super::{TexturedVertex, Vertex};
use crate::simulation::lat_lon_elev_to_vec3;

pub trait VertexLike: glium::Vertex {
    fn from_position(x: f32, y: f32, z: f32) -> Self;
    fn from_position_and_tex(x: f32, y: f32, z: f32, u: f32, v: f32) -> Self;
}

impl VertexLike for Vertex {
    fn from_position(x: f32, y: f32, z: f32) -> Self {
        Vertex {
            position: [x, y, z],
        }
    }

    fn from_position_and_tex(x: f32, y: f32, z: f32, _u: f32, _v: f32) -> Self {
        Vertex {
            position: [x, y, z],
        }
    }
}

impl VertexLike for TexturedVertex {
    fn from_position(x: f32, y: f32, z: f32) -> Self {
        TexturedVertex {
            position: [x, y, z],
            tex_coords: [0.0, 0.0],
        }
    }

    fn from_position_and_tex(x: f32, y: f32, z: f32, u: f32, v: f32) -> Self {
        TexturedVertex {
            position: [x, y, z],
            tex_coords: [u, v],
        }
    }
}

pub struct Mesh<T: VertexLike> {
    vertices: VertexBuffer<T>,
    indices: Vec<IndexBuffer<u32>>,
}

impl<T: VertexLike> Mesh<T> {
    pub fn solid_sphere(display: &Display, n_parallels: u32, n_meridians: u32) -> Mesh<T> {
        let mut vertices = vec![];
        let mut indices = vec![];

        // A vertex deduplication map. This ensures that all vertices are stored exactly once.
        // Points on the sphere are defined as (lat_index, lon_index) for the purpose of
        // non-duplication. lat_index is `0..=n_parallels * n_subdivisions`, lon_index is
        // `0..n_meridians * n_subdivisions`. The map maps point coordinates to vertex index.
        let mut result: HashMap<(u32, u32), u32> = HashMap::new();

        for lat_index in 0..n_parallels + 1 {
            for lon_index in 0..n_meridians + 1 {
                let _ = result.entry((lat_index, lon_index)).or_insert_with(|| {
                    let lat = (90.0 - 180.0 / (n_parallels as f64) * lat_index as f64).to_radians();
                    let lon =
                        (360.0 / (n_meridians as f64) * lon_index as f64 - 180.0).to_radians();
                    let x = lat.cos() * lon.cos();
                    let y = lat.cos() * lon.sin();
                    let z = lat.sin();

                    let u = lon_index as f32 / n_meridians as f32;
                    let v = (n_parallels - lat_index) as f32 / n_parallels as f32;

                    vertices.push(T::from_position_and_tex(y as f32, z as f32, x as f32, u, v));
                    vertices.len() as u32 - 1
                });
                // for poles, only insert lon_index = 0
                if lat_index == 0 || lat_index == n_parallels {
                    break;
                }
            }
        }

        // construct the fans around the north and south pole
        let mut fan_index_n = vec![result[&(0, 0)]];
        let mut fan_index_s = vec![result[&(n_parallels, 0)]];
        for lon_index in 0..n_meridians + 1 {
            fan_index_n.push(result[&(1, lon_index)]);
            fan_index_s.push(result[&(n_parallels - 1, lon_index)]);
        }
        indices.push(
            IndexBuffer::new(display, index::PrimitiveType::TriangleFan, &fan_index_n).unwrap(),
        );
        indices.push(
            IndexBuffer::new(display, index::PrimitiveType::TriangleFan, &fan_index_s).unwrap(),
        );

        // parallel strips
        for lat_index in 1..n_parallels - 1 {
            let mut parallel_index = vec![];
            for lon_index in 0..n_meridians + 1 {
                parallel_index.push(result[&(lat_index, lon_index)]);
                parallel_index.push(result[&(lat_index + 1, lon_index)]);
            }
            indices.push(
                IndexBuffer::new(
                    display,
                    index::PrimitiveType::TriangleStrip,
                    &parallel_index,
                )
                .unwrap(),
            );
        }

        let vertices = VertexBuffer::new(display, &vertices).unwrap();

        Mesh { vertices, indices }
    }

    pub fn ellipsoid(display: &Display) -> Mesh<T> {
        let n_meridians = 24;
        let n_parallels = 12;
        let n_subdivisions = 10;

        let mut vertices = vec![];
        let mut indices = vec![];

        // A vertex deduplication map. This ensures that all vertices are stored exactly once.
        // Points on the sphere are defined as (lat_index, lon_index) for the purpose of
        // non-duplication. lat_index is `0..=n_parallels * n_subdivisions`, lon_index is
        // `0..n_meridians * n_subdivisions`. The map maps point coordinates to vertex index.
        let mut result: HashMap<(u32, u32), u32> = HashMap::new();

        // generate parallels
        for parallel_index in 0..n_parallels + 1 {
            let mut parallel_indices = vec![];
            // inclusive range to append longitude 0 once more at the end of the index buffer
            for lon_index in 0..=n_meridians * n_subdivisions {
                let lat_index = parallel_index * n_subdivisions;
                // if we're at the end of the range, this will map lon_index to 0 again
                let lon_index = lon_index % (n_meridians * n_subdivisions);
                let key = (lat_index, lon_index);
                let entry = result.entry(key).or_insert_with(|| {
                    let lat =
                        90.0 - 180.0 / ((n_parallels * n_subdivisions) as f64) * lat_index as f64;
                    let lon = 360.0 / ((n_meridians * n_subdivisions) as f64) * lon_index as f64;
                    let pos = lat_lon_elev_to_vec3(lat, lon, 0.0);
                    vertices.push(T::from_position(pos.x as f32, pos.y as f32, pos.z as f32));
                    vertices.len() as u32 - 1
                });
                // for poles, only insert lon_index = 0
                if lat_index == 0 || lat_index == n_parallels * n_subdivisions {
                    break;
                }
                parallel_indices.push(*entry);
            }
            if parallel_index == 0 || parallel_index == n_parallels {
                continue;
            }
            indices.push(
                IndexBuffer::new(display, index::PrimitiveType::LineStrip, &parallel_indices)
                    .unwrap(),
            );
        }

        for meridian_index in 0..n_meridians {
            let mut meridian_indices = vec![];
            for lat_index in 0..=n_parallels * n_subdivisions {
                let lon_index = if lat_index == 0 || lat_index == n_parallels * n_subdivisions {
                    0 // poles are always (lat, 0)
                } else {
                    meridian_index * n_subdivisions
                };
                let key = (lat_index, lon_index);
                let entry = result.entry(key).or_insert_with(|| {
                    let lat =
                        90.0 - 180.0 / ((n_parallels * n_subdivisions) as f64) * lat_index as f64;
                    let lon = 360.0 / ((n_meridians * n_subdivisions) as f64) * lon_index as f64;
                    let pos = lat_lon_elev_to_vec3(lat, lon, 0.0);
                    vertices.push(T::from_position(pos.x as f32, pos.y as f32, pos.z as f32));
                    vertices.len() as u32 - 1
                });
                meridian_indices.push(*entry);
            }
            indices.push(
                IndexBuffer::new(display, index::PrimitiveType::LineStrip, &meridian_indices)
                    .unwrap(),
            );
        }

        let vertices = VertexBuffer::new(display, &vertices).unwrap();

        Mesh { vertices, indices }
    }

    pub fn arrow(display: &Display) -> Mesh<T> {
        let n_divisions: u32 = 24;

        let head_len = 0.25f32;
        let radius = head_len / 6.0;

        let mut vertices = vec![
            T::from_position(0.0, 0.0, 1.0),            // tip
            T::from_position(0.0, 0.0, 1.0 - head_len), // middle of the base of the cone
        ];

        // vertices for the head
        for i in 0..n_divisions {
            let ang = (i as f32 * 360.0 / n_divisions as f32).to_radians();
            vertices.push(T::from_position(
                3.0 * radius * ang.cos(),
                3.0 * radius * ang.sin(),
                1.0 - head_len,
            ));
        }

        // vertices for the shaft
        for i in 0..n_divisions {
            let ang = (i as f32 * 360.0 / n_divisions as f32).to_radians();
            vertices.push(T::from_position(
                radius * ang.cos(),
                radius * ang.sin(),
                1.0 - head_len,
            ));
            vertices.push(T::from_position(
                radius * ang.cos(),
                radius * ang.sin(),
                0.0,
            ));
        }

        // middle of the end of the shaft
        vertices.push(T::from_position(0.0, 0.0, 0.0));

        let vertices = VertexBuffer::new(display, &vertices).unwrap();

        let mut indices = vec![];

        let mut head_cone = vec![0u32];
        for i in 0..n_divisions {
            head_cone.push(i + 2);
        }
        head_cone.push(2);
        indices.push(
            IndexBuffer::new(display, index::PrimitiveType::TriangleFan, &head_cone).unwrap(),
        );

        let mut head_base = vec![1];
        for i in 0..n_divisions {
            head_base.push(i + 2);
        }
        head_base.push(2);
        indices.push(
            IndexBuffer::new(display, index::PrimitiveType::TriangleFan, &head_base).unwrap(),
        );

        let mut shaft_base = vec![2 + n_divisions * 3];
        for i in 0..n_divisions {
            shaft_base.push(2 + n_divisions + i * 2 + 1);
        }
        shaft_base.push(2 + n_divisions + 1);
        indices.push(
            IndexBuffer::new(display, index::PrimitiveType::TriangleFan, &shaft_base).unwrap(),
        );

        let mut shaft = vec![];
        for i in 0..n_divisions * 2 {
            shaft.push(2 + n_divisions + i);
        }
        shaft.push(2 + n_divisions);
        shaft.push(2 + n_divisions + 1);
        indices
            .push(IndexBuffer::new(display, index::PrimitiveType::TriangleStrip, &shaft).unwrap());

        Mesh { vertices, indices }
    }

    pub fn draw<U: Uniforms>(
        &self,
        target: &mut Frame,
        program: &Program,
        uniforms: &U,
        draw_parameters: &DrawParameters,
    ) {
        for index_buffer in &self.indices {
            target
                .draw(
                    &self.vertices,
                    index_buffer,
                    program,
                    uniforms,
                    draw_parameters,
                )
                .unwrap();
        }
    }
}
