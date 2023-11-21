use nalgebra::*;
// use octree::{point::Point3D, Octree}; // Stand-in for https://github.com/csiro-robotics/ohm
use bvh::{
    aabb::{AABB,Bounded},
    bounding_hierarchy::BHShape,
    bvh::BVH,
    ray::Ray,
    Vector3 as Vec3,
};

// No generics ATM (but add later)
// Consider different library without floating point points
fn main() {
    println!("Hello, world!");
}

trait Pose {
    fn xy(&self) -> Vector3<f64>;
    fn dir(&self) -> Quaternion<f64>;
}

type RobotPose = (Vector3<f64>, Quaternion<f64>);

impl Pose for RobotPose {
    fn xy(&self) -> Vector3<f64> {
        let p = self.0;
        Vector3::new(p.x as f64, p.y as f64, p.z as f64)
    }
    fn dir(&self) -> Quaternion<f64> { self.1 }
}

trait GridMap {
    fn ray_probe(&self, ray: (&Vector3<u32>, &Vector3<f32>)) -> f64;
    fn insert(&mut self, x: &Vector3<u32>);
}

struct OccupancyGrid {
    map: BVH,
    cells: Vec<GridCell>
}

impl OccupancyGrid {
    fn build(shapes: &mut [GridCell]) -> OccupancyGrid {
        OccupancyGrid {
            cells: shapes.to_vec(),
            map: BVH::build(shapes)
        }
    }

    fn rebuild(&mut self) {
        self.map = BVH::build(&mut self.cells[..])
    }
}

impl GridMap for OccupancyGrid {
    fn ray_probe(&self, ray: (&Vector3<u32>, &Vector3<u32>)) -> Option<f64> {
            let shapes = self.map.traverse_iterator(Ray::new(vector3tovec3(ray.0), vector3tovec3(ray.1)), &self.cells);
            shapes
                .map(|shape| distance(shape.position.into(), ray.0))
                .min()
    }
    // Inefficient in that every time the map is updated, the BVH is rebuilt. Consider Fast
    // insertion algorithms
    fn insert(&mut self, x: &Vector3<u32>) {
        self.cells.push(GridCell {
            position: *x,
            node_index: None
        })
    }
}

// Assume every grid cell is its bottom left coord
#[derive(Debug, Clone)]
struct GridCell {
    position: Vector3<u32>,
    node_index: Option<usize>,
}

impl Bounded for GridCell {
    fn aabb(&self) -> AABB {
        // Conversion into Vec3
        let m = vector3tovec3(self.position);
        AABB::with_bounds(m, m + Vec3::splat(1.))
    }
}

// Consider using macros
impl BHShape for GridCell {
    fn set_bh_node_index(&mut self, index: usize) {
        self.node_index = Some(index)
    }

    fn bh_node_index(&self) -> usize {
        self.node_index.unwrap_or_default()
    }
}

fn vector3tovec3(v: Vector3<u32>) -> Vec3 {
    let mut m = [0., 0., 0.];
    for i in 0..2 {
        m[i] = v[i] as f32
    }
    m.into()
}
