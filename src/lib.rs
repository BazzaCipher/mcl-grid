use std::f32::consts::PI;
use std::time::Duration;
use std::ops::{Index, IndexMut};

use nalgebra::*;
use bvh::{
    aabb::{AABB,Bounded},
    bounding_hierarchy::BHShape,
    bvh::BVH,
    ray::Ray,
    Vector3 as Vec3,
};
use bno055::Error as BnoError;
use ordered_float::NotNan;

// UOM here please
const RAY_MAXDIST: f32 = 5.0;
const DEFAULT_HEIGHT: f32 = 10.0;
const HANG: f32 = 35.0;
const VANG: f32 = 20.0;

// TODO: Generic & Borrowed
trait Pose {
    fn pos(&self) -> Point3<f32>;
    fn dir(&self) -> Rotation3<f32>;
}

// Vector3 for how we do it rn, allow expansion into 3D via trait
/// Standard form: [x, y, theta]T. In metres and radians
type RobotPose = Point3<f32>;

impl Pose for RobotPose {
    fn pos(&self) -> Point3<f32> {
        Point3::new(self.x, self.y, DEFAULT_HEIGHT)
    }
    fn dir(&self) -> Rotation3<f32> { 
        Rotation3::from_euler_angles(0., 0., self.z) 
    }
}

/// Abstracts the grid-map. Is a 3D representation with the floor being the default height
/// Allows updating for integration of data.
// TODO: Generic over cell type; cell trait?
trait GridMap {
    fn ray_probe(&self, ray: (&Point3<f32>, &Vector3<f32>)) -> f32;
    fn insert(&mut self, x: &Point3<i32>);
    fn update(&mut self, x: &impl Pose, u: &impl SampleMeasurement);
}

struct OccupancyGrid {
    bvh: BVH,

    cells: Vec<GridCell>
}

impl OccupancyGrid {
    fn build(shapes: &mut [GridCell]) -> OccupancyGrid {
        OccupancyGrid {
            cells: shapes.to_vec(),
            bvh: BVH::build(shapes)
        }
    }

    /// Must rebuild the bvh in order to use ray_probe. Not my job
    fn rebuild(&mut self) {
        self.bvh = BVH::build(&mut self.cells[..]);
    }
}

// COnsider having both
impl<'point> Index<&'point (f32, f32, f32)> for OccupancyGrid {
    type Output = &'point GridCell;

    fn index(&self, index: &(f32, f32, f32)) -> Self::Output {
        // Fucking shoot me; suggest octree but that'll take time
        self.cells
            .iter()
            .find(|x| index == x.position)
            .unwrap_or_default()
    }
}

impl<'point> IndexMut<&'point (f32, f32, f32)> for OccupancyGrid {
    fn index_mut(&mut self, index: &(f32, f32, f32)) -> &mut Self::Output {
        self.cells
            .iter()
            .find(|x| index == x.position.into())
            .unwrap_or_default()
    }
}

// Do not assume 1:1 scale
impl GridMap for OccupancyGrid {
    fn ray_probe(&self, ray: (&Point3<f32>, &Vector3<f32>)) -> f32 {
     // Should model behaviour of sensor (max dist etc)
         self.bvh
             .traverse_iterator(Ray::new(vector3tovec3(ray.0), vector3tovec3(ray.1)), &self.cells)
             .map(|shape| distance(shape.position.into(), ray.0) as i16) // TODO: Compare f64 dists instead
             .min()
             .map(f32::from)
             .unwrap_or(RAY_MAXDIST)
    }
    // Inefficient in that every time the map is updated, the BVH is rebuilt. Consider Fast
    // insertion algorithms
    // TODO: Generic squares
    fn insert(&mut self, x: &Point3<i32>) {
     // TODO: Handle none 1:1 case
         self.cells
             .push(GridCell {
                 position: *x,
                 intensity: NotNan::new(0.3), // Reasonable initial probability
                 node_index: None,
             })
    }

    // Recall assumption of known position for particle filters
    fn update<const VFOV: usize, const HFOV: usize>(&mut self, x: &RobotPose, u: &LaserImage<VFOV, HFOV>) {
     // TODO: Handle none 1:1 case
     u.integrate_into_map(x, u);
    }
}

// Assume every grid cell is its bottom left coord
#[derive(Default, Debug, Clone, Eq, PartialEq, Hash)]
struct GridCell {
    position: Point3<i32>,
    intensity: NotNan<f32>, // Log-odds representation
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

/// Contains the multimodal estimates, x, of the current pose/robot position
/// as well as best estimate
trait BeliefState {
    /// From the theory, we have the prediction based on all previous controls
    /// and the current state and map (ex ante)
    fn predict(&mut self, u: Vec<&impl SampleControl>);
    /// Integration of actual measurements
    fn integrate(&mut self, am: &impl SampleMeasurement);
    fn best_estimate(&self) -> Box<dyn Pose>; // TODO: Make Belief generic over Pose
}

struct Particles<const P: usize> {
    // Only keep two sets wherein the particles can be an ex ante prediction or
    // ex post (after evidence)
    last_estimate: Vec<RobotPose>,
    particles: [(RobotPose, OccupancyGrid); P],
    is_pred: bool,
}

impl<const P: usize> BeliefState for Particles<P> {
    fn predict(&mut self, mut u: Vec<&Imu>) { 
        // We don't use any other than the latest due to integration into the map already
        // Might make it non-Vec because we could just save it into the map
        let u = u.pop();

        self.particles
            .iter_mut()
            .map(|(pose, m)| u
                 .expect("Empty robot control 'u' vector")
                 .motion_simulation(pose, m)
             );
    }
    fn integrate<const VFOV: usize, const HFOV: usize>(&mut self, am: &LaserImage<VFOV, HFOV>) { 
        self.particles
            .iter_mut()
            .map(|(pose, m)| am.integrate_into_map(pose, m));
    }
    fn best_estimate(&self) -> RobotPose { todo!() }
}

/// Encapsulates the simulation of controls; the kinematic model
/// Allows for prediction of Pr(x_t|x_t-1,u,m)
trait SampleControl {
    fn motion_simulation(&self, x: &impl Pose, m: &impl GridMap) -> &dyn Pose;
}

// We assume it starts from 0,0 on a flat world
struct Imu {
    vel: Vector3<f32>,
    pos: Vector3<f32>,
    dt: f32, // In seconds
}

// TODO: Eventually implement trait directly on Bno055??
impl Imu {
    fn update(&mut self, t: f32) -> Result<(), BnoError<u8>> {
        // This model suffers from drift heavily; either KF or 120+Hz updates
        // TODO: Implement Kalman for integration of accelerations (VERY IMPORTANT)
        let ud = t * self.lin_accel()?;
        let wd = t * self.ang_accel()?.z;
        self.vel += Vector3::new(
            ud.x,
            ud.y,
            wd,
        );

        self.dt = t;

        Ok(())
    }
    // Initial pose offset adjusted calculations
    fn ang_accel(&mut self) -> Result<Vector3<f32>, BnoError<u8>> { todo!() }
    fn lin_accel(&mut self) -> Result<Vector3<f32>, BnoError<u8>> { todo!() }
}

impl SampleControl for Imu {
    fn motion_simulation(&self, x: &RobotPose, m: &OccupancyGrid) -> RobotPose {
        // Consider splitting into Jacobians Fp and Frl
        self.dt * self.vel + *x
        // TODO: Add noise for sampling...
    }
}

/// Encapsulates the simulation of what the robot should see; the measurement
/// model. The robot should be able to measure some fixed area.
/// Simulates Pr(z,m|x,u)
trait SampleMeasurement {
    fn measurement_simulation(&self, x: &impl Pose, u: &impl SampleControl, m: &impl GridMap) -> Self;
    // Choose to have the Measurement integrate itself into the map due to
    // how much harder it'll be to reimplement an occupancy map due to changing
    // measurement specifications. Therefore it'll be the job the measurement to integrate
    // itself into the map
    fn integrate_into_map(&self, x: &impl Pose, m: &impl GridMap);
}

// TODO: I need a way to design this so that the trait is satisfied, there is 
// no overhead on each image, and we can always access the metadata. Likely
// generics later.
struct LaserImage<const VFOV: usize, const HFOV: usize> {
    inner: SMatrix<f32, VFOV, HFOV>
}

impl<const VFOV: usize, const HFOV: usize> LaserImage<VFOV, HFOV> {
    fn new(input: SMatrix<f32, VFOV, HFOV>) -> LaserImage<VFOV, HFOV> {
        LaserImage { inner: input }
    }
}

/// Generic over the implementation type of the measurement for future changes
/// to matrix or otherwise
impl<const VFOV: usize, const HFOV: usize> SampleMeasurement for LaserImage<VFOV, HFOV> {
    fn measurement_simulation(&self, x: &RobotPose, _: &Imu, m: &OccupancyGrid) -> LaserImage<VFOV, HFOV> {
        // By assumption, M is in map units, mu
        let up: Vector3<f32>     = Vector3::z_axis().into_inner(); // By assumption of RobotPose
        let lookat: Vector3<f32> = x.dir().transform_vector(&Vector3::x_axis());
        let left: Vector3<f32>   = lookat.cross(&up); // Will be unit vector

        // TODO: Use inverse of m and n for better precision
        let p = HANG / HFOV as f32;
        let q = VANG / VFOV as f32;

        let mut outmatrix: SMatrix<f32, VFOV, HFOV> = SMatrix::zeros();

        for i in 0..HFOV {
            for j in 0..VFOV {
                let a = ((i - HFOV) as f32 * p).tan(); // Might be incorrect
                let b = ((j - VFOV) as f32 * q).tan();

                let ray = lookat + (a * left + b * up - lookat).normalize();

                // Finding the approximate distance to the nearest block
                outmatrix[(i, j)] = m.ray_probe(
                    (
                        x,
                        &ray
                    )
                );
            }
        }
        
        LaserImage { inner: outmatrix }
    }
    fn integrate_into_map(&self, x: &RobotPose, m: &mut OccupancyGrid) {
        // Important behaviour: Point3.to_homogeneous is different to Vector3
        // 1. Invert transformation of camera coords in terms of world frame
        let msb = Isometry3::<f32>::new(
            Vector3::new(x.x, x.y, 0.), // Zero by assumption of robot pose
            Vector3::new(0., 0., x.z),
        ).inverse();

        let up: Vector3<f32>     = Vector3::z_axis().into_inner(); // By assumption of RobotPose
        let lookat: Vector3<f32> = x.dir().transform_vector(&Vector3::x_axis());
        let left: Vector3<f32>   = lookat.cross(&up); // Will be unit vector

        let p = HANG / HFOV as f32;
        let q = VANG / VFOV as f32;

        for j in 0..VFOV {
            for i in 0..HFOV {
                let cnt = j * VFOV + i;

                let a = ((i - HFOV) as f32 * p).tan(); // Might be incorrect
                let b = ((j - VFOV) as f32 * q).tan();

                let ray = lookat + (a * left + b * up - lookat).normalize();
                let pt = Point3::new(ray.x, ray.y, ray.z); // TODO: Idiomatic conversion

                m[msb.transform_point(&pt)].intensity += 0.1;
            }
        }
    }
}

// Temporary trait; Use UOM conversion trait
// trait Scaled<M> {
//     // Deals with scaling from real life to internal values
//     // TODO: Reimplement with uom
//     fn scale(&self, m: Scalar) -> Scalar;
//     fn inv_scale(&self, n: Scalar) -> Scalar { n / self.scale(1) }
//     fn scale_measurement(&self, &SampleMeasurement<M>) -> &SampleMeasurement<M>;
// }
// 
// impl<M> Scaled<M> for OccupancyGrid {
//     fn scale(&self, m: Scalar) -> Scalar {
//         m * 1 // example. 1 grid cell is one metre in real life.
//     }
// }

///
///
///

fn vector3tovec3(v: Vector3<u32>) -> Vec3 {
    let mut m = [0., 0., 0.];
    for i in 0..2 {
        m[i] = v[i] as f32
    }
    m.into()
}
