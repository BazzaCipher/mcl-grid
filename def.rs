use nalgebra::*;
// use octree::{point::Point3D, Octree}; // Stand-in for https://github.com/csiro-robotics/ohm
use bvh::{
    aabb::{AABB,Bounded},
    bounding_hierarchy::BHShape,
    bvh::BVH,
    ray::Ray,
    Vector3 as Vec3,
};
use bno055::Error as BnoError;

// UOM here please
const RAY_MAXDIST: f64 = 5.0;
const DEFAULT_HEIGHT: f64 = 10.0;

// No generics ATM (but add later)
// Consider different library without floating point points
fn main() {
    println!("Hello, world!");
}

trait Pose {
    fn xy(&self) -> Vector3<f64>;
    fn dir(&self) -> Quaternion<f64>;
}

// Vector3 for how we do it rn, allow expansion into 3D via trait
/// Standard form: [x, y, theta]T. In degrees and metres
type RobotPose = Vector3<f64>;

impl Pose for RobotPose {
    fn xy(&self) -> Vector3<f64> {
        Vector3::new(self.0, self.1, DEFAULT_HEIGHT)
    }
    fn dir(&self) -> Quaternion<f64> { self.2 }
}

/// Abstracts the grid-map. Is a 3D representation with the floor being constant
/// Allows updating for integration of data.
trait GridMap {
    fn ray_probe(&self, ray: (&Vector3<u32>, &Vector3<f32>)) -> f64;
    fn insert(&mut self, x: &impl Pose);
    fn update(&mut self, x: &impl Pose, u: &impl Measurement);
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

// Do not assume 1:1 scale
impl GridMap for OccupancyGrid {
    fn ray_probe(&self, ray: (&Vector3<u32>, &Vector3<u32>)) -> Option<f64> {
        // Should model behaviour of sensor (max dist etc)
            let shapes = self.map.traverse_iterator(Ray::new(vector3tovec3(ray.0), vector3tovec3(ray.1)), &self.cells);
            shapes
                .map(|shape| distance(shape.position.into(), ray.0)) // TODO: Compare f64 dists instead
                .min()
    }
    // Inefficient in that every time the map is updated, the BVH is rebuilt. Consider Fast
    // insertion algorithms
    fn insert(&mut self, x: &RobotPose) {
        // TODO: Handle none 1:1 case
        self.cells.push(GridCell {
            position: *x,
            significance: 0.3, // Reasonable initial probability
            node_index: None,
        })
    }
    // Recall assumption of known position for particle filters
    fn update(&mut self, x: &RobotPose, u: &LaserImage) {
        // TODO: Handle none 1:1 case
        self.cells.
    }
}

// Assume every grid cell is its bottom left coord
#[derive(Debug, Clone)]
struct GridCell {
    position: Vector3<u32>,
    significance: f32, // Log-odds representation
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
    fn predict(&mut self, u: Vec<&impl Control>);
    /// Integration of actual measurements
    fn integrate(&mut self, am: &impl Measurement);
    fn best_estimate(&self) -> &impl Pose;
}

struct Particles<const P: usize> {
    // Only keep two sets wherein the particles can be an ex ante prediction or
    // ex post (after evidence)
    last_estimate: Vec<RobotPose>,
    particles: mut [(RobotPose, OccupancyGrid); P],
    is_pred: bool,
}

impl<const P: usize> BeliefState for Particles<P> {
    fn predict(&mut self, mut u: Vec<&Imu> { 
        /// We don't use any other than the latest due to integration into the map already
        /// Might make it non-Vec because we could just save it into the map
        let u = u.pop();

        particles
            .iter_mut()
            .map(|(pose, _)| u.motion_simulation(pose, m))
    }
    fn integrate(&mut self, am: &LaserImage) { 

    }
    fn best_estimate(&self) -> RobotPose { todo!() }
}

/// Encapsulates the simulation of controls; the kinematic model
/// Allows for prediction of Pr(x_t|x_t-1,u,m)
trait SampleControl {
    fn motion_simulation(&self, x: &impl Pose, m: &impl GridMap) -> impl Pose;
}

struct Imu {
    u: f32,
    w: f32, /// UOM: Deg/s
    last_dist_trav: f32,
    last_dist_turn: f32, /// UOM: Deg
}

// TODO: Eventually implement trait directly on Bno055??
impl Imu {
    fn update(&mut self, t: f32) -> Result<(), BnoError> {
        // This model suffers from drift heavily; either KF or 120+Hz updates
        // TODO: Implement Kalman for integration of accelerations
        let ud = t * self.lin_accel()?;
        let wd = t * self.ang_accel()?;
        self.last_dist_trav = (2 * self.u + ud) * t / 2;
        self.last_dist_turn = (2 * self.w + wd) * t / 2;
        
        self.u += ud;
        self.w += wd;
    }
    fn ang_accel(&mut self) -> Result<f32, BnoError> { todo!() }
    fn lin_accel(&mut self) -> Result<f32, BnoError> { todo!() }
}

impl SampleControl for Imu {
    fn motion_simulation(&self, x: &RobotPose, m: &OccupancyGrid) -> RobotPose {
        // Consider splitting into Jacobians Fp and Frl
        let angd = self.last_dist_turn / 2 + x.dir()

        // TODO: Model/overestimate and add error term
        x + Vector3::new(
            self.last_dist_trav * cos(angd),
            self.last_dist_trav * sin(angd),
            self.last_dist_turn,
        )
    }
}

/// Encapsulates the simulation of what the robot should see; the measurement
/// model. The robot should be able to measure some fixed area.
/// Simulates Pr(z,m|x,u)
trait SampleMeasurement<M> {
    fn measurement_simulation(&self, x: &impl Pose, u: &impl Control) -> M;
}

struct LaserImage {}

impl<M> SampleMeasurement<M> for LaserImage {
    fn measurement_simulation(&self, x: &RobotPose, u: &Imu) -> M {

    }

}

trait Scaled<M> {
    // Deals with scaling from real life to internal values
    // TODO: Reimplement with uom
    fn scale(&self, m: Scalar) -> Scalar;
    fn inv_scale(&self, n: Scalar) -> Scalar { n / scale(1) };
    fn scale_measurement(&self, &impl Measurement<M>) -> &impl Measurement<M>
}

impl Scaled for OccupancyGrid {
    fn scale(&self, m: Scalar) -> Scalar {
        m * 1 // example. 1 grid cell is one metre in real life.
    }
}

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
