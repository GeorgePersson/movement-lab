//! Original controller laboratory guided by partial static recovery.
//! Tuning defaults are approximate and have not been fitted to gameplay.
pub mod bullet;
pub mod capsule_geometry;
pub mod capsule_mtd;
pub mod capsule_sweep;
pub mod cct_response;
pub mod cct_policy;
pub mod cct_overlap;
pub mod cct_scene;
pub mod combo;
pub mod controller;
pub mod gravity;
pub mod impulse;
pub mod jump_animation;
pub mod jump_graph;
pub mod jump_lifecycle;
pub mod landing;
pub mod precise_capsule;
pub mod precise_distance;
pub mod precise_sphere;
pub mod precise_sweep;
pub mod velocity;
pub mod wall_entry;
pub mod wall_jump;
pub mod world;

pub use controller::{Input, MovementState, Player, Tuning};
pub use world::{Solid, World};

pub mod contact_reports;
pub mod contact_finalizer;
pub mod engine_move;
pub mod engine_scene;
pub mod request_adjustment;
pub mod cct_slope;
pub mod capsule_mesh;
pub mod capsule_bounds;
pub mod segment_triangle;
pub mod mesh_publication;
pub mod mesh_iteration;
pub mod mesh_static_support;

pub mod static_terrain;
pub mod engine_mesh_scene;

pub mod slide_slope;
pub mod wall_query;
pub mod wall_traversal;
