//
// Google Benchmark for CollisionChecker using the real Athena robot.
// Compares single-pass (debug_viz ON) vs two-pass (debug_viz OFF) performance.
//
#include <benchmark/benchmark.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "safety_position_controller/collision_checker.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <fstream>
#include <random>

// CollisionChecker is in the global namespace

namespace
{

std::string loadFile( const std::string &filename )
{
  const std::string path =
      ament_index_cpp::get_package_share_directory( "hector_ros_controllers" ) + "/test/config/" +
      filename;
  std::ifstream ifs( path );
  if ( !ifs.is_open() ) {
    throw std::runtime_error( "Cannot open file: " + path );
  }
  return std::string( std::istreambuf_iterator<char>( ifs ), std::istreambuf_iterator<char>() );
}

// Athena arm joints
const std::vector<std::string> ARM_JOINTS = { "arm_joint_1", "arm_joint_2", "arm_joint_3",
                                              "arm_joint_4", "arm_joint_5", "arm_joint_6",
                                              "arm_joint_7" };

// Predefined arm poses from SRDF
struct ArmPose {
  std::string name;
  std::unordered_map<std::string, double> positions;
};

const std::vector<ArmPose> POSES = {
    { "folded",
      { { "arm_joint_1", 3.14 },
        { "arm_joint_2", 1.65 },
        { "arm_joint_3", 0.0 },
        { "arm_joint_4", -1.51 },
        { "arm_joint_5", 0.0 },
        { "arm_joint_6", -1.38 },
        { "arm_joint_7", 0.0 } } },
    { "front",
      { { "arm_joint_1", 0.0 },
        { "arm_joint_2", 0.4 },
        { "arm_joint_3", 0.0 },
        { "arm_joint_4", -0.4 },
        { "arm_joint_5", 3.14 },
        { "arm_joint_6", 0.9 },
        { "arm_joint_7", 0.0 } } },
    { "zero",
      { { "arm_joint_1", 0.0 },
        { "arm_joint_2", 0.0 },
        { "arm_joint_3", 0.0 },
        { "arm_joint_4", 0.0 },
        { "arm_joint_5", 0.0 },
        { "arm_joint_6", 0.0 },
        { "arm_joint_7", 0.0 } } },
};

/// Shared state for benchmarks (avoids re-parsing URDF per iteration)
struct BenchmarkState {
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::string urdf_xml;
  std::string srdf_xml;
  pinocchio::Model model;

  // Pre-generated random configs (Pinocchio q-space, handles cos/sin for continuous joints)
  std::vector<std::unordered_map<std::string, double>> random_configs;

  static BenchmarkState &instance()
  {
    static BenchmarkState s;
    return s;
  }

  void init()
  {
    if ( node )
      return; // already initialized
    rclcpp::NodeOptions opts;
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>( "benchmark_collision_checker", opts );
    urdf_xml = loadFile( "athena.urdf" );
    srdf_xml = loadFile( "athena.srdf" );
    pinocchio::urdf::buildModelFromXML( urdf_xml, model );

    // Generate random configs
    std::mt19937 rng( 42 );
    std::uniform_real_distribution<double> dist( -M_PI, M_PI );
    random_configs.resize( 500 );
    for ( auto &config : random_configs ) {
      for ( const auto &joint : ARM_JOINTS ) { config[joint] = dist( rng ); }
    }
  }

  std::unique_ptr<CollisionChecker> makeChecker( double padding, bool debug_viz )
  {
    auto checker = std::make_unique<CollisionChecker>( node, padding, 0.0, debug_viz );
    checker->setBroadphase( false ); // brute-force baseline
    bool ok = checker->initFromXml( urdf_xml, srdf_xml, ARM_JOINTS );
    if ( !ok )
      throw std::runtime_error( "Failed to init CollisionChecker" );
    return checker;
  }

  std::unique_ptr<CollisionChecker> makeBroadphaseChecker( double padding, bool debug_viz )
  {
    auto checker = std::make_unique<CollisionChecker>( node, padding, 0.0, debug_viz );
    checker->setBroadphase( true );
    bool ok = checker->initFromXml( urdf_xml, srdf_xml, ARM_JOINTS );
    if ( !ok )
      throw std::runtime_error( "Failed to init broadphase CollisionChecker" );
    return checker;
  }
};

// ---- Benchmark: Two-pass (debug_viz OFF) with random configs ----
void BM_CollisionChecker_TwoPass( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeChecker( padding, /*debug_viz=*/false );

  std::size_t i = 0;
  for ( auto _ : state ) {
    const auto &config = bs.random_configs[i % bs.random_configs.size()];
    auto result = checker->checkCollision( config, safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
  state.counters["pairs"] = static_cast<double>( checker->getNumCollisionPairs() );
}
BENCHMARK( BM_CollisionChecker_TwoPass )->Unit( benchmark::kMicrosecond );

// ---- Benchmark: Single-pass (debug_viz ON) with random configs ----
void BM_CollisionChecker_SinglePass( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeChecker( padding, /*debug_viz=*/true );

  std::size_t i = 0;
  for ( auto _ : state ) {
    const auto &config = bs.random_configs[i % bs.random_configs.size()];
    auto result = checker->checkCollision( config, safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
  state.counters["pairs"] = static_cast<double>( checker->getNumCollisionPairs() );
}
BENCHMARK( BM_CollisionChecker_SinglePass )->Unit( benchmark::kMicrosecond );

// Generate configs with small random perturbations around a base pose (defeats cache)
std::vector<std::unordered_map<std::string, double>>
generatePerturbedConfigs( const std::unordered_map<std::string, double> &base, std::size_t count )
{
  std::mt19937 rng( 123 );
  std::uniform_real_distribution<double> perturb( -0.1, 0.1 );
  std::vector<std::unordered_map<std::string, double>> configs( count );
  for ( auto &config : configs ) {
    for ( const auto &[name, val] : base ) { config[name] = val + perturb( rng ); }
  }
  return configs;
}

// ---- Benchmark: Two-pass with perturbed folded poses (near collision) ----
void BM_CollisionChecker_TwoPass_Folded( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeChecker( padding, /*debug_viz=*/false );
  auto configs = generatePerturbedConfigs( POSES[0].positions, 500 );

  std::size_t i = 0;
  for ( auto _ : state ) {
    auto result = checker->checkCollision( configs[i % configs.size()], safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
}
BENCHMARK( BM_CollisionChecker_TwoPass_Folded )->Unit( benchmark::kMicrosecond );

// ---- Benchmark: Single-pass with perturbed folded poses (near collision) ----
void BM_CollisionChecker_SinglePass_Folded( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeChecker( padding, /*debug_viz=*/true );
  auto configs = generatePerturbedConfigs( POSES[0].positions, 500 );

  std::size_t i = 0;
  for ( auto _ : state ) {
    auto result = checker->checkCollision( configs[i % configs.size()], safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
}
BENCHMARK( BM_CollisionChecker_SinglePass_Folded )->Unit( benchmark::kMicrosecond );

// ---- Benchmark: Two-pass, no safety zone (distance-only, no gradients) ----
void BM_CollisionChecker_DistanceOnly( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  auto checker = bs.makeChecker( padding, /*debug_viz=*/false );

  std::size_t i = 0;
  for ( auto _ : state ) {
    const auto &config = bs.random_configs[i % bs.random_configs.size()];
    auto result = checker->checkCollision( config, 0.0 );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
  state.counters["pairs"] = static_cast<double>( checker->getNumCollisionPairs() );
}
BENCHMARK( BM_CollisionChecker_DistanceOnly )->Unit( benchmark::kMicrosecond );

// ======== Broadphase variants ========

// ---- Benchmark: Broadphase two-pass (debug_viz OFF) with random configs ----
void BM_Broadphase_TwoPass( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeBroadphaseChecker( padding, /*debug_viz=*/false );

  std::size_t i = 0;
  for ( auto _ : state ) {
    const auto &config = bs.random_configs[i % bs.random_configs.size()];
    auto result = checker->checkCollision( config, safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
  state.counters["pairs"] = static_cast<double>( checker->getNumCollisionPairs() );
}
BENCHMARK( BM_Broadphase_TwoPass )->Unit( benchmark::kMicrosecond );

// ---- Benchmark: Broadphase single-pass (debug_viz ON) with random configs ----
void BM_Broadphase_SinglePass( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeBroadphaseChecker( padding, /*debug_viz=*/true );

  std::size_t i = 0;
  for ( auto _ : state ) {
    const auto &config = bs.random_configs[i % bs.random_configs.size()];
    auto result = checker->checkCollision( config, safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
  state.counters["pairs"] = static_cast<double>( checker->getNumCollisionPairs() );
}
BENCHMARK( BM_Broadphase_SinglePass )->Unit( benchmark::kMicrosecond );

// ---- Benchmark: Broadphase two-pass with perturbed folded poses (near collision) ----
void BM_Broadphase_TwoPass_Folded( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  constexpr double safety_zone = 0.05;
  auto checker = bs.makeBroadphaseChecker( padding, /*debug_viz=*/false );
  auto configs = generatePerturbedConfigs( POSES[0].positions, 500 );

  std::size_t i = 0;
  for ( auto _ : state ) {
    auto result = checker->checkCollision( configs[i % configs.size()], safety_zone );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
}
BENCHMARK( BM_Broadphase_TwoPass_Folded )->Unit( benchmark::kMicrosecond );

// ---- Benchmark: Broadphase distance-only (no gradients) ----
void BM_Broadphase_DistanceOnly( benchmark::State &state )
{
  auto &bs = BenchmarkState::instance();
  constexpr double padding = 0.01;
  auto checker = bs.makeBroadphaseChecker( padding, /*debug_viz=*/false );

  std::size_t i = 0;
  for ( auto _ : state ) {
    const auto &config = bs.random_configs[i % bs.random_configs.size()];
    auto result = checker->checkCollision( config, 0.0 );
    benchmark::DoNotOptimize( result );
    ++i;
  }
  state.SetItemsProcessed( static_cast<int64_t>( state.iterations() ) );
  state.counters["pairs"] = static_cast<double>( checker->getNumCollisionPairs() );
}
BENCHMARK( BM_Broadphase_DistanceOnly )->Unit( benchmark::kMicrosecond );

} // namespace

int main( int argc, char **argv )
{
  rclcpp::init( argc, argv );
  BenchmarkState::instance().init();

  benchmark::Initialize( &argc, argv );
  benchmark::RunSpecifiedBenchmarks();
  benchmark::Shutdown();

  BenchmarkState::instance().node.reset();
  rclcpp::shutdown();

#if defined( __GNUC__ )
  extern void __gcov_dump() __attribute__( ( weak ) );
  if ( __gcov_dump )
    __gcov_dump();
#endif
  _exit( 0 );
}
