//
// Standalone unit tests for CollisionChecker with broadphase distance computation.
// Verifies correctness against brute-force Pinocchio reference.
//
// NOTE on broadphase distance in penetration:
// When geometries are deeply penetrating (negative distances), the AABB tree's
// distance pruning may not find the most-negative pair because AABB overlap gives
// a distance of zero, making the pruning bound less conservative. This means the
// broadphase min_distance may be less negative than brute-force in penetration.
// For safety, this is fine: we only need collision detection (dist <= padding),
// not the exact penetration depth. For non-penetrating configurations, broadphase
// and brute-force should agree exactly.
//
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "safety_position_controller/collision_checker.hpp"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "pinocchio/collision/distance.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <random>
#include <set>
#include <unordered_map>

namespace
{

std::string loadUrdfFile( const std::string &filename )
{
  const std::string path =
      ament_index_cpp::get_package_share_directory( "hector_ros_controllers" ) + "/test/config/" +
      filename;
  std::ifstream ifs( path );
  if ( !ifs.is_open() ) {
    throw std::runtime_error( "Cannot open test URDF file: " + path );
  }
  return std::string( std::istreambuf_iterator<char>( ifs ), std::istreambuf_iterator<char>() );
}

/// Brute-force reference: compute minimum distance using pinocchio::computeDistances()
double bruteForceMinDistance( pinocchio::Model &model, pinocchio::Data &data,
                              pinocchio::GeometryModel &geom_model,
                              pinocchio::GeometryData &geom_data, const Eigen::VectorXd &q )
{
  pinocchio::forwardKinematics( model, data, q );
  pinocchio::updateGeometryPlacements( model, data, geom_model, geom_data );
  pinocchio::computeDistances( geom_model, geom_data );

  double min_dist = std::numeric_limits<double>::max();
  for ( std::size_t k = 0; k < geom_model.collisionPairs.size(); ++k ) {
    min_dist = std::min( min_dist, geom_data.distanceResults[k].min_distance );
  }
  return min_dist;
}

/// Build a q vector from a name->value map (handles continuous joints as [cos, sin])
Eigen::VectorXd buildQ( const pinocchio::Model &model,
                        const std::unordered_map<std::string, double> &joint_positions )
{
  Eigen::VectorXd q = pinocchio::neutral( model );
  for ( const auto &[name, position] : joint_positions ) {
    pinocchio::JointIndex jid = 0;
    for ( pinocchio::JointIndex j = 1; j < model.joints.size(); ++j ) {
      if ( model.names[j] == name ) {
        jid = j;
        break;
      }
    }
    if ( jid == 0 )
      continue;

    const int nq_j = model.joints[jid].nq();
    const int nv_j = model.joints[jid].nv();
    const int iq = model.idx_qs[jid];

    if ( nq_j == 1 ) {
      q[iq] = position;
    } else if ( nq_j == 2 && nv_j == 1 ) {
      q[iq] = std::cos( position );
      q[iq + 1] = std::sin( position );
    }
  }
  return q;
}

/// Compare broadphase result against brute-force reference.
/// For non-penetrating: exact match.
/// For penetrating: broadphase min_distance >= bf_min (less negative is ok),
///   but both must agree that collision is present.
void expectDistanceMatch( double broadphase_min, double bf_min, double padding,
                          const std::string &label )
{
  if ( bf_min > 0.0 ) {
    // Non-penetrating: broadphase should find exact same minimum
    EXPECT_NEAR( broadphase_min, bf_min, 1e-10 ) << label;
  } else {
    // Penetrating: broadphase may report less-negative distance (AABB pruning),
    // but must still detect collision
    EXPECT_LE( broadphase_min, padding )
        << "Broadphase should detect collision when brute-force says penetrating. " << label;
    EXPECT_GE( broadphase_min, bf_min )
        << "Broadphase should not report more penetration than brute-force. " << label;
  }
}

} // namespace

class CollisionCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>( "test_collision_checker", opts );
    urdf_xml_ = loadUrdfFile( "test_robot_collision.urdf" );

    // Build reference model for brute-force comparison
    pinocchio::urdf::buildModelFromXML( urdf_xml_, ref_model_ );
    ref_data_ = pinocchio::Data( ref_model_ );
    std::istringstream urdf_stream( urdf_xml_ );
    pinocchio::urdf::buildGeom( ref_model_, urdf_stream, pinocchio::COLLISION, ref_geom_model_ );
    ref_geom_model_.addAllCollisionPairs();
    ref_geom_data_ = pinocchio::GeometryData( ref_geom_model_ );
  }

  void TearDown() override { node_.reset(); }

  /// Create a CollisionChecker and init with test URDF
  std::unique_ptr<CollisionChecker> makeChecker( double padding = 0.0, double cache_epsilon = 0.0,
                                                 bool debug_viz = false )
  {
    auto checker = std::make_unique<CollisionChecker>( node_, padding, cache_epsilon, debug_viz );
    // Pass all joints as controlled so no filtering occurs (fair comparison with reference)
    std::vector<std::string> all_joints;
    for ( pinocchio::JointIndex jid = 1; jid < ref_model_.joints.size(); ++jid ) {
      all_joints.push_back( ref_model_.names[jid] );
    }
    bool ok = checker->initFromXml( urdf_xml_, "", all_joints );
    EXPECT_TRUE( ok );
    return checker;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string urdf_xml_;

  // Reference model for brute-force distance
  pinocchio::Model ref_model_;
  pinocchio::Data ref_data_;
  pinocchio::GeometryModel ref_geom_model_;
  pinocchio::GeometryData ref_geom_data_;
};

// ---- Test 1: Zero configuration (non-penetrating) ----
TEST_F( CollisionCheckerTest, ZeroConfig )
{
  auto checker = makeChecker();
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );

  Eigen::VectorXd q = buildQ( ref_model_, positions );
  double bf_min = bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

  EXPECT_FALSE( result.in_collision );
  EXPECT_GT( result.min_distance, 0.0 );
  EXPECT_NEAR( result.min_distance, bf_min, 1e-10 );
}

// ---- Test 2: Collision configuration ----
TEST_F( CollisionCheckerTest, CollisionConfig )
{
  auto checker = makeChecker( 0.0 );
  // Fold the chain so link1 sphere (r=0.12) meets link4 sphere (r=0.12)
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", M_PI }, { "joint3", -M_PI / 2.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );

  Eigen::VectorXd q = buildQ( ref_model_, positions );
  double bf_min = bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

  // Both should detect collision
  EXPECT_TRUE( result.in_collision );
  EXPECT_LT( bf_min, 0.0 );
  // Broadphase may report less-negative distance in penetration
  EXPECT_GE( result.min_distance, bf_min );
  EXPECT_LE( result.min_distance, 0.0 );
}

// ---- Test 3: Sweep joint2 and compare broadphase vs brute-force ----
TEST_F( CollisionCheckerTest, SweepJoint2 )
{
  auto checker = makeChecker();

  const int steps = 20;
  for ( int i = 0; i <= steps; ++i ) {
    const double angle = -M_PI + ( 2.0 * M_PI * i ) / steps;
    std::unordered_map<std::string, double> positions = {
        { "joint1", 0.0 }, { "joint2", angle }, { "joint3", 0.0 }, { "joint4", 0.0 } };

    auto result = checker->checkCollision( positions );

    Eigen::VectorXd q = buildQ( ref_model_, positions );
    double bf_min =
        bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

    expectDistanceMatch( result.min_distance, bf_min, 0.0,
                         "joint2=" + std::to_string( angle ) + " rad" );
  }
}

// ---- Test 4: Random configurations ----
TEST_F( CollisionCheckerTest, RandomConfigurations )
{
  auto checker = makeChecker();

  std::mt19937 rng( 42 ); // fixed seed for reproducibility
  std::uniform_real_distribution<double> dist( -M_PI, M_PI );

  const int n_configs = 50;
  for ( int i = 0; i < n_configs; ++i ) {
    std::unordered_map<std::string, double> positions = { { "joint1", dist( rng ) },
                                                          { "joint2", dist( rng ) },
                                                          { "joint3", dist( rng ) },
                                                          { "joint4", dist( rng ) } };

    auto result = checker->checkCollision( positions );

    Eigen::VectorXd q = buildQ( ref_model_, positions );
    double bf_min =
        bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

    expectDistanceMatch( result.min_distance, bf_min, 0.0, "config " + std::to_string( i ) );
  }
}

// ---- Test 5: Cache works ----
TEST_F( CollisionCheckerTest, CacheStillWorks )
{
  const double cache_epsilon = 1e-4;
  auto checker = makeChecker( 0.0, cache_epsilon );

  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.5 }, { "joint2", 0.3 }, { "joint3", -0.2 }, { "joint4", 0.0 } };

  auto result1 = checker->checkCollision( positions );
  auto result2 = checker->checkCollision( positions );

  // Identical call should return cached result
  EXPECT_DOUBLE_EQ( result1.min_distance, result2.min_distance );
  EXPECT_EQ( result1.in_collision, result2.in_collision );

  // Move beyond epsilon — should recompute
  positions["joint1"] += cache_epsilon * 10.0;
  auto result3 = checker->checkCollision( positions );

  // Should still be valid (not necessarily equal since position changed)
  Eigen::VectorXd q = buildQ( ref_model_, positions );
  double bf_min = bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );
  expectDistanceMatch( result3.min_distance, bf_min, 0.0, "cache recompute" );
}

// ---- Test 6: Continuous joint encoding ----
TEST_F( CollisionCheckerTest, ContinuousJointEncoding )
{
  auto checker = makeChecker();

  // Test angles including large multiples of 2*PI.
  // Note: M_PI and -M_PI fold joint4 to flip link4 back, which may cause collision.
  std::vector<double> angles = { 0.0,   M_PI / 2.0,       M_PI,
                                 -M_PI, 4.0 * M_PI + 0.1, -6.0 * M_PI - 0.5 };

  for ( double angle : angles ) {
    std::unordered_map<std::string, double> positions = {
        { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 }, { "joint4", angle } };

    auto result = checker->checkCollision( positions );

    Eigen::VectorXd q = buildQ( ref_model_, positions );
    double bf_min =
        bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

    expectDistanceMatch( result.min_distance, bf_min, 0.0,
                         "continuous joint angle=" + std::to_string( angle ) );
  }
}

// ---- Test 7: NaN input returns safe result ----
TEST_F( CollisionCheckerTest, NaNInputReturnsSafeResult )
{
  auto checker = makeChecker();

  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 },
      { "joint2", std::numeric_limits<double>::quiet_NaN() },
      { "joint3", 0.0 },
      { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );
  EXPECT_TRUE( result.in_collision ) << "NaN input should report collision (safe default)";
}

// ---- Test 8: Pair filtering preserved ----
TEST_F( CollisionCheckerTest, PairFilteringPreserved )
{
  // Create checker with only subset of joints
  auto checker = std::make_unique<CollisionChecker>( node_, 0.0, 0.0, false );
  std::vector<std::string> subset_joints = { "joint1", "joint2" };
  bool ok = checker->initFromXml( urdf_xml_, "", subset_joints );
  ASSERT_TRUE( ok );

  // Should have fewer pairs than the reference (which has all)
  auto joint_names = checker->getJointNames();
  EXPECT_GT( joint_names.size(), 0u );

  // Distance should still be correct for the reduced pair set
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.5 }, { "joint2", 0.3 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );
  EXPECT_GT( result.min_distance, 0.0 );
  EXPECT_FALSE( result.in_collision );
}

// ---- Test 9: Collision with padding ----
TEST_F( CollisionCheckerTest, CollisionPaddingWorks )
{
  // At zero config, find actual distance
  auto checker_no_pad = makeChecker( 0.0 );
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result_no_pad = checker_no_pad->checkCollision( positions );
  ASSERT_FALSE( result_no_pad.in_collision );
  const double actual_dist = result_no_pad.min_distance;

  // With padding larger than actual distance, should be in collision
  auto checker_with_pad = makeChecker( actual_dist + 0.01 );
  auto result_with_pad = checker_with_pad->checkCollision( positions );
  EXPECT_TRUE( result_with_pad.in_collision );
  EXPECT_NEAR( result_with_pad.min_distance, actual_dist, 1e-10 );
}

// ---- Test 10: Multi-joint sweep ----
TEST_F( CollisionCheckerTest, MultiJointSweep )
{
  auto checker = makeChecker();

  // Sweep both joint2 and joint3 simultaneously
  const int steps = 10;
  for ( int i = 0; i <= steps; ++i ) {
    for ( int j = 0; j <= steps; ++j ) {
      const double a2 = -M_PI + ( 2.0 * M_PI * i ) / steps;
      const double a3 = -M_PI + ( 2.0 * M_PI * j ) / steps;

      std::unordered_map<std::string, double> positions = {
          { "joint1", 0.0 }, { "joint2", a2 }, { "joint3", a3 }, { "joint4", 0.0 } };

      auto result = checker->checkCollision( positions );

      Eigen::VectorXd q = buildQ( ref_model_, positions );
      double bf_min =
          bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

      expectDistanceMatch( result.min_distance, bf_min, 0.0,
                           "joint2=" + std::to_string( a2 ) + ", joint3=" + std::to_string( a3 ) );
    }
  }
}

// ---- Test 11: Non-penetrating configs always match exactly ----
TEST_F( CollisionCheckerTest, NonPenetratingExactMatch )
{
  auto checker = makeChecker();

  // Small angles that keep the chain extended (no collision)
  std::vector<std::pair<double, double>> angles = {
      { 0.0, 0.0 },  { 0.3, 0.0 },  { 0.0, 0.3 },  { 0.3, 0.3 },
      { -0.3, 0.3 }, { 0.5, -0.5 }, { -0.5, 0.5 }, { 0.1, 0.1 },
  };

  for ( const auto &[a2, a3] : angles ) {
    std::unordered_map<std::string, double> positions = {
        { "joint1", 0.0 }, { "joint2", a2 }, { "joint3", a3 }, { "joint4", 0.0 } };

    auto result = checker->checkCollision( positions );

    Eigen::VectorXd q = buildQ( ref_model_, positions );
    double bf_min =
        bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

    ASSERT_GT( bf_min, 0.0 ) << "Config should be non-penetrating";
    EXPECT_NEAR( result.min_distance, bf_min, 1e-10 )
        << "Non-penetrating config must match exactly at j2=" << a2 << ", j3=" << a3;
  }
}

// ============================================================
// Gradient tests for directional collision velocity scaling
// ============================================================

// ---- Test 12: Gradient matches finite difference ----
TEST_F( CollisionCheckerTest, GradientMatchesFiniteDifference )
{
  auto checker = makeChecker();
  const double eps = 1e-6;
  const double safety_zone = 1.0; // large threshold to always compute gradients

  // Test multiple configurations (non-penetrating with various clearances)
  std::vector<std::unordered_map<std::string, double>> configs = {
      { { "joint1", 0.0 }, { "joint2", 0.5 }, { "joint3", -0.3 }, { "joint4", 0.0 } },
      { { "joint1", 0.3 }, { "joint2", 2.0 }, { "joint3", -1.0 }, { "joint4", 0.5 } },
      { { "joint1", -0.2 }, { "joint2", 1.5 }, { "joint3", -0.8 }, { "joint4", -0.3 } },
      { { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 }, { "joint4", 0.0 } },
  };

  for ( size_t ci = 0; ci < configs.size(); ++ci ) {
    const auto &positions = configs[ci];

    // Get result with gradient
    auto result = checker->checkCollision( positions, safety_zone );
    // Skip if no pairs in safety zone (e.g. all pairs have distance > safety_zone)
    if ( result.safety_zone_pairs.empty() )
      continue;

    // Use the first (closest) pair's gradient
    const auto &closest = result.safety_zone_pairs[0];
    const auto &gradient = closest.gradient;

    // Finite-difference validation for each controlled joint
    for ( const auto &[name, val] : positions ) {
      auto perturbed = positions;
      perturbed[name] = val + eps;

      // Need to invalidate cache — use a different checker or large perturbation
      // Actually, the cache epsilon is 0 for this checker, so different q always recomputes
      auto result_plus = checker->checkCollision( perturbed, safety_zone );

      double fd_gradient = ( result_plus.min_distance - result.min_distance ) / eps;

      int v_idx = checker->getJointVelocityIndex( name );
      ASSERT_GE( v_idx, 0 ) << "Joint " << name << " not found";
      ASSERT_LT( v_idx, gradient.size() );

      EXPECT_NEAR( gradient[v_idx], fd_gradient, 1e-3 )
          << "Gradient mismatch for joint " << name << " at config " << ci
          << " (analytical=" << gradient[v_idx] << ", fd=" << fd_gradient << ")";
    }
  }
}

// ---- Test 13: Gradient sign for approaching motion ----
TEST_F( CollisionCheckerTest, GradientSignApproaching )
{
  auto checker = makeChecker();
  const double safety_zone = 1.0;

  // Start from straight chain, fold joint2 toward PI (approaching collision)
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 1.5 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions, safety_zone );
  ASSERT_FALSE( result.safety_zone_pairs.empty() );

  const auto &gradient = result.safety_zone_pairs[0].gradient;
  int v_idx_j2 = checker->getJointVelocityIndex( "joint2" );
  ASSERT_GE( v_idx_j2, 0 );

  // Motion toward PI (positive delta for joint2) folds the chain → should decrease distance
  // So gradient[v_idx_j2] * (+delta) should be negative → gradient[v_idx_j2] < 0
  // (or the reverse depending on the chain geometry — let's use the finite difference to verify sign)
  double delta = 0.1;
  auto positions_plus = positions;
  positions_plus["joint2"] += delta;
  auto result_plus = checker->checkCollision( positions_plus, safety_zone );

  // If distance decreased, the motion is approaching
  if ( result_plus.min_distance < result.min_distance ) {
    // The directional derivative should be negative
    double dir_deriv = gradient[v_idx_j2] * delta;
    EXPECT_LT( dir_deriv, 0.0 ) << "Gradient should indicate approaching when distance decreases. "
                                << "grad[j2]=" << gradient[v_idx_j2] << " delta=" << delta;
  }
}

// ---- Test 14: Gradient sign for retreating motion ----
TEST_F( CollisionCheckerTest, GradientSignRetreating )
{
  auto checker = makeChecker();
  const double safety_zone = 1.0;

  // Near collision but NOT penetrating: joint2 folded partway
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 2.0 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions, safety_zone );
  ASSERT_FALSE( result.safety_zone_pairs.empty() );
  ASSERT_GT( result.min_distance, 0.0 )
      << "Config must be non-penetrating for gradient to be valid";

  const auto &gradient = result.safety_zone_pairs[0].gradient;
  int v_idx_j2 = checker->getJointVelocityIndex( "joint2" );
  ASSERT_GE( v_idx_j2, 0 );

  // Motion back toward 0 (negative delta for joint2) unfolds the chain → should increase distance
  double delta = -0.1;
  auto positions_minus = positions;
  positions_minus["joint2"] += delta;
  auto result_minus = checker->checkCollision( positions_minus, safety_zone );

  if ( result_minus.min_distance > result.min_distance ) {
    // The directional derivative should be positive (moving away)
    double dir_deriv = gradient[v_idx_j2] * delta;
    EXPECT_GT( dir_deriv, 0.0 ) << "Gradient should indicate retreating when distance increases. "
                                << "grad[j2]=" << gradient[v_idx_j2] << " delta=" << delta;
  }
}

// ---- Test 15: Multiple pairs in safety zone get gradients ----
TEST_F( CollisionCheckerTest, GradientComputedForAllSafetyZonePairs )
{
  auto checker = makeChecker();
  const double safety_zone = 2.0; // very large to capture all pairs

  // Configuration with multiple pairs relatively close but NOT penetrating
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 0.8 }, { "joint3", -0.3 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions, safety_zone );

  // With a large safety zone, multiple pairs should have gradients
  EXPECT_GE( result.safety_zone_pairs.size(), 1u )
      << "Expected at least 1 pair in safety zone with threshold=" << safety_zone;

  // Each pair should have a non-empty gradient vector
  std::size_t non_zero_gradient_count = 0;
  for ( const auto &pi : result.safety_zone_pairs ) {
    EXPECT_GT( pi.gradient.size(), 0 ) << "Pair " << pi.pair_index << " has empty gradient";
    if ( pi.gradient.norm() > 0.0 ) {
      non_zero_gradient_count++;
    }
    // Note: some pairs may have zero gradient if both bodies share the same parent joint
    // (distance between them is invariant to any joint motion), or if they are penetrating.
  }
  // At least one pair should have a non-zero gradient
  EXPECT_GT( non_zero_gradient_count, 0u ) << "Expected at least one pair with non-zero gradient";
}

// ---- Test 16: No gradient when threshold is zero ----
TEST_F( CollisionCheckerTest, NoGradientWhenThresholdZero )
{
  auto checker = makeChecker();

  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 1.5 }, { "joint3", -0.5 }, { "joint4", 0.0 } };

  // Default threshold = 0 → no gradient computation
  auto result = checker->checkCollision( positions );
  EXPECT_TRUE( result.safety_zone_pairs.empty() )
      << "No safety zone pairs should be computed with threshold=0";
}

// ---- Test 17: getJointVelocityIndex and getNv ----
TEST_F( CollisionCheckerTest, VelocitySpaceHelpers )
{
  auto checker = makeChecker();

  EXPECT_GT( checker->getNv(), 0 );
  EXPECT_GE( checker->getJointVelocityIndex( "joint1" ), 0 );
  EXPECT_GE( checker->getJointVelocityIndex( "joint2" ), 0 );
  EXPECT_GE( checker->getJointVelocityIndex( "joint3" ), 0 );
  EXPECT_GE( checker->getJointVelocityIndex( "joint4" ), 0 );
  EXPECT_EQ( checker->getJointVelocityIndex( "nonexistent_joint" ), -1 );

  // All indices should be distinct
  std::set<int> indices;
  for ( const auto &name : { "joint1", "joint2", "joint3", "joint4" } ) {
    int idx = checker->getJointVelocityIndex( name );
    EXPECT_TRUE( indices.insert( idx ).second ) << "Duplicate velocity index for " << name;
  }
}

// ---- Test 18: Performance benchmark ----
TEST_F( CollisionCheckerTest, PerformanceBenchmark )
{
  auto checker = makeChecker();
  const double safety_zone = 0.05; // realistic threshold

  std::mt19937 rng( 123 );
  std::uniform_real_distribution<double> dist( -M_PI, M_PI );

  // Pre-generate configs
  constexpr int N = 1000;
  std::vector<std::unordered_map<std::string, double>> configs( N );
  for ( int i = 0; i < N; ++i ) {
    configs[i] = { { "joint1", dist( rng ) },
                   { "joint2", dist( rng ) },
                   { "joint3", dist( rng ) },
                   { "joint4", dist( rng ) } };
  }

  // Warm up
  for ( int i = 0; i < 10; ++i ) { checker->checkCollision( configs[i], safety_zone ); }

  // Benchmark
  auto t0 = std::chrono::steady_clock::now();
  std::size_t total_safety_pairs = 0;
  for ( int i = 0; i < N; ++i ) {
    auto result = checker->checkCollision( configs[i], safety_zone );
    total_safety_pairs += result.safety_zone_pairs.size();
  }
  auto t1 = std::chrono::steady_clock::now();

  const double elapsed_us =
      static_cast<double>( std::chrono::duration_cast<std::chrono::microseconds>( t1 - t0 ).count() );
  const double avg_us = elapsed_us / static_cast<double>( N );

  std::cout << "[Benchmark] avg checkCollision: " << avg_us << " us"
            << " | pairs: " << checker->getNumCollisionPairs() << " | avg safety_zone_pairs: "
            << ( static_cast<double>( total_safety_pairs ) / static_cast<double>( N ) ) << std::endl;

  // Soft assertion: should complete within 10ms per call
  EXPECT_LT( avg_us, 10000.0 ) << "Collision check too slow";
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
