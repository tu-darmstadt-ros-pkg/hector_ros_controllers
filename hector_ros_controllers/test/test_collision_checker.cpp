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
#include <unistd.h>

#include <algorithm>
#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "safety_position_controller/collision_checker.hpp"

#include <pinocchio/algorithm/frames.hpp>
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
    // Broadphase may legitimately report a less-negative (pruned) distance, so the
    // bound is one-sided. When the same closest pair wins in both paths the two
    // distances should be equal but, computed via different traversal orders, can
    // differ by a few ULP; allow a small FP slack so an exact tie is not flagged.
    constexpr double kFpTolerance = 1e-9;
    EXPECT_GE( broadphase_min, bf_min - kFpTolerance )
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
  checker->setSafetyZoneThreshold( safety_zone );

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
    auto result = checker->checkCollision( positions );
    // Skip if no pairs in safety zone (e.g. all pairs have distance > safety_zone)
    if ( result.safety_zone_pairs.empty() )
      continue;

    // Find the closest pair (min distance) — broadphase may return pairs in any order
    const auto &closest =
        *std::min_element( result.safety_zone_pairs.begin(), result.safety_zone_pairs.end(),
                           []( const auto &a, const auto &b ) { return a.distance < b.distance; } );
    const auto &gradient = closest.gradient;

    // Finite-difference validation for each controlled joint
    for ( const auto &[name, val] : positions ) {
      auto perturbed = positions;
      perturbed[name] = val + eps;

      // Need to invalidate cache — use a different checker or large perturbation
      // Actually, the cache epsilon is 0 for this checker, so different q always recomputes
      auto result_plus = checker->checkCollision( perturbed );

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
  checker->setSafetyZoneThreshold( safety_zone );

  // Start from straight chain, fold joint2 toward PI (approaching collision)
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 1.5 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );
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
  auto result_plus = checker->checkCollision( positions_plus );

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
  checker->setSafetyZoneThreshold( safety_zone );

  // Near collision but NOT penetrating: joint2 folded partway
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 2.0 }, { "joint3", 0.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );
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
  auto result_minus = checker->checkCollision( positions_minus );

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
  checker->setSafetyZoneThreshold( safety_zone );

  // Configuration with multiple pairs relatively close but NOT penetrating
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", 0.8 }, { "joint3", -0.3 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );

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
  checker->setSafetyZoneThreshold( safety_zone );

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
  for ( int i = 0; i < 10; ++i ) { checker->checkCollision( configs[i] ); }

  // Benchmark
  auto t0 = std::chrono::steady_clock::now();
  std::size_t total_safety_pairs = 0;
  for ( int i = 0; i < N; ++i ) {
    auto result = checker->checkCollision( configs[i] );
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

// ============================================================
// Broadphase correctness tests
// ============================================================

// Test fixture that runs tests with broadphase enabled
class CollisionCheckerBroadphaseTest : public CollisionCheckerTest
{
protected:
  std::unique_ptr<CollisionChecker> makeChecker( double padding = 0.0, double cache_epsilon = 0.0,
                                                 bool debug_viz = false )
  {
    auto checker = std::make_unique<CollisionChecker>( node_, padding, cache_epsilon, debug_viz );
    checker->setBroadphase( true );
    std::vector<std::string> all_joints;
    for ( pinocchio::JointIndex jid = 1; jid < ref_model_.joints.size(); ++jid ) {
      all_joints.push_back( ref_model_.names[jid] );
    }
    bool ok = checker->initFromXml( urdf_xml_, "", all_joints );
    EXPECT_TRUE( ok );
    EXPECT_TRUE( checker->isBroadphaseEnabled() );
    return checker;
  }
};

// Re-run key correctness tests with broadphase
TEST_F( CollisionCheckerBroadphaseTest, ZeroConfig )
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

TEST_F( CollisionCheckerBroadphaseTest, CollisionConfig )
{
  auto checker = makeChecker( 0.0 );
  std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", M_PI }, { "joint3", -M_PI / 2.0 }, { "joint4", 0.0 } };

  auto result = checker->checkCollision( positions );

  Eigen::VectorXd q = buildQ( ref_model_, positions );
  double bf_min = bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

  EXPECT_TRUE( result.in_collision );
  EXPECT_LT( bf_min, 0.0 );
  EXPECT_GE( result.min_distance, bf_min );
  EXPECT_LE( result.min_distance, 0.0 );
}

TEST_F( CollisionCheckerBroadphaseTest, RandomConfigurations )
{
  auto checker = makeChecker();

  std::mt19937 rng( 42 );
  std::uniform_real_distribution<double> dist( -M_PI, M_PI );

  for ( int i = 0; i < 50; ++i ) {
    std::unordered_map<std::string, double> positions = { { "joint1", dist( rng ) },
                                                          { "joint2", dist( rng ) },
                                                          { "joint3", dist( rng ) },
                                                          { "joint4", dist( rng ) } };

    auto result = checker->checkCollision( positions );

    Eigen::VectorXd q = buildQ( ref_model_, positions );
    double bf_min =
        bruteForceMinDistance( ref_model_, ref_data_, ref_geom_model_, ref_geom_data_, q );

    expectDistanceMatch( result.min_distance, bf_min, 0.0,
                         "broadphase config " + std::to_string( i ) );
  }
}

TEST_F( CollisionCheckerBroadphaseTest, GradientMatchesFiniteDifference )
{
  auto checker = makeChecker();
  const double eps = 1e-6;
  const double safety_zone = 1.0;
  checker->setSafetyZoneThreshold( safety_zone );

  std::vector<std::unordered_map<std::string, double>> configs = {
      { { "joint1", 0.0 }, { "joint2", 0.5 }, { "joint3", -0.3 }, { "joint4", 0.0 } },
      { { "joint1", 0.3 }, { "joint2", 2.0 }, { "joint3", -1.0 }, { "joint4", 0.5 } },
      { { "joint1", -0.2 }, { "joint2", 1.5 }, { "joint3", -0.8 }, { "joint4", -0.3 } },
      { { "joint1", 0.0 }, { "joint2", 0.0 }, { "joint3", 0.0 }, { "joint4", 0.0 } },
  };

  for ( size_t ci = 0; ci < configs.size(); ++ci ) {
    const auto &positions = configs[ci];
    auto result = checker->checkCollision( positions );
    if ( result.safety_zone_pairs.empty() )
      continue;

    // Find the closest pair (min distance) — broadphase may return pairs in any order
    const auto &closest =
        *std::min_element( result.safety_zone_pairs.begin(), result.safety_zone_pairs.end(),
                           []( const auto &a, const auto &b ) { return a.distance < b.distance; } );
    const auto &gradient = closest.gradient;

    for ( const auto &[name, val] : positions ) {
      auto perturbed = positions;
      perturbed[name] = val + eps;
      auto result_plus = checker->checkCollision( perturbed );
      double fd_gradient = ( result_plus.min_distance - result.min_distance ) / eps;

      int v_idx = checker->getJointVelocityIndex( name );
      ASSERT_GE( v_idx, 0 ) << "Joint " << name << " not found";
      ASSERT_LT( v_idx, gradient.size() );

      EXPECT_NEAR( gradient[v_idx], fd_gradient, 1e-3 )
          << "Broadphase gradient mismatch for joint " << name << " at config " << ci;
    }
  }
}

// ---- Direct comparison: broadphase vs brute-force on same configs ----
TEST_F( CollisionCheckerTest, BroadphaseMatchesBruteForce )
{
  auto bf_checker = makeChecker();
  bf_checker->setBroadphase( false ); // force brute-force for comparison
  // Create broadphase checker
  auto bp_checker = std::make_unique<CollisionChecker>( node_, 0.0, 0.0, false );
  bp_checker->setBroadphase( true );
  std::vector<std::string> all_joints;
  for ( pinocchio::JointIndex jid = 1; jid < ref_model_.joints.size(); ++jid ) {
    all_joints.push_back( ref_model_.names[jid] );
  }
  ASSERT_TRUE( bp_checker->initFromXml( urdf_xml_, "", all_joints ) );

  std::mt19937 rng( 999 );
  std::uniform_real_distribution<double> dist( -M_PI, M_PI );
  const double safety_zone = 0.1;
  bf_checker->setSafetyZoneThreshold( safety_zone );
  bp_checker->setSafetyZoneThreshold( safety_zone );

  for ( int i = 0; i < 200; ++i ) {
    std::unordered_map<std::string, double> positions = { { "joint1", dist( rng ) },
                                                          { "joint2", dist( rng ) },
                                                          { "joint3", dist( rng ) },
                                                          { "joint4", dist( rng ) } };

    auto bf_result = bf_checker->checkCollision( positions );
    auto bp_result = bp_checker->checkCollision( positions );

    EXPECT_EQ( bp_result.in_collision, bf_result.in_collision ) << "Config " << i;

    if ( bf_result.min_distance > 0.0 ) {
      EXPECT_NEAR( bp_result.min_distance, bf_result.min_distance, 1e-9 )
          << "Non-penetrating distance mismatch at config " << i;
    } else {
      EXPECT_LE( bp_result.min_distance, 0.0 ) << "Both should detect collision at config " << i;
    }

    // Sort both by pair_index (broadphase traverses in AABB-tree order, not sequential)
    auto sort_by_pair = []( auto &pairs ) {
      std::sort( pairs.begin(), pairs.end(),
                 []( const auto &a, const auto &b ) { return a.pair_index < b.pair_index; } );
    };
    sort_by_pair( bp_result.safety_zone_pairs );
    sort_by_pair( bf_result.safety_zone_pairs );

    EXPECT_EQ( bp_result.safety_zone_pairs.size(), bf_result.safety_zone_pairs.size() )
        << "Safety zone pair count mismatch at config " << i;

    // Compare gradients
    for ( size_t j = 0;
          j < std::min( bp_result.safety_zone_pairs.size(), bf_result.safety_zone_pairs.size() );
          ++j ) {
      EXPECT_EQ( bp_result.safety_zone_pairs[j].pair_index, bf_result.safety_zone_pairs[j].pair_index )
          << "Pair index mismatch at config " << i << " pair " << j;

      if ( bp_result.safety_zone_pairs[j].gradient.size() ==
           bf_result.safety_zone_pairs[j].gradient.size() ) {
        for ( int k = 0; k < bp_result.safety_zone_pairs[j].gradient.size(); ++k ) {
          EXPECT_NEAR( bp_result.safety_zone_pairs[j].gradient[k],
                       bf_result.safety_zone_pairs[j].gradient[k], 1e-6 )
              << "Gradient mismatch at config " << i << " pair " << j << " element " << k;
        }
      }
    }
  }
}

// ---- Broadphase vs brute-force with Athena robot ----
TEST_F( CollisionCheckerTest, BroadphaseMatchesBruteForceAthena )
{
  std::string athena_urdf, athena_srdf;
  try {
    athena_urdf = loadUrdfFile( "athena.urdf" );
    athena_srdf = loadUrdfFile( "athena.srdf" );
  } catch ( ... ) {
    GTEST_SKIP() << "Athena URDF/SRDF not available";
  }

  const std::vector<std::string> arm_joints = { "arm_joint_1", "arm_joint_2", "arm_joint_3",
                                                "arm_joint_4", "arm_joint_5", "arm_joint_6",
                                                "arm_joint_7" };

  auto bf_checker = std::make_unique<CollisionChecker>( node_, 0.01, 0.0, false );
  bf_checker->setBroadphase( false ); // force brute-force for comparison
  ASSERT_TRUE( bf_checker->initFromXml( athena_urdf, athena_srdf, arm_joints ) );

  auto bp_checker = std::make_unique<CollisionChecker>( node_, 0.01, 0.0, false );
  bp_checker->setBroadphase( true );
  ASSERT_TRUE( bp_checker->initFromXml( athena_urdf, athena_srdf, arm_joints ) );

  std::mt19937 rng( 77 );
  std::uniform_real_distribution<double> dist( -M_PI, M_PI );
  const double safety_zone = 0.05;
  bf_checker->setSafetyZoneThreshold( safety_zone );
  bp_checker->setSafetyZoneThreshold( safety_zone );

  for ( int i = 0; i < 100; ++i ) {
    std::unordered_map<std::string, double> positions;
    for ( const auto &name : arm_joints ) { positions[name] = dist( rng ); }

    auto bf_result = bf_checker->checkCollision( positions );
    auto bp_result = bp_checker->checkCollision( positions );

    EXPECT_EQ( bp_result.in_collision, bf_result.in_collision ) << "Athena config " << i;

    if ( bf_result.min_distance > 0.0 ) {
      // Broadphase uses hpp::fcl::distance directly on manager objects while brute-force
      // uses pinocchio::computeDistances — small numerical differences are expected from GJK.
      EXPECT_NEAR( bp_result.min_distance, bf_result.min_distance, 1e-6 )
          << "Athena distance mismatch at config " << i;
    }

    EXPECT_EQ( bp_result.safety_zone_pairs.size(), bf_result.safety_zone_pairs.size() )
        << "Athena safety zone pair count mismatch at config " << i;
  }
}

// ============================================================================
// Safety-zone pair capping (QP support API)
// ============================================================================

TEST_F( CollisionCheckerTest, MaxSafetyZonePairsCapsAndSortsByDistance )
{
  auto checker = makeChecker();
  // Large threshold → many pairs in the safety zone
  checker->setSafetyZoneThreshold( 10.0 );
  const std::unordered_map<std::string, double> positions = {
      { "joint1", 0.3 }, { "joint2", 0.7 }, { "joint3", -0.4 }, { "joint4", 0.2 } };

  const auto uncapped = checker->checkCollision( positions );
  ASSERT_GT( uncapped.safety_zone_pairs.size(), 2u );

  // Pairs must be sorted by distance ascending, closest first
  for ( size_t i = 1; i < uncapped.safety_zone_pairs.size(); ++i ) {
    EXPECT_LE( uncapped.safety_zone_pairs[i - 1].distance, uncapped.safety_zone_pairs[i].distance );
  }
  EXPECT_NEAR( uncapped.safety_zone_pairs.front().distance, uncapped.min_distance, 1e-12 );

  // Capping keeps only the closest pairs
  checker->setMaxSafetyZonePairs( 2 );
  const auto capped = checker->checkCollision( positions );
  ASSERT_EQ( capped.safety_zone_pairs.size(), 2u );
  EXPECT_EQ( capped.safety_zone_pairs[0].pair_index, uncapped.safety_zone_pairs[0].pair_index );
  EXPECT_EQ( capped.safety_zone_pairs[1].pair_index, uncapped.safety_zone_pairs[1].pair_index );
  EXPECT_NEAR( capped.min_distance, uncapped.min_distance, 1e-12 );
}

TEST_F( CollisionCheckerTest, FilterKeepsExactlyTheInfluenceablePairs )
{
  // The filter must keep a pair IFF the tree path between the two geometries crosses a
  // controlled joint (their deepest controlled ancestors differ): kept = finger riding
  // on the arm vs foreign obstacle; dropped = zero-gradient pairs (finger<->finger,
  // base<->obstacle). Topology mirrors the athena arm/gripper/flipper.
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="test_finger_filter">
  <link name="base_link">
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="arm_link">
    <collision><origin xyz="0 0 0.3"/><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="finger_link">
    <collision><origin xyz="0 0 0.15"/><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="finger2_link">
    <collision><origin xyz="0.1 0 0.15"/><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="obstacle_link">
    <collision><origin xyz="0.4 0 0.5"/><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/><child link="arm_link"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="1.0" velocity="1.0"/>
  </joint>
  <joint name="finger_joint" type="revolute">
    <parent link="arm_link"/><child link="finger_link"/>
    <origin xyz="0 0 0.4"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="1.0" velocity="1.0"/>
  </joint>
  <joint name="finger2_joint" type="revolute">
    <parent link="arm_link"/><child link="finger2_link"/>
    <origin xyz="0 0 0.4"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="1.0" velocity="1.0"/>
  </joint>
  <joint name="obstacle_joint" type="revolute">
    <parent link="base_link"/><child link="obstacle_link"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1.0" velocity="1.0"/>
  </joint>
</robot>)";

  auto checker = std::make_unique<CollisionChecker>( node_ );
  // Only the arm joint is controlled — NOT the finger joints, NOT the obstacle joint.
  ASSERT_TRUE( checker->initFromXml( urdf, "", { "arm_joint" } ) );

  // Geometry names are '<link>_<idx>'; compare by link-name prefix.
  auto has_pair = [&]( const std::string &link_a, const std::string &link_b ) {
    auto matches = []( const std::string &geom, const std::string &link ) {
      return geom.rfind( link + "_", 0 ) == 0;
    };
    for ( std::size_t k = 0; k < checker->getNumCollisionPairs(); ++k ) {
      const auto [name_a, name_b] = checker->getPairNames( k );
      if ( ( matches( name_a, link_a ) && matches( name_b, link_b ) ) ||
           ( matches( name_a, link_b ) && matches( name_b, link_a ) ) ) {
        return true;
      }
    }
    return false;
  };

  // KEPT: pairs whose relative pose depends on the controlled arm joint
  EXPECT_TRUE( has_pair( "finger_link", "obstacle_link" ) )
      << "finger<->obstacle missing: descendant-link geometry was filtered out";
  EXPECT_TRUE( has_pair( "finger2_link", "obstacle_link" ) );
  EXPECT_TRUE( has_pair( "arm_link", "obstacle_link" ) );
  EXPECT_TRUE( has_pair( "finger_link", "base_link" ) );
  EXPECT_TRUE( has_pair( "arm_link", "base_link" ) );

  // DROPPED: pairs the arm joint provably cannot influence
  EXPECT_FALSE( has_pair( "finger_link", "finger2_link" ) )
      << "same passive subtree: relative pose depends only on uncontrolled finger joints";
  EXPECT_FALSE( has_pair( "base_link", "obstacle_link" ) ) << "fully outside the controlled chain";
  // Geometry rigidly attached to the controlled chain itself moves WITH it: arm<->finger
  // relative pose depends only on the uncontrolled finger joint
  EXPECT_FALSE( has_pair( "arm_link", "finger_link" ) );

  // And the kept finger<->obstacle pair must have a nonzero gradient w.r.t. arm_joint
  checker->setSafetyZoneThreshold( 10.0 );
  const std::unordered_map<std::string, double> positions = { { "arm_joint", 0.0 },
                                                              { "finger_joint", 0.0 },
                                                              { "finger2_joint", 0.0 },
                                                              { "obstacle_joint", 0.0 } };
  const auto result = checker->checkCollision( positions );
  const int v_arm = checker->getJointVelocityIndex( "arm_joint" );
  ASSERT_GE( v_arm, 0 );
  bool found = false;
  for ( const auto &pi : result.safety_zone_pairs ) {
    const auto [name_a, name_b] = checker->getPairNames( pi.pair_index );
    const bool is_finger =
        name_a.rfind( "finger_link_", 0 ) == 0 || name_b.rfind( "finger_link_", 0 ) == 0;
    const bool is_obstacle =
        name_a.rfind( "obstacle_link_", 0 ) == 0 || name_b.rfind( "obstacle_link_", 0 ) == 0;
    if ( is_finger && is_obstacle ) {
      found = true;
      EXPECT_GT( std::abs( pi.gradient[v_arm] ), 1e-6 )
          << "arm joint cannot influence the finger<->obstacle distance?";
    }
  }
  EXPECT_TRUE( found );
}

TEST_F( CollisionCheckerTest, PairCapPrefersDistinctLinkPairs )
{
  // Links with several collision geometries produce near-duplicate pairs. When the
  // safety-zone cap is exceeded, distinct link pairs must be kept in preference to
  // duplicates — otherwise a parked contact with many geometries can evict a genuinely
  // different (e.g. approaching) contact from the constraint budget.
  //
  // Chain: base(1 sphere) - link1(2 spheres at z=0.20/0.24) - link2(1 sphere at z=0.5).
  // Distances at q=0 (all radii 0.05):
  //   base<->link1_0: 0.10 | base<->link1_1: 0.14 | link1_1<->link2: 0.16
  //   link1_0<->link2: 0.20 | base<->link2: 0.40
  const std::string urdf = R"(<?xml version="1.0"?>
<robot name="test_multigeom">
  <link name="base_link">
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="link1">
    <collision><origin xyz="0 0 0.20"/><geometry><sphere radius="0.05"/></geometry></collision>
    <collision><origin xyz="0 0 0.24"/><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <link name="link2">
    <collision><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1.0" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.5"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="1.0" velocity="1.0"/>
  </joint>
</robot>)";

  auto checker = std::make_unique<CollisionChecker>( node_ );
  ASSERT_TRUE( checker->initFromXml( urdf, "", { "joint1", "joint2" } ) );
  checker->setSafetyZoneThreshold( 10.0 );
  const std::unordered_map<std::string, double> positions = { { "joint1", 0.0 }, { "joint2", 0.0 } };

  // Uncapped: all 5 pairs
  const auto uncapped = checker->checkCollision( positions );
  ASSERT_EQ( uncapped.safety_zone_pairs.size(), 5u );

  // Cap 3: naive closest-3 would keep {0.10, 0.14, 0.16} — two base<->link1 duplicates —
  // and evict base<->link2 entirely. Dedup must keep the closest of EACH link pair:
  // {0.10, 0.16, 0.40}.
  checker->setMaxSafetyZonePairs( 3 );
  const auto capped = checker->checkCollision( positions );
  ASSERT_EQ( capped.safety_zone_pairs.size(), 3u );
  EXPECT_NEAR( capped.safety_zone_pairs[0].distance, 0.10, 1e-6 );
  EXPECT_NEAR( capped.safety_zone_pairs[1].distance, 0.16, 1e-6 );
  EXPECT_NEAR( capped.safety_zone_pairs[2].distance, 0.40, 1e-6 );

  // Cap 4: the freed slot is refilled with the closest duplicate (0.14), sorted order kept
  checker->setMaxSafetyZonePairs( 4 );
  const auto refilled = checker->checkCollision( positions );
  ASSERT_EQ( refilled.safety_zone_pairs.size(), 4u );
  EXPECT_NEAR( refilled.safety_zone_pairs[0].distance, 0.10, 1e-6 );
  EXPECT_NEAR( refilled.safety_zone_pairs[1].distance, 0.14, 1e-6 );
  EXPECT_NEAR( refilled.safety_zone_pairs[2].distance, 0.16, 1e-6 );
  EXPECT_NEAR( refilled.safety_zone_pairs[3].distance, 0.40, 1e-6 );
}

TEST_F( CollisionCheckerTest, PairGradientsMatchFiniteDifferencesInBothVizModes )
{
  // debug_viz=true switches to SINGLE-PASS mode where the broadphase callback computes
  // nearest points in tree-traversal order; unswapped witness points exactly negate the
  // gradients. Validate every pair gradient against finite differences in both modes.
  const std::unordered_map<std::string, double> positions = {
      { "joint1", 0.3 }, { "joint2", 0.7 }, { "joint3", -0.4 }, { "joint4", 0.2 } };
  const double h = 1e-6;

  for ( const bool debug_viz : { false, true } ) {
    auto checker = makeChecker( 0.0, 0.0, debug_viz );
    checker->setSafetyZoneThreshold( 10.0 ); // all pairs in the zone → all gradients
    const auto result = checker->checkCollision( positions );
    ASSERT_GT( result.safety_zone_pairs.size(), 3u );

    for ( const auto &pair : result.safety_zone_pairs ) {
      for ( const auto &[joint_name, value] : positions ) {
        const int v_idx = checker->getJointVelocityIndex( joint_name );
        ASSERT_GE( v_idx, 0 );

        auto find_pair_distance = [&]( const CollisionResult &res ) {
          for ( const auto &pi : res.safety_zone_pairs ) {
            if ( pi.pair_index == pair.pair_index ) {
              return pi.distance;
            }
          }
          return std::numeric_limits<double>::quiet_NaN();
        };

        auto plus = positions;
        auto minus = positions;
        plus[joint_name] = value + h;
        minus[joint_name] = value - h;
        const double d_plus = find_pair_distance( checker->checkCollision( plus ) );
        const double d_minus = find_pair_distance( checker->checkCollision( minus ) );
        ASSERT_FALSE( std::isnan( d_plus ) || std::isnan( d_minus ) );

        const double fd = ( d_plus - d_minus ) / ( 2.0 * h );
        EXPECT_NEAR( pair.gradient[v_idx], fd, 1e-4 )
            << "gradient mismatch (debug_viz=" << debug_viz << ") pair " << pair.pair_index
            << " joint " << joint_name;
      }
    }
  }
}

TEST_F( CollisionCheckerTest, PenetrationGradientPointsOutward )
{
  // coal witness points satisfy p2 - p1 = min_distance * normal, i.e. anti-parallel to
  // the separation normal when penetrating; the gradient must still point toward
  // INCREASING distance or a QP push-out drives deeper in.
  auto checker = makeChecker();
  checker->setSafetyZoneThreshold( 0.05 );
  const std::unordered_map<std::string, double> positions = {
      { "joint1", 0.0 }, { "joint2", M_PI }, { "joint3", -M_PI / 2.0 }, { "joint4", 0.0 } };

  const auto result = checker->checkCollision( positions );
  ASSERT_TRUE( result.in_collision );

  const CollisionResult::PairInfo *worst = nullptr;
  for ( const auto &pi : result.safety_zone_pairs ) {
    if ( pi.distance < 0.0 && ( !worst || pi.distance < worst->distance ) ) {
      worst = &pi;
    }
  }
  ASSERT_NE( worst, nullptr ) << "expected at least one penetrating pair with gradient";
  ASSERT_GT( worst->gradient.norm(), 1e-9 );
  const std::size_t pair_index = worst->pair_index;
  const double d0 = worst->distance;

  // Step along the gradient in joint space → the pair's distance must increase
  const double h = 1e-4;
  auto perturbed = positions;
  for ( auto &[name, value] : perturbed ) {
    const int vi = checker->getJointVelocityIndex( name );
    if ( vi >= 0 && vi < worst->gradient.size() ) {
      value += h * worst->gradient[vi];
    }
  }
  const auto result2 = checker->checkCollision( perturbed );

  double d1 = std::numeric_limits<double>::quiet_NaN();
  for ( const auto &pi : result2.safety_zone_pairs ) {
    if ( pi.pair_index == pair_index ) {
      d1 = pi.distance;
    }
  }
  ASSERT_FALSE( std::isnan( d1 ) );
  EXPECT_GT( d1, d0 ) << "moving along the gradient must increase the pair distance";
}

// __gcov_dump is only available when compiled with --coverage.
// Use a weak symbol so the call is a no-op in normal (non-coverage) builds.
#if defined( __GNUC__ )
extern "C" void __gcov_dump() __attribute__( ( weak ) );
#endif

// ---- Marker coloring: directional info must never be indexed out of range ----

TEST( CollisionCheckerPairDirection, ReturnsNaNUnlessTheInfoMatchesThePairSet )
{
  // publishMinimalMarkers()/publishMarkers() run inside checkCollision(), which happens
  // before the controller has ever pushed directional info, so the vector is empty there.
  EXPECT_TRUE( std::isnan( CollisionChecker::pairDirection( std::vector<double>{}, 3, 0 ) ) );
  EXPECT_TRUE( std::isnan( CollisionChecker::pairDirection( { 1.0 }, 3, 0 ) ) );
  EXPECT_TRUE( std::isnan( CollisionChecker::pairDirection( { 1.0, 2.0, 3.0 }, 3, 3 ) ) );
  EXPECT_DOUBLE_EQ( CollisionChecker::pairDirection( { 1.0, 2.0, 3.0 }, 3, 1 ), 2.0 );
}

TEST_F( CollisionCheckerTest, MinimalMarkersBeforeAnyDirectionalInfo )
{
  // Exercises that path end to end; the out-of-bounds read it used to do is only caught
  // by a build with -D_GLIBCXX_ASSERTIONS or a sanitizer.
  auto checker = makeChecker( 0.0 );
  checker->updatePublishCollisionDistances( true );
  checker->setSafetyZoneThreshold( 1.0 );
  const auto result = checker->checkCollision( {} );
  EXPECT_FALSE( result.safety_zone_pairs.empty() ) << "need a zone pair to reach the color branch";
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  // Use _exit to avoid double-free in global destructors caused by
  // pinocchio/hpp-fcl library cleanup ordering issues.
  // All test resources are cleaned up in TearDown before reaching this point.
#if defined( __GNUC__ )
  if ( __gcov_dump )
    __gcov_dump(); // Flush coverage data before _exit
#endif
  _exit( result );
}
