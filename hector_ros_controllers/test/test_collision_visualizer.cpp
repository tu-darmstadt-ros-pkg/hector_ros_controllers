// Unit tests for CollisionVisualizer: marker publishing must never index the
// directional info out of range, and must survive being called before the controller
// has produced any.

#include <unistd.h>

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "safety_position_controller/collision_visualizer.hpp"

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

// __gcov_dump is only available when compiled with --coverage; weak so the call is a
// no-op in normal builds.
#if defined( __GNUC__ )
extern "C" void __gcov_dump() __attribute__( ( weak ) );
#endif

namespace spc = safety_position_controller;

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
} // namespace

class CollisionVisualizerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>( "test_collision_visualizer",
                                                               rclcpp::NodeOptions() );
    checker_ = std::make_unique<CollisionChecker>( node_, 0.0, 0.0 );
    ASSERT_TRUE( checker_->initFromXml( loadUrdfFile( "test_robot_collision.urdf" ), "", {} ) );
    checker_->setSafetyZoneThreshold( 1.0 ); // wide: every pair lands in the zone
    visualizer_ = std::make_unique<spc::CollisionVisualizer>( node_ );
  }

  void TearDown() override
  {
    visualizer_.reset();
    checker_.reset();
    node_.reset();
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::unique_ptr<CollisionChecker> checker_;
  std::unique_ptr<spc::CollisionVisualizer> visualizer_;
};

TEST_F( CollisionVisualizerTest, PublishesWithoutDirectionalInfo )
{
  // The first cycle publishes before any directional info exists; an out-of-range read
  // here is only caught by -D_GLIBCXX_ASSERTIONS or a sanitizer.
  const auto &result = checker_->checkCollision( {} );
  ASSERT_FALSE( result.safety_zone_pairs.empty() ) << "need a zone pair to reach the color branch";

  visualizer_->publish( *checker_, spc::CollisionVisualizer::Level::LinesOnly, {}, 1.0 );
  visualizer_->publish( *checker_, spc::CollisionVisualizer::Level::FullGeometry, {}, 1.0 );
}

TEST_F( CollisionVisualizerTest, PublishesWithMismatchedAndMatchingDirectionalInfo )
{
  checker_->checkCollision( {} );
  const std::size_t num_pairs = checker_->getNumCollisionPairs();
  ASSERT_GT( num_pairs, 0u );

  // A vector sized for a different pair set must be ignored, not indexed.
  visualizer_->publish( *checker_, spc::CollisionVisualizer::Level::LinesOnly,
                        std::vector<double>( num_pairs - 1, 1.0 ), 1.0 );
  // The matching size colors the lines.
  visualizer_->publish( *checker_, spc::CollisionVisualizer::Level::FullGeometry,
                        std::vector<double>( num_pairs, -1.0 ), 1.0 );
}

TEST_F( CollisionVisualizerTest, PublishesBeforeAnyCheckHasRun )
{
  // No collision query yet: the latched result is empty and every distance stale.
  visualizer_->publish( *checker_, spc::CollisionVisualizer::Level::FullGeometry, {}, 0.05 );
}

// ---------------------------------------------------------------------------
// Shape markers
// ---------------------------------------------------------------------------
//
// pinocchio does not leave meshPath empty for primitives: it stores the shape's NAME
// ("BOX", "CYLINDER", "SPHERE", "CAPSULE") there with a meshScale of (1, 1, 1). Any
// code path that reaches for meshPath as a mesh resource therefore asks RViz for a
// 1 m unit box instead of the real geometry. These tests pin that shut.

namespace
{
using Marker = visualization_msgs::msg::Marker;

/// A geometry object shaped like the ones pinocchio's URDF parser produces: the given
/// geometry, plus the primitive-name placeholder in meshPath with a unit meshScale.
pinocchio::GeometryObject makeObject( std::shared_ptr<coal::CollisionGeometry> geometry,
                                      const std::string &mesh_path )
{
  pinocchio::GeometryObject go( "obj", 0, 0, pinocchio::SE3::Identity(), std::move( geometry ) );
  go.meshPath = mesh_path;
  go.meshScale = Eigen::Vector3d::Ones();
  return go;
}

Marker describe( const pinocchio::GeometryObject &go )
{
  Marker m;
  m.pose.orientation.w = 1.0;
  spc::CollisionVisualizer::describeShape( go, m );
  return m;
}

::testing::AssertionResult isUnitBox( const Marker &m )
{
  if ( m.type == Marker::CUBE && m.scale.x == 1.0 && m.scale.y == 1.0 && m.scale.z == 1.0 ) {
    return ::testing::AssertionSuccess() << "unit CUBE";
  }
  if ( m.type == Marker::MESH_RESOURCE && m.mesh_resource.find( '/' ) == std::string::npos ) {
    return ::testing::AssertionSuccess()
           << "mesh resource '" << m.mesh_resource << "' at scale " << m.scale.x;
  }
  return ::testing::AssertionFailure();
}
} // namespace

TEST_F( CollisionVisualizerTest, PrimitivesKeepTheirRealShapeAndSize )
{
  const auto &geom_model = checker_->geometryModel();
  ASSERT_FALSE( geom_model.geometryObjects.empty() );

  for ( const auto &go : geom_model.geometryObjects ) {
    const Marker m = describe( go );
    EXPECT_FALSE( isUnitBox( m ) ) << "object '" << go.name << "' fell back to a unit box";
    ASSERT_EQ( m.type, Marker::SPHERE ) << go.name; // the test URDF is all spheres
    const auto &sphere = static_cast<const coal::Sphere &>( *go.geometry );
    EXPECT_DOUBLE_EQ( m.scale.x, 2.0 * sphere.radius );
    EXPECT_DOUBLE_EQ( m.scale.y, 2.0 * sphere.radius );
    EXPECT_DOUBLE_EQ( m.scale.z, 2.0 * sphere.radius );
  }
}

TEST_F( CollisionVisualizerTest, BoxAndCylinderAreSizedFromTheGeometryNotMeshScale )
{
  const Marker box = describe( makeObject( std::make_shared<coal::Box>( 0.2, 0.4, 0.6 ), "BOX" ) );
  EXPECT_EQ( box.type, Marker::CUBE );
  EXPECT_DOUBLE_EQ( box.scale.x, 0.2 );
  EXPECT_DOUBLE_EQ( box.scale.y, 0.4 );
  EXPECT_DOUBLE_EQ( box.scale.z, 0.6 );
  EXPECT_TRUE( box.mesh_resource.empty() );

  const Marker cyl =
      describe( makeObject( std::make_shared<coal::Cylinder>( 0.05, 0.3 ), "CYLINDER" ) );
  EXPECT_EQ( cyl.type, Marker::CYLINDER );
  EXPECT_DOUBLE_EQ( cyl.scale.x, 0.1 );
  EXPECT_DOUBLE_EQ( cyl.scale.z, 0.3 );
  EXPECT_TRUE( cyl.mesh_resource.empty() );
}

TEST_F( CollisionVisualizerTest, CapsuleIsDrawnAsItsEnclosingCylinder )
{
  // No capsule marker type exists; the enclosing cylinder over-covers, so nothing ever
  // looks smaller than the geometry the checker uses.
  const Marker m =
      describe( makeObject( std::make_shared<coal::Capsule>( 0.05, 0.3 ), "CAPSULE" ) );
  EXPECT_EQ( m.type, Marker::CYLINDER );
  EXPECT_DOUBLE_EQ( m.scale.x, 0.1 );
  EXPECT_DOUBLE_EQ( m.scale.z, 0.3 + 2.0 * 0.05 );
  EXPECT_TRUE( m.mesh_resource.empty() );
}

TEST_F( CollisionVisualizerTest, UnrepresentableShapeFallsBackToItsBoundingBoxNotAUnitBox )
{
  // A cone has no marker type. The old fallback took meshPath ("BOX") as a mesh
  // resource and meshScale (1,1,1) as its size — a 1 m box in place of the geometry.
  auto cone = std::make_shared<coal::Cone>( 0.1, 0.4 );
  cone->computeLocalAABB();
  const Marker m = describe( makeObject( cone, "CONE" ) );

  EXPECT_FALSE( isUnitBox( m ) );
  EXPECT_TRUE( m.mesh_resource.empty() ) << "a primitive name is not a mesh resource";
  EXPECT_EQ( m.type, Marker::CUBE );
  EXPECT_NEAR( m.scale.x, 0.2, 1e-9 );
  EXPECT_NEAR( m.scale.y, 0.2, 1e-9 );
  EXPECT_NEAR( m.scale.z, 0.4, 1e-9 );
}

TEST_F( CollisionVisualizerTest, RealMeshPathBecomesAFileUri )
{
  // pinocchio resolves package:// to an absolute filesystem path, which
  // resource_retriever cannot fetch without a scheme.
  auto mesh = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
  const Marker m = describe( makeObject( mesh, "/opt/ros/share/pkg/meshes/link.stl" ) );

  EXPECT_EQ( m.type, Marker::MESH_RESOURCE );
  EXPECT_EQ( m.mesh_resource, "file:///opt/ros/share/pkg/meshes/link.stl" );
}

TEST_F( CollisionVisualizerTest, MeshUriIsPassedThroughUnchanged )
{
  auto mesh = std::make_shared<coal::BVHModel<coal::OBBRSS>>();
  const Marker m = describe( makeObject( mesh, "package://pkg/meshes/link.stl" ) );

  EXPECT_EQ( m.type, Marker::MESH_RESOURCE );
  EXPECT_EQ( m.mesh_resource, "package://pkg/meshes/link.stl" );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  rclcpp::init( argc, argv );
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  // Same as test_collision_checker: pinocchio/coal global destructors corrupt the heap
  // on the way out. Everything this binary owns is released in TearDown.
#if defined( __GNUC__ )
  if ( __gcov_dump )
    __gcov_dump(); // flush coverage data before _exit
#endif
  _exit( result );
}
