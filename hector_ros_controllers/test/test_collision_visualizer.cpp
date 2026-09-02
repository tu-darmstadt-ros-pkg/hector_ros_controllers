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
    checker_ = std::make_unique<CollisionChecker>( node_, 0.0, 0.0, true );
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
