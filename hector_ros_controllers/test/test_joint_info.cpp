// Pure unit tests for parse_joint_infos(): no ROS runtime, no controller fixture.

#include <gmock/gmock.h>

#include <safety_position_controller/joint_info.hpp>

namespace spc = safety_position_controller;

namespace
{
constexpr double kDefaultVelocity = 1.5;

// One chain covering every case the parser has to distinguish. urdfdom requires
// <limit> on revolute/prismatic joints, so "bounded joint without limits" is not
// representable and is not tested.
const char *kUrdf = R"(<robot name="test">
  <link name="base"/>
  <link name="l1"/>
  <link name="l2"/>
  <link name="l3"/>
  <link name="l4"/>
  <link name="l5"/>
  <link name="l6"/>
  <joint name="cont_no_limit" type="continuous">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="cont_with_limit" type="continuous">
    <parent link="l1"/><child link="l2"/><axis xyz="0 0 1"/>
    <limit effort="10" velocity="2.5"/>
  </joint>
  <joint name="rev" type="revolute">
    <parent link="l2"/><child link="l3"/><axis xyz="0 0 1"/>
    <limit effort="10" velocity="1.0" lower="-1.0" upper="1.0"/>
  </joint>
  <joint name="rev_zero_velocity" type="revolute">
    <parent link="l3"/><child link="l4"/><axis xyz="0 0 1"/>
    <limit effort="10" velocity="0.0" lower="-1.0" upper="1.0"/>
  </joint>
  <joint name="rev_invalid_range" type="revolute">
    <parent link="l4"/><child link="l5"/><axis xyz="0 0 1"/>
    <limit effort="10" velocity="1.0" lower="1.0" upper="-1.0"/>
  </joint>
  <joint name="fixed_joint" type="fixed">
    <parent link="l5"/><child link="l6"/>
  </joint>
</robot>)";

spc::JointInfoParseResult parse( const std::vector<std::string> &joints )
{
  return spc::parse_joint_infos( kUrdf, joints, kDefaultVelocity );
}

bool warnsAbout( const spc::JointInfoParseResult &result, const std::string &needle )
{
  for ( const auto &warning : result.warnings ) {
    if ( warning.find( needle ) != std::string::npos ) {
      return true;
    }
  }
  return false;
}
} // namespace

TEST( ParseJointInfos, ContinuousWithoutVelocityLimitFallsBackToDefault )
{
  // A URDF velocity limit is optional on continuous joints; without the fallback the
  // joint would be stepped unbounded.
  const auto result = parse( { "cont_no_limit" } );
  ASSERT_TRUE( result.ok );
  EXPECT_EQ( result.joints[0].type, spc::JointType::CONTINUOUS );
  EXPECT_FALSE( result.joints[0].has_position_limits );
  EXPECT_DOUBLE_EQ( result.joints[0].velocity_limit, kDefaultVelocity );
  EXPECT_TRUE( warnsAbout( result, "cont_no_limit" ) );
}

TEST( ParseJointInfos, VelocityLimitFromUrdfIsKept )
{
  const auto result = parse( { "cont_with_limit", "rev" } );
  ASSERT_TRUE( result.ok );
  EXPECT_DOUBLE_EQ( result.joints[0].velocity_limit, 2.5 );
  EXPECT_DOUBLE_EQ( result.joints[1].velocity_limit, 1.0 );
  EXPECT_TRUE( result.warnings.empty() );
}

TEST( ParseJointInfos, NonPositiveVelocityLimitFallsBackToDefault )
{
  const auto result = parse( { "rev_zero_velocity" } );
  ASSERT_TRUE( result.ok );
  EXPECT_DOUBLE_EQ( result.joints[0].velocity_limit, kDefaultVelocity );
  EXPECT_TRUE( warnsAbout( result, "rev_zero_velocity" ) );
}

TEST( ParseJointInfos, BoundedJointKeepsPositionLimits )
{
  const auto result = parse( { "rev" } );
  ASSERT_TRUE( result.ok );
  EXPECT_EQ( result.joints[0].type, spc::JointType::REVOLUTE_BOUNDED );
  EXPECT_TRUE( result.joints[0].has_position_limits );
  EXPECT_DOUBLE_EQ( result.joints[0].lower_limit, -1.0 );
  EXPECT_DOUBLE_EQ( result.joints[0].upper_limit, 1.0 );
}

TEST( ParseJointInfos, InvalidPositionLimitsAreDropped )
{
  const auto result = parse( { "rev_invalid_range" } );
  ASSERT_TRUE( result.ok );
  EXPECT_FALSE( result.joints[0].has_position_limits );
  EXPECT_DOUBLE_EQ( result.joints[0].velocity_limit, 1.0 ) << "velocity limit stays usable";
  EXPECT_TRUE( warnsAbout( result, "rev_invalid_range" ) );
}

TEST( ParseJointInfos, UnknownJointFallsBackToDefaultVelocity )
{
  const auto result = parse( { "not_in_urdf" } );
  ASSERT_TRUE( result.ok );
  EXPECT_EQ( result.joints[0].type, spc::JointType::OTHER );
  EXPECT_DOUBLE_EQ( result.joints[0].velocity_limit, kDefaultVelocity );
  EXPECT_TRUE( warnsAbout( result, "not_in_urdf" ) );
}

TEST( ParseJointInfos, AllJointNamesExcludeFixedJoints )
{
  const auto result = parse( {} );
  ASSERT_TRUE( result.ok );
  EXPECT_THAT( result.all_joint_names,
               ::testing::UnorderedElementsAre( "cont_no_limit", "cont_with_limit", "rev",
                                                "rev_zero_velocity", "rev_invalid_range" ) );
}

TEST( ParseJointInfos, UnparsableUrdfFails )
{
  const auto result = spc::parse_joint_infos( "not xml", { "rev" }, kDefaultVelocity );
  EXPECT_FALSE( result.ok );
}

int main( int argc, char **argv )
{
  ::testing::InitGoogleMock( &argc, argv );
  return RUN_ALL_TESTS();
}
