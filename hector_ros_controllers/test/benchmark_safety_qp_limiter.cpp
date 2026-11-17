//
// Benchmark for SafetyQpLimiter: solve time at realistic problem sizes.
// The QP runs inside the controller update loop (typically 100 Hz), so the
// solve must stay well under ~100 µs.
//
#include <benchmark/benchmark.h>

#include <random>

#include "safety_position_controller/safety_qp_limiter.hpp"

using safety_position_controller::QpCollisionConstraints;
using safety_position_controller::SafetyQpInput;
using safety_position_controller::SafetyQpLimiter;
using safety_position_controller::SafetyQpParams;

namespace
{

SafetyQpParams makeParams( const int n )
{
  SafetyQpParams p;
  p.dt = 0.01;
  p.v_max = Eigen::VectorXd::Constant( n, 1.5 );
  p.a_acc = Eigen::VectorXd::Constant( n, 8.0 );
  p.a_dec = Eigen::VectorXd::Constant( n, 25.0 );
  p.max_collision_constraints = 10;
  return p;
}

/// Deterministic pseudo-random input resembling a mid-motion cycle near obstacles.
SafetyQpInput makeInput( const int n, const int num_collisions, std::mt19937 &rng )
{
  std::uniform_real_distribution<double> unit( -1.0, 1.0 );
  SafetyQpInput in;
  in.v_des = Eigen::VectorXd::NullaryExpr( n, [&]() { return unit( rng ); } );
  in.v_prev = 0.8 * in.v_des;
  in.q = Eigen::VectorXd::NullaryExpr( n, [&]() { return unit( rng ); } );
  in.q_lo = Eigen::VectorXd::Constant( n, -2.5 );
  in.q_hi = Eigen::VectorXd::Constant( n, 2.5 );
  in.collisions.reserve( num_collisions, n );
  for ( int k = 0; k < num_collisions; ++k ) {
    const Eigen::VectorXd normal =
        Eigen::VectorXd::NullaryExpr( n, [&]() { return 0.2 * unit( rng ); } );
    in.collisions.push( normal, 0.015 + 0.03 * std::abs( unit( rng ) ) ); // inside the safety zone
  }
  return in;
}

void BM_SolveWarmStarted( benchmark::State &state )
{
  const int n = static_cast<int>( state.range( 0 ) );
  const int num_collisions = static_cast<int>( state.range( 1 ) );
  std::mt19937 rng( 42 );

  SafetyQpLimiter limiter( static_cast<std::size_t>( n ), makeParams( n ) );
  auto in = makeInput( n, num_collisions, rng );
  limiter.solve( in ); // first solve = cold init, excluded from the loop

  for ( auto _ : state ) {
    // Perturb slightly so warm starting is realistic, not identical resolves
    in.v_des[0] += 1e-4;
    auto result = limiter.solve( in );
    benchmark::DoNotOptimize( result );
  }
}

/// Athena's limb configuration (athena_simulation_controllers.yaml): 64 collision rows,
/// deceleration = 3x acceleration, 3 cm padding inside a 10-12 cm safety zone.
SafetyQpParams makeAthenaParams( const int n, const double a_acc, const double d_zone )
{
  SafetyQpParams p;
  p.dt = 0.02;
  p.v_max = Eigen::VectorXd::Constant( n, 1.0 );
  p.a_acc = Eigen::VectorXd::Constant( n, a_acc );
  p.a_dec = 3.0 * p.a_acc;
  p.d_pad = 0.03;
  p.d_zone = d_zone;
  p.contact_crawl_speed = 0.15;
  p.max_collision_constraints = 64;
  return p;
}

/// Cycles through distinct problems: every control cycle sees new bounds and normals.
void solveSequence( benchmark::State &state, const int n, const SafetyQpParams &params )
{
  const int num_collisions = static_cast<int>( state.range( 0 ) );
  std::mt19937 rng( 7 );
  std::uniform_real_distribution<double> unit( -1.0, 1.0 );

  std::vector<SafetyQpInput> inputs;
  for ( int i = 0; i < 64; ++i ) {
    SafetyQpInput in = makeInput( n, num_collisions, rng );
    QpCollisionConstraints spread;
    spread.reserve( num_collisions, n );
    for ( Eigen::Index k = 0; k < in.collisions.size(); ++k ) {
      spread.push( in.collisions.normal( k ).transpose(),
                   params.d_pad + ( params.d_zone - params.d_pad ) * std::abs( unit( rng ) ) );
    }
    in.collisions = spread;
    inputs.push_back( std::move( in ) );
  }

  SafetyQpLimiter limiter( static_cast<std::size_t>( n ), params );
  limiter.solve( inputs.front() );

  std::size_t k = 0;
  long iterations = 0;
  for ( auto _ : state ) {
    const auto &result = limiter.solve( inputs[k++ % inputs.size()] );
    iterations += result.iterations;
    benchmark::DoNotOptimize( result.v.data() );
  }
  state.counters["qp_iter"] =
      benchmark::Counter( static_cast<double>( iterations ), benchmark::Counter::kAvgIterations );
}

void BM_AthenaArm( benchmark::State &state )
{ solveSequence( state, 7, makeAthenaParams( 7, 5.28, 0.12 ) ); }

void BM_AthenaFlipper( benchmark::State &state )
{ solveSequence( state, 4, makeAthenaParams( 4, 8.0, 0.1 ) ); }

} // namespace

// 7-DoF arm: free space, few obstacles, many obstacles
BENCHMARK( BM_SolveWarmStarted )->Args( { 7, 0 } )->Args( { 7, 3 } )->Args( { 7, 10 } );
// Small and large joint groups
BENCHMARK( BM_SolveWarmStarted )->Args( { 3, 3 } )->Args( { 12, 10 } );

// Athena's arm and flipper safety controllers: free space up to many close pairs
BENCHMARK( BM_AthenaArm )->Arg( 0 )->Arg( 4 )->Arg( 16 )->Arg( 64 );
BENCHMARK( BM_AthenaFlipper )->Arg( 0 )->Arg( 4 )->Arg( 16 );

BENCHMARK_MAIN();
