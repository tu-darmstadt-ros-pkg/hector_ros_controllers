//
// Benchmark for SafetyQpLimiter: solve time at realistic problem sizes.
// The QP runs inside the controller update loop (typically 100 Hz), so the
// solve must stay well under ~100 µs.
//
#include <benchmark/benchmark.h>

#include <random>

#include "safety_position_controller/safety_qp_limiter.hpp"

using safety_position_controller::QpCollisionConstraint;
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
  for ( int k = 0; k < num_collisions; ++k ) {
    QpCollisionConstraint c;
    c.normal = Eigen::VectorXd::NullaryExpr( n, [&]() { return 0.2 * unit( rng ); } );
    c.distance = 0.015 + 0.03 * std::abs( unit( rng ) ); // inside the safety zone
    in.collisions.push_back( std::move( c ) );
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

} // namespace

// 7-DoF arm: free space, few obstacles, many obstacles
BENCHMARK( BM_SolveWarmStarted )->Args( { 7, 0 } )->Args( { 7, 3 } )->Args( { 7, 10 } );
// Small and large joint groups
BENCHMARK( BM_SolveWarmStarted )->Args( { 3, 3 } )->Args( { 12, 10 } );

BENCHMARK_MAIN();
