#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>

namespace hector_test
{

/// Test access to one handle's value, readable and assignable like a double.
class HandleValue
{
public:
  explicit HandleValue( hardware_interface::Handle &handle ) : handle_( handle ) { }

  operator double() const
  {
    const auto value = handle_.get_optional();
    if ( !value ) {
      throw std::runtime_error( "test could not read handle '" + handle_.get_name() + "'" );
    }
    return *value;
  }

  HandleValue &operator=( double value )
  {
    if ( !handle_.set_value( value, true ) ) {
      throw std::runtime_error( "test could not write handle '" + handle_.get_name() + "'" );
    }
    return *this;
  }

  HandleValue &operator=( const HandleValue &other )
  { return *this = static_cast<double>( other ); }

private:
  hardware_interface::Handle &handle_;
};

/// The values behind the hardware interfaces a test loans to a controller. Each slot holds a
/// plain number until make_command or make_state turns it into a handle that owns the value.
class HandleValues
{
public:
  void assign( std::size_t size, double value )
  {
    initial_.assign( size, value );
    handles_.assign( size, nullptr );
  }

  std::size_t size() const { return initial_.size(); }

  void fill( double value )
  {
    for ( std::size_t i = 0; i < size(); ++i ) { ( *this )[i] = value; }
  }

  std::shared_ptr<hardware_interface::CommandInterface>
  make_command( std::size_t index, const std::string &prefix, const std::string &name )
  { return make<hardware_interface::CommandInterface>( index, prefix, name ); }

  std::shared_ptr<hardware_interface::StateInterface>
  make_state( std::size_t index, const std::string &prefix, const std::string &name )
  { return make<hardware_interface::StateInterface>( index, prefix, name ); }

  HandleValue operator[]( std::size_t index ) const
  {
    if ( !handles_.at( index ) ) {
      throw std::runtime_error( "test reads slot " + std::to_string( index ) +
                                " before it was made into a handle" );
    }
    return HandleValue( *handles_[index] );
  }

private:
  template<typename Interface>
  std::shared_ptr<Interface> make( std::size_t index, const std::string &prefix,
                                   const std::string &name )
  {
    auto handle = std::make_shared<Interface>( prefix, name );
    if ( !handle->set_value( initial_.at( index ), true ) ) {
      throw std::runtime_error( "test could not initialise handle '" + handle->get_name() + "'" );
    }
    handles_[index] = handle;
    return handle;
  }

  std::vector<double> initial_;
  std::vector<std::shared_ptr<hardware_interface::Handle>> handles_;
};

/// The reference interface @p index a chainable controller exported.
template<typename Controller>
HandleValue reference( Controller &controller, std::size_t index )
{ return HandleValue( *controller.ordered_exported_reference_interfaces_.at( index ) ); }

} // namespace hector_test
