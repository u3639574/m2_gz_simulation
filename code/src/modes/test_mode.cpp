#include "m2_gz_simulation/modes/test_mode.h"

#include "trolly/log/trolly_logger_macro.h"

namespace m2::gz_simulation {

test_mode::test_mode() noexcept
    : fsm::input::ps_mode("Test Mode")
{
}

void test_mode::on_enter() noexcept
{
    TROLLY_INFO("[test_mode] Enter.");
}

void test_mode::on_exit() noexcept
{
    TROLLY_INFO("[test_mode] Exit.");
}

void test_mode::tick() noexcept
{
}

void test_mode::on_input(const ps_input_data_type& /* data */) noexcept
{
}

}  // namespace m2::gz_simulation