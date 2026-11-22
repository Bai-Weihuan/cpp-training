#include <gtest/gtest.h>

#include <memory>
#include <tuple>

#include "Executor.hpp"

namespace adas {

bool operator==(const Pose& lhs, const Pose& rhs)
{
    return std::tie(lhs.x, lhs.y, lhs.heading) ==
           std::tie(rhs.x, rhs.y, rhs.heading);
}

TEST(ExecutorTest, should_return_init_pose_when_without_command)
{
    std::unique_ptr<Executor> executor(Executor::NewExecutor({3, -2, 'E'}));
    ASSERT_NE(nullptr, executor);
    const Pose expected{3, -2, 'E'};
    EXPECT_EQ(expected, executor->Query());
}

TEST(ExecutorTest, should_return_default_pose_when_without_init_and_command)
{
    std::unique_ptr<Executor> executor(Executor::NewExecutor());
    ASSERT_NE(nullptr, executor);
    const Pose expected{0, 0, 'N'};
    EXPECT_EQ(expected, executor->Query());
}

struct CommandCase {
    const char* name;
    Pose initial;
    const char* commands;
    Pose expected;
};

class ExecutorCommandTest : public ::testing::TestWithParam<CommandCase> {};

TEST_P(ExecutorCommandTest, returns_expected_pose)
{
    const auto& testCase = GetParam();
    std::unique_ptr<Executor> executor(Executor::NewExecutor(testCase.initial));
    ASSERT_NE(nullptr, executor);
    executor->Execute(testCase.commands);
    EXPECT_EQ(testCase.expected, executor->Query());
}

INSTANTIATE_TEST_SUITE_P(
    Commands, ExecutorCommandTest,
    ::testing::Values(
        CommandCase{"move_east", {0, 0, 'E'}, "M", {1, 0, 'E'}},
        CommandCase{"move_west", {0, 0, 'W'}, "M", {-1, 0, 'W'}},
        CommandCase{"move_north", {0, 0, 'N'}, "M", {0, 1, 'N'}},
        CommandCase{"move_south", {0, 0, 'S'}, "M", {0, -1, 'S'}},
        CommandCase{"left_from_east", {2, 3, 'E'}, "L", {2, 3, 'N'}},
        CommandCase{"left_from_north", {2, 3, 'N'}, "L", {2, 3, 'W'}},
        CommandCase{"left_from_west", {2, 3, 'W'}, "L", {2, 3, 'S'}},
        CommandCase{"left_from_south", {2, 3, 'S'}, "L", {2, 3, 'E'}},
        CommandCase{"right_from_east", {2, 3, 'E'}, "R", {2, 3, 'S'}},
        CommandCase{"right_from_south", {2, 3, 'S'}, "R", {2, 3, 'W'}},
        CommandCase{"right_from_west", {2, 3, 'W'}, "R", {2, 3, 'N'}},
        CommandCase{"right_from_north", {2, 3, 'N'}, "R", {2, 3, 'E'}},
        CommandCase{"ppt_sequence", {0, 0, 'N'}, "MLMMRM", {-2, 2, 'N'}},
        CommandCase{"repeated_moves", {4, -3, 'S'}, "MMM", {4, -6, 'S'}},
        CommandCase{"full_rotation", {0, 0, 'N'}, "LLLL", {0, 0, 'N'}}),
    [](const ::testing::TestParamInfo<CommandCase>& info) {
        return info.param.name;
    });

}  // namespace adas
