#!/bin/bash

# Script to run gripper driver tests
# Usage: ./run_tests.sh [unit|integration|manual|all]
export PYTHONPATH="$(pwd)/../src:$PYTHONPATH"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default to all tests
TEST_TYPE=${1:-all}

echo -e "${YELLOW}Robotiq Gripper Driver Test Suite${NC}"
echo "=================================="

# Check if we're in ROS workspace
if [ ! -f "../package.xml" ]; then
    echo -e "${RED}Error: Must be run from the test directory of the ROS package${NC}"
    exit 1
fi

# Check if test file exists
if [ ! -f "test_gripper_driver.py" ]; then
    echo -e "${RED}Error: test_gripper_driver.py not found${NC}"
    exit 1
fi

# Make test file executable
chmod +x test_gripper_driver.py

echo -e "${YELLOW}Running tests...${NC}"

case $TEST_TYPE in
    "unit")
        echo -e "${YELLOW}Running unit tests only${NC}"
        python3 test_gripper_driver.py TestGripperDriver
        ;;
    "integration")
        echo -e "${YELLOW}Running integration tests only${NC}"
        python3 test_gripper_driver.py TestGripperDriverIntegration
        ;;
    "manual")
        echo -e "${YELLOW}Running manual tests (requires ROS environment)${NC}"
        if [ -z "$ROS_PACKAGE_PATH" ]; then
            echo -e "${RED}Error: ROS environment not sourced${NC}"
            echo "Please run: source /opt/ros/noetic/setup.bash"
            echo "           source ~/ros_ws/devel/setup.bash"
            exit 1
        fi
        python3 test_gripper_driver.py manual
        ;;
    "all")
        echo -e "${YELLOW}Running all tests${NC}"
        python3 test_gripper_driver.py
        ;;
    "pytest")
        echo -e "${YELLOW}Running tests with pytest${NC}"
        if command -v pytest &> /dev/null; then
            pytest test_gripper_driver.py -v
        else
            echo -e "${RED}Error: pytest not installed${NC}"
            echo "Install with: pip install pytest"
            exit 1
        fi
        ;;
    *)
        echo -e "${RED}Error: Unknown test type: $TEST_TYPE${NC}"
        echo "Usage: $0 [unit|integration|manual|all|pytest]"
        exit 1
        ;;
esac

# Check exit code
if [ $? -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
else
    echo -e "${RED}Some tests failed!${NC}"
    exit 1
fi