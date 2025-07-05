#!/usr/bin/env python3
import sys
import os
import unittest
from unittest.mock import Mock, patch, MagicMock
import threading
import time

# Add the scripts directory to Python path
script_dir = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, script_dir)

# Mock ROS modules before importing
sys.modules['rospy'] = Mock()
sys.modules['robotiq_hand_e_control'] = Mock()
sys.modules['robotiq_hand_e_control.msg'] = Mock()

# Mock pyRobotiqGripper driver module
sys.modules['pyRobotiqGripper'] = Mock()
sys.modules['pyRobotiqGripper.driver'] = Mock()

# Mock message types
class MockGripperCommand:
    def __init__(self):
        self.command = ""
        self.arg1 = 0
        self.arg2 = 0
        self.arg3 = 0

class MockGripperStatus:
    def __init__(self):
        self.gACT = 0
        self.gGTO = 0
        self.gSTA = 0
        self.gOBJ = 0
        self.gFLT = 0
        self.gPR = 0
        self.gPO = 0
        self.gCU = 0
        self.gACT_desc = ""
        self.gGTO_desc = ""
        self.gSTA_desc = ""
        self.gOBJ_desc = ""
        self.gFLT_desc = ""
        self.gCU_desc = ""

# Set up mock messages
sys.modules['robotiq_hand_e_control.msg'].GripperCommand = MockGripperCommand
sys.modules['robotiq_hand_e_control.msg'].GripperStatus = MockGripperStatus

# Mock rospy functions
rospy = sys.modules['rospy']
rospy.init_node = Mock()
rospy.Subscriber = Mock()
rospy.Publisher = Mock()
rospy.Rate = Mock()
rospy.spin = Mock()
rospy.is_shutdown = Mock(return_value=False)
rospy.loginfo = Mock()
rospy.logwarn = Mock()
rospy.logerr = Mock()
rospy.logdebug = Mock()
rospy.sleep = Mock()
rospy.ROSInterruptException = Exception

class TestGripperDriver(unittest.TestCase):
    """Test suite for GripperDriverNode"""
    
    def setUp(self):
        """Set up test fixtures"""
        # Reset all mocks
        for mock_obj in [rospy.init_node, rospy.Subscriber, rospy.Publisher, rospy.Rate, 
                        rospy.spin, rospy.is_shutdown, rospy.loginfo, rospy.logwarn, 
                        rospy.logerr, rospy.logdebug, rospy.sleep]:
            mock_obj.reset_mock()
        
        # Mock gripper
        self.mock_gripper = Mock()
        
        # Mock gripper parameters
        self.mock_gripper.paramDic = {
            'gACT': 1, 'gGTO': 1, 'gSTA': 3, 'gOBJ': 0,
            'gFLT': 0, 'gPR': 0, 'gPO': 0, 'gCU': 10
        }
        
        # Mock register dictionary
        self.mock_gripper.registerDic = {
            'gACT': {0: 'Reset', 1: 'Activated'},
            'gGTO': {0: 'Stopped', 1: 'Go to requested position'},
            'gSTA': {0: 'Reset', 1: 'Activation in progress', 2: 'Not used', 3: 'Activation complete'},
            'gOBJ': {0: 'Fingers in motion', 1: 'Stopped due to contact', 2: 'Stopped due to requested position', 3: 'Fingers stopped due to contact'},
            'gFLT': {0: 'No fault', 1: 'Action delayed', 2: 'Undertemperature', 3: 'Overtemperature'},
            'gCU': {10: 'Current 100mA'}
        }
        
        # Mock gripper methods
        self.mock_gripper.activate = Mock()
        self.mock_gripper.open = Mock()
        self.mock_gripper.close = Mock()
        self.mock_gripper.goTo = Mock()
        self.mock_gripper.goTomm = Mock()
        self.mock_gripper.getPosition = Mock(return_value=128)
        self.mock_gripper.getPositionmm = Mock(return_value=50.0)
        self.mock_gripper.reset = Mock()
        self.mock_gripper.resetActivate = Mock()
        self.mock_gripper.calibrate = Mock()
        self.mock_gripper.resetCalibration = Mock()
        self.mock_gripper.printInfo = Mock()
        self.mock_gripper.readAll = Mock()
        
        # Mock Rate
        self.mock_rate = Mock()
        self.mock_rate.sleep = Mock()
        rospy.Rate.return_value = self.mock_rate
        
        # Mock Publisher
        self.mock_publisher = Mock()
        rospy.Publisher.return_value = self.mock_publisher
        
        # Mock Subscriber
        self.mock_subscriber = Mock()
        rospy.Subscriber.return_value = self.mock_subscriber
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_node_initialization(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test node initialization"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        # Import and create node
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Verify initialization calls
            rospy.init_node.assert_called_once_with("gripper_driver_node")
            mock_gripper_class.assert_called_once()
            self.mock_gripper.activate.assert_called_once()
            
            # Verify thread was created and started
            mock_thread.assert_called_once()
            mock_thread_instance.start.assert_called_once()
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found - testing import structure")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_open(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test open command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "open"
            cmd_msg.arg1 = 255
            cmd_msg.arg2 = 100
            cmd_msg.arg3 = 0
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.open was called with correct parameters
            self.mock_gripper.open.assert_called_once_with(speed=255, force=100)
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_close(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test close command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "close"
            cmd_msg.arg1 = 255
            cmd_msg.arg2 = 100
            cmd_msg.arg3 = 0
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.close was called with correct parameters
            self.mock_gripper.close.assert_called_once_with(speed=255, force=100)
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_goto(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test goto command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "goto"
            cmd_msg.arg1 = 128
            cmd_msg.arg2 = 255
            cmd_msg.arg3 = 100
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.goTo was called with correct parameters
            self.mock_gripper.goTo.assert_called_once_with(position=128, speed=255, force=100)
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_goposmm(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test goposmm command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "goposmm"
            cmd_msg.arg1 = 50.0
            cmd_msg.arg2 = 255
            cmd_msg.arg3 = 100
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.goTomm was called with correct parameters
            self.mock_gripper.goTomm.assert_called_once_with(positionmm=50.0, speed=255, force=100)
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_getpos(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test getpos command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "getpos"
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.getPosition was called
            self.mock_gripper.getPosition.assert_called_once()
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_unknown_command(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test unknown command handling"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message with unknown command
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "unknown_command"
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify warning was logged
            rospy.logwarn.assert_called_once()
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_callback_exception_handling(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test exception handling in command callback"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        # Make gripper.open raise an exception
        self.mock_gripper.open.side_effect = Exception("Test exception")
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "open"
            cmd_msg.arg1 = 255
            cmd_msg.arg2 = 100
            
            # Execute command (should not raise exception)
            node.command_callback(cmd_msg)
            
            # Verify error was logged
            rospy.logerr.assert_called_once()
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_publish_status_loop(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test status publishing loop"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Mock is_shutdown to return True after first iteration
            rospy.is_shutdown.side_effect = [False, True]
            
            # Run one iteration of status loop
            node.publish_status_loop()
            
            # Verify readAll was called
            self.mock_gripper.readAll.assert_called_once()
            
            # Verify status was published
            self.mock_publisher.publish.assert_called_once()
            
            # Verify published message is GripperStatus
            published_msg = self.mock_publisher.publish.call_args[0][0]
            self.assertIsInstance(published_msg, MockGripperStatus)
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_calibrate_command(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test calibrate command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "calibrate"
            cmd_msg.arg1 = 0.0
            cmd_msg.arg2 = 85.0
            cmd_msg.arg3 = 0
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.calibrate was called with correct parameters
            self.mock_gripper.calibrate.assert_called_once_with(closemm=0.0, openmm=85.0)
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_reset_activate_command(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test resetactivate command"""
        mock_gripper_class.return_value = self.mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Create command message
            cmd_msg = MockGripperCommand()
            cmd_msg.command = "resetactivate"
            
            # Execute command
            node.command_callback(cmd_msg)
            
            # Verify gripper.resetActivate was called
            self.mock_gripper.resetActivate.assert_called_once()
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")


class TestGripperDriverIntegration(unittest.TestCase):
    """Integration tests for GripperDriverNode"""
    
    def setUp(self):
        """Set up integration test fixtures"""
        # Reset all mocks
        for mock_obj in [rospy.init_node, rospy.Subscriber, rospy.Publisher, rospy.Rate, 
                        rospy.spin, rospy.is_shutdown, rospy.loginfo, rospy.logwarn, 
                        rospy.logerr, rospy.logdebug, rospy.sleep]:
            mock_obj.reset_mock()
        
        self.test_commands = [
            {"command": "open", "arg1": 255, "arg2": 100, "arg3": 0},
            {"command": "close", "arg1": 255, "arg2": 100, "arg3": 0},
            {"command": "goto", "arg1": 128, "arg2": 255, "arg3": 100},
            {"command": "getpos", "arg1": 0, "arg2": 0, "arg3": 0},
            {"command": "reset", "arg1": 0, "arg2": 0, "arg3": 0},
        ]
    
    @patch('pyRobotiqGripper.driver.RobotiqGripper')
    @patch('time.sleep')
    @patch('threading.Thread')
    def test_command_sequence(self, mock_thread, mock_sleep, mock_gripper_class):
        """Test a sequence of commands"""
        mock_gripper = Mock()
        mock_gripper.paramDic = {'gACT': 1, 'gGTO': 1, 'gSTA': 3}
        mock_gripper.registerDic = {'gACT': {1: 'Activated'}}
        mock_gripper.activate = Mock()
        mock_gripper.open = Mock()
        mock_gripper.close = Mock()
        mock_gripper.goTo = Mock()
        mock_gripper.getPosition = Mock(return_value=128)
        mock_gripper.reset = Mock()
        mock_gripper.readAll = Mock()
        
        mock_gripper_class.return_value = mock_gripper
        mock_thread_instance = Mock()
        mock_thread.return_value = mock_thread_instance
        
        try:
            from robotiq_driver_node import GripperDriverNode
            node = GripperDriverNode()
            
            # Execute sequence of commands
            for cmd_data in self.test_commands:
                cmd_msg = MockGripperCommand()
                cmd_msg.command = cmd_data["command"]
                cmd_msg.arg1 = cmd_data["arg1"]
                cmd_msg.arg2 = cmd_data["arg2"]
                cmd_msg.arg3 = cmd_data["arg3"]
                
                # Should not raise exception
                node.command_callback(cmd_msg)
            
            # Verify all commands were processed
            mock_gripper.open.assert_called_once()
            mock_gripper.close.assert_called_once()
            mock_gripper.goTo.assert_called_once()
            mock_gripper.getPosition.assert_called_once()
            mock_gripper.reset.assert_called_once()
            
        except ImportError:
            self.skipTest("robotiq_driver_node module not found")


class TestModuleStructure(unittest.TestCase):
    """Test the module structure and imports"""
    
    def test_script_directory_exists(self):
        """Test that the scripts directory exists"""
        script_dir = os.path.join(os.path.dirname(__file__), '..', 'scripts')
        self.assertTrue(os.path.exists(script_dir), "Scripts directory should exist")
    
    def test_robotiq_driver_node_exists(self):
        """Test that robotiq_driver_node.py exists"""
        script_dir = os.path.join(os.path.dirname(__file__), '..', 'scripts')
        driver_node_path = os.path.join(script_dir, 'robotiq_driver_node.py')
        self.assertTrue(os.path.exists(driver_node_path), "robotiq_driver_node.py should exist")
    
    def test_python_path_setup(self):
        """Test that the Python path is set up correctly"""
        script_dir = os.path.join(os.path.dirname(__file__), '..', 'scripts')
        self.assertIn(script_dir, sys.path, "Scripts directory should be in Python path")
    
    def test_import_attempt(self):
        """Test importing the robotiq_driver_node module"""
        try:
            import robotiq_driver_node
            self.assertTrue(hasattr(robotiq_driver_node, 'GripperDriverNode'), 
                          "Module should have GripperDriverNode class")
        except ImportError as e:
            self.fail(f"Failed to import robotiq_driver_node: {e}")


# Test helper functions
def create_test_node():
    """Helper function to create a test node for manual testing"""
    print("Creating test node...")
    
    # This would be used in a real ROS environment
    import rospy
    from robotiq_hand_e_control.msg import GripperCommand
    
    rospy.init_node('gripper_test_node')
    pub = rospy.Publisher('/gripper_command_topic', GripperCommand, queue_size=10)
    
    # Wait for connection
    rospy.sleep(1)
    
    # Test commands
    commands = [
        ("activate", 0, 0, 0),
        ("open", 255, 100, 0),
        ("close", 255, 100, 0),
        ("goto", 128, 255, 100),
        ("getpos", 0, 0, 0),
        ("reset", 0, 0, 0),
    ]
    
    for cmd, arg1, arg2, arg3 in commands:
        msg = GripperCommand()
        msg.command = cmd
        msg.arg1 = arg1
        msg.arg2 = arg2
        msg.arg3 = arg3
        
        pub.publish(msg)
        print(f"Published command: {cmd}")
        rospy.sleep(2)


if __name__ == '__main__':
    # Run unit tests
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'manual':
        # Manual test mode (requires ROS environment)
        print("Manual test mode requires ROS environment")
        print("Run: rosrun robotiq_hand_e_control test_gripper_driver.py manual")
    else:
        # Unit test mode
        unittest.main()