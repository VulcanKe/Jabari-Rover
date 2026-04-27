#!/usr/bin/env python3
"""
Test script for JABARI Manual Control Node

This script provides comprehensive testing capabilities for the manual control node,
including unit tests, integration tests, and interactive testing modes.

Usage:
    # Run all tests
    python3 test_manual_control.py
    
    # Run specific test
    python3 test_manual_control.py --test velocity_limits
    
    # Interactive testing
    python3 test_manual_control.py --interactive
    
    # Performance testing
    python3 test_manual_control.py --performance
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
import time
import json
import threading
import argparse
import sys
from typing import List, Dict, Any


class ManualControlTester(Node):
    """
    Test node for validating manual control node functionality
    """
    
    def __init__(self):
        super().__init__('manual_control_tester')
        
        # Test results storage
        self.test_results = []
        self.received_commands = []
        self.received_status = []
        
        # Publishers for testing
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.external_cmd_pub = self.create_publisher(Twist, '/external_manual_cmd', 10)
        
        # Subscribers for monitoring
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/manual_cmd_vel', self.cmd_vel_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/manual_control_status', self.status_callback, 10)
        
        # Test synchronization
        self.test_lock = threading.Lock()
        self.command_received = threading.Event()
        self.status_received = threading.Event()
        
        self.get_logger().info('Manual Control Tester initialized')
    
    def cmd_vel_callback(self, msg: Twist):
        """Monitor velocity commands from manual control node"""
        with self.test_lock:
            command_data = {
                'timestamp': time.time(),
                'linear_x': msg.linear.x,
                'angular_z': msg.angular.z
            }
            self.received_commands.append(command_data)
            self.command_received.set()
            
            self.get_logger().debug(
                f'Received command: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}'
            )
    
    def status_callback(self, msg: String):
        """Monitor status messages from manual control node"""
        try:
            status_data = json.loads(msg.data)
            with self.test_lock:
                self.received_status.append(status_data)
                self.status_received.set()
                
            self.get_logger().debug(f'Received status: {status_data.get("node_name", "unknown")}')
        except json.JSONDecodeError:
            self.get_logger().error('Invalid status JSON received')
    
    def send_external_command(self, linear: float, angular: float):
        """Send command via external command topic"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.external_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent external command: linear={linear}, angular={angular}')
    
    def send_emergency_stop(self, activate: bool):
        """Send emergency stop command"""
        msg = Bool()
        msg.data = activate
        self.emergency_stop_pub.publish(msg)
        self.get_logger().info(f'Emergency stop: {"ACTIVATED" if activate else "DEACTIVATED"}')
    
    def wait_for_command(self, timeout: float = 5.0) -> bool:
        """Wait for a command to be received"""
        self.command_received.clear()
        return self.command_received.wait(timeout)
    
    def wait_for_status(self, timeout: float = 5.0) -> bool:
        """Wait for a status message to be received"""
        self.status_received.clear()
        return self.status_received.wait(timeout)
    
    def clear_received_data(self):
        """Clear all received data for fresh testing"""
        with self.test_lock:
            self.received_commands.clear()
            self.received_status.clear()
    
    def get_latest_command(self) -> Dict[str, Any]:
        """Get the most recent command received"""
        with self.test_lock:
            if self.received_commands:
                return self.received_commands[-1]
            return {}
    
    def get_latest_status(self) -> Dict[str, Any]:
        """Get the most recent status received"""
        with self.test_lock:
            if self.received_status:
                return self.received_status[-1]
            return {}


class ManualControlTestSuite:
    """
    Comprehensive test suite for manual control node
    """
    
    def __init__(self):
        self.tester = None
        self.executor = None
        self.test_results = []
    
    def setup(self):
        """Setup test environment"""
        rclpy.init()
        self.tester = ManualControlTester()
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.tester)
        
        # Start executor in separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()
        
        # Wait for connections
        time.sleep(2.0)
        print("Test environment initialized")
    
    def teardown(self):
        """Cleanup test environment"""
        if self.executor:
            self.executor.shutdown()
        if self.tester:
            self.tester.destroy_node()
        rclpy.shutdown()
        print("Test environment cleaned up")
    
    def run_test(self, test_name: str, test_func) -> bool:
        """Run a single test and record results"""
        print(f"\n--- Running Test: {test_name} ---")
        
        try:
            self.tester.clear_received_data()
            start_time = time.time()
            
            result = test_func()
            
            end_time = time.time()
            duration = end_time - start_time
            
            self.test_results.append({
                'test_name': test_name,
                'result': result,
                'duration': duration,
                'timestamp': start_time
            })
            
            status = "PASSED" if result else "FAILED"
            print(f"Test {test_name}: {status} (Duration: {duration:.2f}s)")
            
            return result
            
        except Exception as e:
            print(f"Test {test_name}: FAILED (Exception: {e})")
            self.test_results.append({
                'test_name': test_name,
                'result': False,
                'error': str(e),
                'timestamp': time.time()
            })
            return False
    
    def test_basic_communication(self) -> bool:
        """Test basic communication with manual control node"""
        print("Testing basic communication...")
        
        # Send a simple command
        self.tester.send_external_command(0.5, 0.0)
        
        # Wait for command to be processed
        if not self.tester.wait_for_command(timeout=3.0):
            print("ERROR: No command received within timeout")
            return False
        
        # Check received command
        latest_command = self.tester.get_latest_command()
        if not latest_command:
            print("ERROR: No command data available")
            return False
        
        # Validate command values
        expected_linear = 0.5
        expected_angular = 0.0
        
        if (abs(latest_command['linear_x'] - expected_linear) > 0.001 or
            abs(latest_command['angular_z'] - expected_angular) > 0.001):
            print(f"ERROR: Command mismatch. Expected: ({expected_linear}, {expected_angular}), "
                  f"Got: ({latest_command['linear_x']}, {latest_command['angular_z']})")
            return False
        
        print("✓ Basic communication test passed")
        return True
    
    def test_velocity_limits(self) -> bool:
        """Test velocity limiting functionality"""
        print("Testing velocity limits...")
        
        # Test cases: (input_linear, input_angular, expected_max_linear, expected_max_angular)
        test_cases = [
            (5.0, 0.0, 1.0, 0.0),      # Linear limit
            (0.0, 10.0, 0.0, 2.0),     # Angular limit  
            (-5.0, -10.0, -1.0, -2.0), # Negative limits
            (0.1, 0.1, 0.1, 0.1),      # Within limits
        ]
        
        for i, (lin_in, ang_in, lin_exp, ang_exp) in enumerate(test_cases):
            print(f"  Test case {i+1}: input=({lin_in}, {ang_in})")
            
            self.tester.send_external_command(lin_in, ang_in)
            
            if not self.tester.wait_for_command(timeout=2.0):
                print(f"    ERROR: No response for test case {i+1}")
                return False
            
            latest_command = self.tester.get_latest_command()
            lin_actual = latest_command['linear_x']
            ang_actual = latest_command['angular_z']
            
            if (abs(lin_actual - lin_exp) > 0.001 or abs(ang_actual - ang_exp) > 0.001):
                print(f"    ERROR: Expected ({lin_exp}, {ang_exp}), got ({lin_actual}, {ang_actual})")
                return False
            
            print(f"    ✓ Test case {i+1} passed")
        
        print("✓ Velocity limits test passed")
        return True
    
    def test_emergency_stop(self) -> bool:
        """Test emergency stop functionality"""
        print("Testing emergency stop...")
        
        # Send normal command first
        self.tester.send_external_command(0.5, 1.0)
        if not self.tester.wait_for_command(timeout=2.0):
            print("ERROR: Initial command not received")
            return False
        
        # Activate emergency stop
        self.tester.send_emergency_stop(True)
        time.sleep(0.5)  # Allow processing time
        
        # Send another command (should be ignored)
        self.tester.send_external_command(0.8, -1.5)
        
        # Wait and check if zero command was sent (emergency stop response)
        if not self.tester.wait_for_command(timeout=2.0):
            print("ERROR: No emergency stop response received")
            return False
        
        latest_command = self.tester.get_latest_command()
        if (abs(latest_command['linear_x']) > 0.001 or 
            abs(latest_command['angular_z']) > 0.001):
            print(f"ERROR: Emergency stop failed. Received non-zero command: "
                  f"({latest_command['linear_x']}, {latest_command['angular_z']})")
            return False
        
        # Deactivate emergency stop
        self.tester.send_emergency_stop(False)
        time.sleep(0.5)
        
        # Test normal operation resumes
        self.tester.send_external_command(0.3, 0.5)
        if not self.tester.wait_for_command(timeout=2.0):
            print("ERROR: Normal operation not resumed after emergency stop deactivation")
            return False
        
        latest_command = self.tester.get_latest_command()
        if (abs(latest_command['linear_x'] - 0.3) > 0.001 or 
            abs(latest_command['angular_z'] - 0.5) > 0.001):
            print("ERROR: Normal operation not properly resumed")
            return False
        
        print("✓ Emergency stop test passed")
        return True
    
    def test_status_monitoring(self) -> bool:
        """Test status message functionality"""
        print("Testing status monitoring...")
        
        # Wait for status message
        if not self.tester.wait_for_status(timeout=5.0):
            print("ERROR: No status message received")
            return False
        
        status = self.tester.get_latest_status()
        if not status:
            print("ERROR: Empty status message")
            return False
        
        # Check required status fields
        required_fields = [
            'node_name', 'timestamp', 'is_active', 'emergency_stop_active',
            'current_linear_velocity', 'current_angular_velocity', 'command_count'
        ]
        
        for field in required_fields:
            if field not in status:
                print(f"ERROR: Missing required status field: {field}")
                return False
        
        # Validate status content
        if status['node_name'] != 'manual_control_node':
            print(f"ERROR: Incorrect node name: {status['node_name']}")
            return False
        
        if not isinstance(status['is_active'], bool):
            print("ERROR: is_active should be boolean")
            return False
        
        print("✓ Status monitoring test passed")
        return True
    
    def test_performance(self) -> bool:
        """Test performance characteristics"""
        print("Testing performance...")
        
        # Send multiple rapid commands
        num_commands = 100
        start_time = time.time()
        
        for i in range(num_commands):
            linear = 0.1 * (i % 10)
            angular = 0.1 * ((i + 5) % 10)
            self.tester.send_external_command(linear, angular)
            time.sleep(0.01)  # 100Hz command rate
        
        # Wait for all commands to be processed
        time.sleep(2.0)
        
        end_time = time.time()
        duration = end_time - start_time
        
        # Check how many commands were received
        with self.tester.test_lock:
            received_count = len(self.tester.received_commands)
        
        print(f"  Sent {num_commands} commands in {duration:.2f}s")
        print(f"  Received {received_count} command responses")
        print(f"  Command processing rate: {received_count/duration:.1f} Hz")
        
        # Performance criteria (adjust as needed)
        if received_count < num_commands * 0.8:  # At least 80% success rate
            print(f"ERROR: Poor command processing rate: {received_count}/{num_commands}")
            return False
        
        if duration > num_commands * 0.02:  # Should handle faster than 50Hz
            print(f"ERROR: Slow processing time: {duration:.2f}s for {num_commands} commands")
            return False
        
        print("✓ Performance test passed")
        return True
    
    def run_all_tests(self) -> bool:
        """Run complete test suite"""
        print("=== JABARI Manual Control Node Test Suite ===")
        
        tests = [
            ("Basic Communication", self.test_basic_communication),
            ("Velocity Limits", self.test_velocity_limits),
            ("Emergency Stop", self.test_emergency_stop),
            ("Status Monitoring", self.test_status_monitoring),
            ("Performance", self.test_performance),
        ]
        
        passed = 0
        total = len(tests)
        
        for test_name, test_func in tests:
            if self.run_test(test_name, test_func):
                passed += 1
        
        print(f"\n=== Test Results ===")
        print(f"Passed: {passed}/{total}")
        print(f"Success Rate: {passed/total*100:.1f}%")
        
        if passed == total:
            print("🎉 All tests PASSED!")
            return True
        else:
            print("❌ Some tests FAILED!")
            return False
    
    def interactive_test(self):
        """Interactive testing mode"""
        print("\n=== Interactive Testing Mode ===")
        print("Commands:")
        print("  move <linear> <angular> - Send velocity command")
        print("  stop - Send stop command")
        print("  estop - Toggle emergency stop")
        print("  status - Show latest status")
        print("  quit - Exit interactive mode")
        
        emergency_stop_active = False
        
        while True:
            try:
                cmd = input("\nTest> ").strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'quit':
                    break
                elif cmd[0] == 'move' and len(cmd) == 3:
                    linear = float(cmd[1])
                    angular = float(cmd[2])
                    self.tester.send_external_command(linear, angular)
                    print(f"Sent command: linear={linear}, angular={angular}")
                elif cmd[0] == 'stop':
                    self.tester.send_external_command(0.0, 0.0)
                    print("Sent stop command")
                elif cmd[0] == 'estop':
                    emergency_stop_active = not emergency_stop_active
                    self.tester.send_emergency_stop(emergency_stop_active)
                    print(f"Emergency stop: {'ACTIVE' if emergency_stop_active else 'INACTIVE'}")
                elif cmd[0] == 'status':
                    status = self.tester.get_latest_status()
                    if status:
                        print(f"Latest status: {json.dumps(status, indent=2)}")
                    else:
                        print("No status available")
                else:
                    print("Invalid command. Type 'quit' to exit.")
                    
            except KeyboardInterrupt:
                break
            except ValueError:
                print("Invalid number format")
            except Exception as e:
                print(f"Error: {e}")
        
        print("Exiting interactive mode")


def main():
    """Main function for test script"""
    parser = argparse.ArgumentParser(description='Test JABARI Manual Control Node')
    parser.add_argument('--test', type=str, help='Run specific test')
    parser.add_argument('--interactive', action='store_true', help='Interactive testing mode')
    parser.add_argument('--performance', action='store_true', help='Run performance tests only')
    
    args = parser.parse_args()
    
    test_suite = ManualControlTestSuite()
    
    try:
        test_suite.setup()
        
        if args.interactive:
            test_suite.interactive_test()
        elif args.performance:
            test_suite.run_test("Performance Test", test_suite.test_performance)
        elif args.test:
            # Run specific test
            test_methods = {
                'communication': test_suite.test_basic_communication,
                'velocity_limits': test_suite.test_velocity_limits,
                'emergency_stop': test_suite.test_emergency_stop,
                'status': test_suite.test_status_monitoring,
                'performance': test_suite.test_performance,
            }
            
            if args.test in test_methods:
                test_suite.run_test(args.test, test_methods[args.test])
            else:
                print(f"Unknown test: {args.test}")
                print(f"Available tests: {list(test_methods.keys())}")
        else:
            # Run all tests
            success = test_suite.run_all_tests()
            sys.exit(0 if success else 1)
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test error: {e}")
        sys.exit(1)
    finally:
        test_suite.teardown()


if __name__ == '__main__':
    main()