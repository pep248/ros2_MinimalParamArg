#!/usr/bin/env python3
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from param_example.src.python_dynamic_param_class import MyPythonDynamicParamClass

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    python_param_node = MyPythonDynamicParamClass('python_param_node')
    
    # Some node require being run on the main thread,
    # therefore we execute the spinner in a different one:
    ros_thread = threading.Thread(target=executor_spin, args=[python_param_node])
    ros_thread.daemon = True
    ros_thread.start()
    
    # Execute the main routine if any
    python_param_node.start()

def executor_spin(node:Node):
    try:
        # Create executor and add nodes
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
