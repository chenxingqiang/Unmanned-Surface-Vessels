"""
Run the waypoint following test for USV controllers.
"""

import os
import sys
import matplotlib.pyplot as plt
import traceback

# Add project root to path to ensure imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

from usv_system.tests.test_controllers import waypoint_following_test

if __name__ == "__main__":
    try:
        print("Starting USV waypoint following test...")
        
        # Create output directory if it doesn't exist
        output_dir = os.path.join(os.path.dirname(__file__), 'outputs')
        os.makedirs(output_dir, exist_ok=True)
        print(f"Output directory: {output_dir}")
        
        # Set backend to avoid GUI issues
        plt.switch_backend('agg')
        
        waypoint_following_test()
        
        # Save the figure explicitly
        plt.savefig(os.path.join(output_dir, 'waypoint_test.png'))
        plt.close()
        
        print(f"Test completed. Plot saved to {os.path.join(output_dir, 'waypoint_test.png')}")
        
    except Exception as e:
        print(f"Error during test: {str(e)}")
        traceback.print_exc() 